#!/usr/bin/env python3
"""
lidar.py - LIDAR hardware driver for the RPLidar A1M8.

Provides three functions used by the SLAM process:
  connect()      - open the serial port, reset the sensor, and start the motor
  get_scan_mode() - return the recommended scan mode for this sensor model
  scan_rounds()  - yield one complete 360-degree scan per motor rotation
  disconnect()   - stop the motor and close the serial port

The LIDAR_PORT and LIDAR_BAUD settings live in settings.py.
"""

# this acts as the server 

import mpsv0_connection_params as net_params 
import struct 
import asyncio 

import sys 
from pathlib import Path 
sys.path.insert(1, str(Path(__file__).parent.parent.parent.parent/"project_v0"/"dependencies"))
print(sys.path[1])

import time
from pyrplidar import PyRPlidar

from settings import LIDAR_PORT, LIDAR_BAUD


def connect(port=LIDAR_PORT, baudrate=LIDAR_BAUD):
    """Open the LIDAR serial port, reset the sensor, and start the motor.

    Performing a reset before starting ensures the sensor is in a clean state
    even if it was left running from a previous session.

    Returns the PyRPlidar object on success, or None if the connection fails
    (e.g. wrong port or the device is not plugged in).
    """
    try:
        lidar = PyRPlidar()
        # Connect, reset, wait for the sensor to restart, then reconnect.
        lidar.connect(port=port, baudrate=baudrate, timeout=10)
        lidar.reset()
        time.sleep(2)
        lidar.disconnect()
        lidar.connect(port=port, baudrate=baudrate, timeout=10)
        lidar.set_motor_pwm(500)
        return lidar
    except Exception as exc:
        print(f'[lidar] Could not connect on {port}: {exc}')
        return None


def get_scan_mode(lidar):
    """Return the recommended scan mode index for this LIDAR model.

    Falls back to mode 2 (a safe default for the A1M8) if the query fails.
    """
    try:
        return lidar.get_scan_mode_typical()
    except Exception:
        return 2


def scan_rounds(lidar, mode):
    """Yield one complete 360-degree scan per motor rotation.

    Each yielded value is a (angles, distances) tuple containing two
    parallel lists:
      angles    - float degrees, 0.0 to 360.0
      distances - float mm (0 means no return / out of range)

    The generator runs until the LIDAR is disconnected or an exception is
    raised by the underlying PyRPlidar library.
    """
    buff = []
    started = False
    for meas in lidar.start_scan_express(mode)():
        if meas.start_flag:
            # A start_flag marks the beginning of a new rotation.
            # Yield the completed buffer from the previous rotation.
            if started and buff:
                yield [m.angle for m in buff], [m.distance for m in buff]
            buff = [meas]
            started = True
        elif started:
            buff.append(meas)


def disconnect(lidar):
    """Stop the LIDAR motor and close the serial connection."""
    try:
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()
    except Exception:
        pass




# api handler 

# Global state to manage LIDAR instances and generators
lidar_instances = {}
scan_generators = {}
next_lidar_id = 1
scan_mode_tracker = {}

async def handle_client(reader, writer):
    global next_lidar_id
    addr = writer.get_extra_info('peername')
    print(f"Connected by {addr}")
    client_lidar_id = None

    try:
        while True:
            command_data = await reader.readexactly(4)
            command = struct.unpack('i', command_data)[0]
            
            if command == net_params.CMD_QUIT:
                print("CLIENT TERMINATED CONNECTION") 
                break
            
            elif command == net_params.CMD_CONNECT:
                # Preferred format: [port_len:int32][port:utf8 bytes][baud:int32]
                # Legacy format fallback: [port_as_int:int32][baud:int32]
                first_data = await reader.readexactly(4)
                first_value = struct.unpack('i', first_data)[0]

                if 0 <= first_value <= 1024:
                    port_len = first_value
                    port_bytes = await reader.readexactly(port_len)
                    port = port_bytes.decode('utf-8')
                    baud_data = await reader.readexactly(4)
                    baudrate = struct.unpack('i', baud_data)[0]
                else:
                    port = str(first_value)
                    baud_data = await reader.readexactly(4)
                    baudrate = struct.unpack('i', baud_data)[0]
                
                print("CONNECTING")
                lidar = connect(port=port, baudrate=baudrate)
                if lidar is not None:
                    lidar_id = next_lidar_id
                    next_lidar_id += 1
                    lidar_instances[lidar_id] = lidar
                    client_lidar_id = lidar_id
                    response = struct.pack('<bi', 1, lidar_id)  # success flag + lidar_id
                    print(f"  Connected LIDAR {lidar_id}")
                else:
                    response = struct.pack('<bi', 0, -1)  # failure
                    print(f"  Failed to connect LIDAR")
                writer.write(response)
                await writer.drain()
            
            elif command == net_params.CMD_GET_SCAN_MODE:
                lidar_id_data = await reader.readexactly(4)
                lidar_id = struct.unpack('i', lidar_id_data)[0]
                
                if lidar_id in lidar_instances:
                    mode = get_scan_mode(lidar_instances[lidar_id])
                    response = struct.pack('i', mode)
                    print(f"  Scan mode for LIDAR {lidar_id}: {mode}")
                else:
                    response = struct.pack('i', -1)  # error
                    print(f"  LIDAR {lidar_id} not found")
                writer.write(response)
                await writer.drain()
            
            elif command == net_params.CMD_START_SCAN_ROUNDS:
                lidar_id_data = await reader.readexactly(4)
                lidar_id = struct.unpack('i', lidar_id_data)[0]
                mode_data = await reader.readexactly(4)
                mode = struct.unpack('i', mode_data)[0]
                
                if lidar_id in lidar_instances:
                    lidar = lidar_instances[lidar_id]
                    scan_generators[lidar_id] = scan_rounds(lidar, mode)
                    scan_mode_tracker[lidar_id] = mode
                    response = struct.pack('b', 1)  # success
                    print(f"  Started scan rounds for LIDAR {lidar_id} with mode {mode}")
                else:
                    response = struct.pack('b', 0)  # failure
                    print(f"  LIDAR {lidar_id} not found for scan")
                writer.write(response)
                await writer.drain()
            
            elif command == net_params.CMD_GET_NEXT_SCAN_ROUND:
                lidar_id_data = await reader.readexactly(4)
                lidar_id = struct.unpack('i', lidar_id_data)[0]
                
                if lidar_id in scan_generators:
                    try:
                        angles, distances = next(scan_generators[lidar_id])
                        # Send success flag
                        writer.write(struct.pack('b', 1))
                        # Send count
                        count = len(angles)
                        writer.write(struct.pack('i', count))
                        # Send angles
                        for angle in angles:
                            writer.write(struct.pack('f', angle))
                        # Send distances
                        for distance in distances:
                            writer.write(struct.pack('f', distance))
                        await writer.drain()
                        # don't print anymore to save compute & not clog logs 
                        #print(f"  Sent scan round {count} measurements for LIDAR {lidar_id}")
                    except StopIteration:
                        # Generator exhausted
                        writer.write(struct.pack('b', 0))
                        await writer.drain()
                        print(f"  Scan generator exhausted for LIDAR {lidar_id}")
                else:
                    writer.write(struct.pack('b', 0))  # no data
                    await writer.drain()
                    print(f"  No active scan for LIDAR {lidar_id}")
            
            elif command == net_params.CMD_DISCONNECT:
                lidar_id_data = await reader.readexactly(4)
                lidar_id = struct.unpack('i', lidar_id_data)[0]
                
                if lidar_id in lidar_instances:
                    disconnect(lidar_instances[lidar_id])
                    del lidar_instances[lidar_id]
                    if lidar_id in scan_generators:
                        del scan_generators[lidar_id]
                    if lidar_id in scan_mode_tracker:
                        del scan_mode_tracker[lidar_id]
                    response = struct.pack('b', 1)  # success
                    print(f"  Disconnected LIDAR {lidar_id}")
                else:
                    response = struct.pack('b', 0)  # failure
                    print(f"  LIDAR {lidar_id} not found")
                writer.write(response)
                await writer.drain()

    except asyncio.CancelledError:
        print(f"Client handler task cancelled for {addr}")
    except Exception as e:
        print(f"Error with client {addr}: {e}")
    finally:
        # Clean up any LIDARs connected by this client
        if client_lidar_id and client_lidar_id in lidar_instances:
            try:
                disconnect(lidar_instances[client_lidar_id])
                del lidar_instances[client_lidar_id]
            except:
                pass
        print(f"Closing connection for {addr}")
        writer.close()
        await writer.wait_closed()

async def main():
    server = await asyncio.start_server(
        handle_client, '0.0.0.0', net_params.server_lidar_port)

    addrs = ', '.join(str(sock.getsockname()) for sock in server.sockets)
    print(f'Serving on {addrs}')

    # Run the server indefinitely
    async with server:
        await server.serve_forever()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server stopped manually.")
