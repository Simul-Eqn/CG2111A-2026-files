#!/usr/bin/env python3
"""
lidar.py - Client-side LIDAR API for remote RPLidar A1M8 access.

Provides the same interface as the server rp_lidar_api.py:
  connect()      - open the serial port, reset the sensor, and start the motor
  get_scan_mode() - return the recommended scan mode for this sensor model
  scan_rounds()  - yield one complete 360-degree scan per motor rotation
  disconnect()   - stop the motor and close the serial connection

All operations are performed remotely via network to rp_lidar_api.py server.
Uses threading to provide synchronous interface while maintaining async networking.
"""

import mpsv0_connection_params as net_params
import struct
import asyncio
import threading

from typing import Optional, Tuple, List, Generator
from settings import LIDAR_PORT, LIDAR_BAUD

# Global connection state and event loop
_reader = None
_writer = None
_connected = False
_loop = None
_loop_thread = None
_lock = threading.Lock()

from log_to_file import _client_log 

def _run_in_background_loop(coro):
    """Run an async coroutine in the background event loop."""
    global _loop
    if _loop is None:
        raise RuntimeError("Event loop not initialized")
    # Schedule the coroutine and wait for it to complete
    future = asyncio.run_coroutine_threadsafe(coro, _loop)
    return future.result()


def _start_background_loop():
    """Start the background event loop in a separate thread."""
    global _loop, _loop_thread
    if _loop is not None:
        return
    
    def _loop_runner():
        global _loop
        _loop = asyncio.new_event_loop()
        asyncio.set_event_loop(_loop)
        _loop.run_forever()
    
    _loop_thread = threading.Thread(target=_loop_runner, daemon=True)
    _loop_thread.start()
    # Give the thread a moment to start
    import time
    time.sleep(0.1)


# Start the background loop when this module is imported
_start_background_loop()


async def _ensure_connection():
    """Ensure we have a connection to the server."""
    global _reader, _writer, _connected
    if not _connected:
        try:
            _reader, _writer = await asyncio.open_connection(
                net_params.server_IP, net_params.server_lidar_port)
            _connected = True
            _client_log(
                f"Connected to LIDAR server at {net_params.server_IP}:{net_params.server_lidar_port}"
            )
        except Exception as e:
            _client_log(f"Failed to connect to LIDAR server: {e}")
            raise


async def _send_command(command, *args):
    """Send a command to the server and receive response."""
    await _ensure_connection()
    
    # Send command
    _writer.write(struct.pack('i', command))
    
    # Send arguments
    for arg in args:
        if isinstance(arg, int):
            _writer.write(struct.pack('i', arg))
        elif isinstance(arg, float):
            _writer.write(struct.pack('f', arg))
        elif isinstance(arg, str):
            encoded = arg.encode('utf-8')
            _writer.write(struct.pack('i', len(encoded)))
            _writer.write(encoded)
    
    await _writer.drain()


async def _connect_async(port=LIDAR_PORT, baudrate=LIDAR_BAUD):
    """Async implementation of connect."""
    try:
        _client_log(f"[lidar] connect request port={port} baudrate={baudrate}")
        await _send_command(net_params.CMD_CONNECT, port, baudrate)
        
        # Read response: success flag (1 byte) + lidar_id (4 bytes)
        response = await _reader.readexactly(5)
        success = struct.unpack('<b', response[0:1])[0]
        lidar_id = struct.unpack('<i', response[1:5])[0]
        
        if success:
            _client_log(f'[lidar] Connected successfully, LIDAR ID: {lidar_id}')
            return lidar_id
        else:
            _client_log(f'[lidar] Could not connect on port {port}')
            return None
    except Exception as e:
        _client_log(f'[lidar] Connection error: {e}')
        return None


async def _get_scan_mode_async(lidar_id):
    """Async implementation of get_scan_mode."""
    try:
        await _send_command(net_params.CMD_GET_SCAN_MODE, lidar_id)
        
        # Read response: mode (4 bytes)
        mode_data = await _reader.readexactly(4)
        mode = struct.unpack('i', mode_data)[0]
        
        if mode >= 0:
            return mode
        else:
            _client_log(f"[lidar] Failed to get scan mode for LIDAR {lidar_id}, using default mode 2")
            return 2
    except Exception as e:
        _client_log(f"[lidar] Error getting scan mode: {e}")
        return 2


async def _start_scan_rounds_async(lidar_id, mode):
    """Start the scan round generator on the server."""
    try:
        await _send_command(net_params.CMD_START_SCAN_ROUNDS, lidar_id, mode)
        
        # Read response: success flag (1 byte)
        response = await _reader.readexactly(1)
        success = struct.unpack('b', response)[0]
        
        return bool(success)
    except Exception as e:
        _client_log(f"[lidar] Error starting scan rounds: {e}")
        return False


async def _get_next_scan_round_async(lidar_id):
    """Get the next scan round from the server."""
    try:
        await _send_command(net_params.CMD_GET_NEXT_SCAN_ROUND, lidar_id)
        
        # Read response: has_data flag (1 byte)
        has_data_data = await _reader.readexactly(1)
        has_data = struct.unpack('b', has_data_data)[0]
        
        if not has_data:
            return None  # No more data
        
        # Read count (4 bytes)
        count_data = await _reader.readexactly(4)
        count = struct.unpack('i', count_data)[0]
        
        # Read angles
        angles = []
        for _ in range(count):
            angle_data = await _reader.readexactly(4)
            angle = struct.unpack('f', angle_data)[0]
            angles.append(angle)
        
        # Read distances
        distances = []
        for _ in range(count):
            distance_data = await _reader.readexactly(4)
            distance = struct.unpack('f', distance_data)[0]
            distances.append(distance)
        
        return angles, distances
    except Exception as e:
        _client_log(f"[lidar] Error getting scan round: {e}")
        return None


async def _disconnect_async(lidar_id):
    """Async implementation of disconnect."""
    try:
        await _send_command(net_params.CMD_DISCONNECT, lidar_id)
        
        # Read response: success flag (1 byte)
        response = await _reader.readexactly(1)
        success = struct.unpack('b', response)[0]
        
        if success:
            _client_log(f'[lidar] LIDAR {lidar_id} disconnected successfully')
        else:
            _client_log(f'[lidar] Failed to disconnect LIDAR {lidar_id}')
    except Exception as e:
        _client_log(f'[lidar] Error disconnecting: {e}')


# ============================================================================
# PUBLIC SYNCHRONOUS API (100% compatible with original interface)
# ============================================================================

def connect(port=LIDAR_PORT, baudrate=LIDAR_BAUD):
    """Open the LIDAR serial port, reset the sensor, and start the motor.

    Performing a reset before starting ensures the sensor is in a clean state
    even if it was left running from a previous session.

    Returns the LIDAR ID on success, or None if the connection fails
    (e.g. wrong port or the device is not plugged in).
    """
    return _run_in_background_loop(_connect_async(port, baudrate))


def get_scan_mode(lidar_id):
    """Return the recommended scan mode index for this LIDAR model.

    Falls back to mode 2 (a safe default for the A1M8) if the query fails.
    """
    return _run_in_background_loop(_get_scan_mode_async(lidar_id))


def scan_rounds(lidar_id, mode) -> Generator[Tuple[List[float], List[float]], None, None]:
    """Yield one complete 360-degree scan per motor rotation.

    Each yielded value is a (angles, distances) tuple containing two
    parallel lists:
      angles    - float degrees, 0.0 to 360.0
      distances - float mm (0 means no return / out of range)

    The generator runs until the LIDAR is disconnected or an exception is
    raised by the server.
    """
    # Start scan on server
    if not _run_in_background_loop(_start_scan_rounds_async(lidar_id, mode)):
        _client_log(f"[lidar] Failed to start scan rounds for LIDAR {lidar_id}")
        return
    
    # Yield scans in a loop
    while True:
        result = _run_in_background_loop(_get_next_scan_round_async(lidar_id))
        if result is None:
            break
        angles, distances = result
        yield angles, distances


def disconnect(lidar_id):
    """Stop the LIDAR motor and close the serial connection."""
    return _run_in_background_loop(_disconnect_async(lidar_id))


