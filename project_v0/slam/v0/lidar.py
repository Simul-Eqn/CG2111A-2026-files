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
import time

from typing import Optional, Tuple, List, Generator
from settings import LIDAR_PORT, LIDAR_BAUD, LIDAR_POLL_DELAY
from packets import (
    COMMAND_FORWARD,
    COMMAND_BACKWARD,
    COMMAND_LEFT,
    COMMAND_RIGHT,
    COMMAND_STOP,
)

ARDUINO_PORT = '/dev/ttyACM0'
ARDUINO_BAUD = 9600
SERVER_OPEN_TIMEOUT_SEC = 3.0
SERVER_RESPONSE_TIMEOUT_SEC = 5.0

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
            _reader, _writer = await asyncio.wait_for(
                asyncio.open_connection(
                    net_params.server_IP,
                    net_params.server_lidar_port,
                ),
                timeout=SERVER_OPEN_TIMEOUT_SEC,
            )
            _connected = True
            _client_log(
                f"Connected to LIDAR server at {net_params.server_IP}:{net_params.server_lidar_port}"
            )
        except Exception as e:
            await _reset_connection()
            _client_log(f"Failed to connect to LIDAR server: {e}")
            raise


async def _reset_connection():
    """Drop the current TCP connection so the next command reconnects cleanly."""
    global _reader, _writer, _connected
    writer = _writer
    _reader = None
    _writer = None
    _connected = False
    if writer is not None:
        writer.close()
        try:
            await writer.wait_closed()
        except Exception:
            pass


async def _readexactly(num_bytes, *, context, timeout_sec=SERVER_RESPONSE_TIMEOUT_SEC):
    """Read a fixed-size server response with a finite timeout."""
    try:
        return await asyncio.wait_for(
            _reader.readexactly(num_bytes),
            timeout=timeout_sec,
        )
    except asyncio.TimeoutError as exc:
        await _reset_connection()
        raise TimeoutError(
            f"Timed out waiting for {context} after {timeout_sec:.1f}s"
        ) from exc
    except Exception:
        await _reset_connection()
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
    
    try:
        await _writer.drain()
    except Exception:
        await _reset_connection()
        raise


async def _connect_async(port=LIDAR_PORT, baudrate=LIDAR_BAUD):
    """Async implementation of connect."""
    try:
        _client_log(f"[lidar] connect request port={port} baudrate={baudrate}")
        await _send_command(net_params.CMD_CONNECT, port, baudrate)
        
        # Read response: success flag (1 byte) + lidar_id (4 bytes)
        response = await _readexactly(5, context="CMD_CONNECT response")
        success = struct.unpack('<b', response[0:1])[0]
        lidar_id = struct.unpack('<i', response[1:5])[0]
        
        if success:
            _client_log(f'[lidar] Connected successfully, LIDAR ID: {lidar_id}')
            return lidar_id
        else:
            _client_log(f'[lidar] Could not connect on port {port}')
            return None
    except TimeoutError as e:
        _client_log(
            "[lidar] Timed out waiting for CMD_CONNECT response. "
            "This usually means the Raspberry Pi is still running an older "
            "rp_lidar_api.py or the server is wedged; redeploy the current "
            "project_v0/slam/v0/rp_lidar_api.py and restart it."
        )
        _client_log(f"[lidar] Connection timeout detail: {e}")
        return None
    except Exception as e:
        _client_log(f'[lidar] Connection error: {e}')
        return None


async def _get_scan_mode_async(lidar_id):
    """Async implementation of get_scan_mode."""
    try:
        await _send_command(net_params.CMD_GET_SCAN_MODE, lidar_id)
        
        # Read response: mode (4 bytes)
        mode_data = await _readexactly(4, context="scan-mode response")
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
        response = await _readexactly(1, context="start-scan response")
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
        has_data_data = await _readexactly(1, context="scan-round availability flag")
        has_data = struct.unpack('b', has_data_data)[0]
        
        if not has_data:
            return None  # No more data
        
        # Read count (4 bytes)
        count_data = await _readexactly(4, context="scan-round count")
        count = struct.unpack('i', count_data)[0]
        
        # Read angles
        angles = []
        for _ in range(count):
            angle_data = await _readexactly(4, context="scan-round angle value")
            angle = struct.unpack('f', angle_data)[0]
            angles.append(angle)
        
        # Read distances
        distances = []
        for _ in range(count):
            distance_data = await _readexactly(4, context="scan-round distance value")
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
        response = await _readexactly(1, context="disconnect response")
        success = struct.unpack('b', response)[0]
        
        if success:
            _client_log(f'[lidar] LIDAR {lidar_id} disconnected successfully')
        else:
            _client_log(f'[lidar] Failed to disconnect LIDAR {lidar_id}')
    except Exception as e:
        _client_log(f'[lidar] Error disconnecting: {e}')


async def _sensor_serial_connect_async(port=ARDUINO_PORT, baudrate=ARDUINO_BAUD):
    try:
        await _send_command(net_params.CMD_SENSOR_SERIAL_CONNECT, port, baudrate)
        response = await _readexactly(1, context="sensor-serial-connect response")
        return bool(struct.unpack('b', response)[0])
    except Exception as e:
        _client_log(f"[sensor] Serial connect failed: {e}")
        return False


async def _sensor_serial_disconnect_async():
    try:
        await _send_command(net_params.CMD_SENSOR_SERIAL_DISCONNECT)
        response = await _readexactly(1, context="sensor-serial-disconnect response")
        return bool(struct.unpack('b', response)[0])
    except Exception as e:
        _client_log(f"[sensor] Serial disconnect failed: {e}")
        return False


async def _sensor_estop_async():
    try:
        await _send_command(net_params.CMD_SENSOR_ESTOP)
        response = await _readexactly(1, context="sensor-estop response")
        return bool(struct.unpack('b', response)[0])
    except Exception as e:
        _client_log(f"[sensor] E-Stop failed: {e}")
        return False


async def _sensor_get_status_async():
    try:
        await _send_command(net_params.CMD_SENSOR_GET_STATUS)
        response = await _readexactly(5, context="sensor-status response")
        success = struct.unpack('<b', response[0:1])[0]
        state = struct.unpack('<i', response[1:5])[0]
        if not success:
            return None
        return state
    except Exception as e:
        _client_log(f"[sensor] Get status failed: {e}")
        return None


async def _sensor_get_color_async():
    try:
        await _send_command(net_params.CMD_SENSOR_GET_COLOR)
        response = await _readexactly(13, context="sensor-color response")
        success, r, g, b = struct.unpack('<biii', response)
        if not success:
            return None
        return r, g, b
    except Exception as e:
        _client_log(f"[sensor] Get color failed: {e}")
        return None


async def _sensor_drive_async(drive_cmd):
    try:
        await _send_command(net_params.CMD_SENSOR_DRIVE, drive_cmd)
        response = await _readexactly(1, context="sensor-drive response")
        return bool(struct.unpack('b', response)[0])
    except Exception as e:
        _client_log(f"[sensor] Drive command failed: {e}")
        return False


async def _sensor_set_speed_async(speed):
    try:
        speed = max(0, min(255, int(speed)))
        await _send_command(net_params.CMD_SENSOR_SET_SPEED, speed)
        response = await _readexactly(1, context="sensor-set-speed response")
        return bool(struct.unpack('b', response)[0])
    except Exception as e:
        _client_log(f"[sensor] Set speed failed: {e}")
        return False


async def _sensor_camera_capture_async():
    try:
        await _send_command(net_params.CMD_SENSOR_CAMERA_CAPTURE)
        response = await _readexactly(1, context="sensor-camera response")
        return bool(struct.unpack('b', response)[0])
    except Exception as e:
        _client_log(f"[sensor] Camera capture failed: {e}")
        return False


async def _sensor_arm_text_async(text_cmd):
    try:
        await _send_command(net_params.CMD_SENSOR_ARM_TEXT, str(text_cmd))
        response = await _readexactly(1, context="sensor-arm-text response")
        return bool(struct.unpack('b', response)[0])
    except Exception as e:
        _client_log(f"[sensor] Arm text command failed: {e}")
        return False


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

    # Wait between requests using a direct delay value in seconds.
    poll_delay_sec = float(LIDAR_POLL_DELAY)
    
    # Yield scans in a loop
    while True:
        result = _run_in_background_loop(_get_next_scan_round_async(lidar_id))
        if result is None:
            break
        angles, distances = result
        yield angles, distances
        if poll_delay_sec > 0.0:
            time.sleep(poll_delay_sec)


def disconnect(lidar_id):
    """Stop the LIDAR motor and close the serial connection."""
    return _run_in_background_loop(_disconnect_async(lidar_id))


def sensor_serial_connect(port=ARDUINO_PORT, baudrate=ARDUINO_BAUD):
    """Connect rp_lidar_api.py to the Arduino sensor serial link."""
    return _run_in_background_loop(_sensor_serial_connect_async(port, baudrate))


def sensor_serial_disconnect():
    """Disconnect rp_lidar_api.py from the Arduino sensor serial link."""
    return _run_in_background_loop(_sensor_serial_disconnect_async())


def sensor_estop():
    """Trigger software E-Stop on the Arduino side."""
    return _run_in_background_loop(_sensor_estop_async())


def sensor_get_status():
    """Return current E-Stop state as integer (STATE_RUNNING or STATE_STOPPED)."""
    return _run_in_background_loop(_sensor_get_status_async())


def sensor_get_color():
    """Request one color-sensor reading; returns (r, g, b) in Hz or None."""
    return _run_in_background_loop(_sensor_get_color_async())


def sensor_drive(command):
    """Send a movement command (FORWARD/BACKWARD/LEFT/RIGHT/STOP)."""
    return _run_in_background_loop(_sensor_drive_async(command))


def sensor_forward():
    return sensor_drive(COMMAND_FORWARD)


def sensor_backward():
    return sensor_drive(COMMAND_BACKWARD)


def sensor_left():
    return sensor_drive(COMMAND_LEFT)


def sensor_right():
    return sensor_drive(COMMAND_RIGHT)


def sensor_stop():
    return sensor_drive(COMMAND_STOP)


def sensor_set_speed(speed):
    """Set motor speed in range [0, 255]."""
    return _run_in_background_loop(_sensor_set_speed_async(speed))


def sensor_camera_capture():
    """Capture and forward one camera frame using camera_handler."""
    return _run_in_background_loop(_sensor_camera_capture_async())


def sensor_arm_text(text_cmd):
    """Send arm command in X000 form, where X in {B,S,E,G,H,V}."""
    if not isinstance(text_cmd, str):
        return False
    cmd = text_cmd.strip().upper()
    if not cmd:
        return False

    if len(cmd) == 1:
        return _run_in_background_loop(_sensor_arm_text_async(cmd))

    if len(cmd) != 4:
        return False
    if cmd[0] not in {'B', 'S', 'E', 'G', 'V'}:
        return False
    if not cmd[1:].isdigit():
        return False
    return _run_in_background_loop(_sensor_arm_text_async(cmd))

