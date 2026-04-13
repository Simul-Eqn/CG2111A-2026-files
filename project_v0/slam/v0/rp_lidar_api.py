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
import serial
import os

import sys 
from pathlib import Path 
sys.path.insert(1, str(Path(__file__).parent.parent.parent.parent/"project_v0"/"dependencies"))
print(sys.path[1])

USE_EXPRESS_SCAN = "--express" in sys.argv

import time
from pyrplidar import PyRPlidar
import camera_handler as CameraHandler

from packets import *

from settings import LIDAR_PORT, LIDAR_BAUD


ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 9600

ANS_TYPE_NORMAL = 0x81


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
        typical = lidar.get_scan_mode_typical()
        if USE_EXPRESS_SCAN:
            return typical

        # Non-express decoding requires NORMAL ans_type frames.
        scan_modes = lidar.get_scan_modes()
        for idx, scan_mode in enumerate(scan_modes):
            if getattr(scan_mode, 'ans_type', None) == ANS_TYPE_NORMAL:
                _debug_log(f"non-express selected NORMAL mode={idx} instead of typical={typical}")
                return idx

        _debug_log(f"non-express NORMAL mode not found, fallback typical={typical}")
        return typical
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
    scan_api = "start_scan_express" if USE_EXPRESS_SCAN else "start_scan"
    _debug_log(f"scan_rounds start api={scan_api} mode={mode}")

    if USE_EXPRESS_SCAN:
        scan_iter = lidar.start_scan_express(mode)()
    else:
        scan_iter = lidar.start_scan()()

    buff = []
    started = False
    round_index = 0
    for meas in scan_iter:
        if meas.start_flag:
            # A start_flag marks the beginning of a new rotation.
            # Yield the completed buffer from the previous rotation.
            if started and buff:
                round_index += 1
                angles = [m.angle for m in buff]
                distances = [m.distance for m in buff]

                if USE_EXPRESS_SCAN:
                    filter_enable = EXPRESS_FILTER_ENABLE
                    max_distance_mm = EXPRESS_MAX_DISTANCE_MM
                    filter_profile = "express"
                else:
                    filter_enable = NONEXPRESS_FILTER_ENABLE
                    max_distance_mm = NONEXPRESS_MAX_DISTANCE_MM
                    filter_profile = "normal"

                if filter_enable:
                    filtered_angles = []
                    filtered_distances = []
                    wrapped_angles = 0
                    dropped_distance = 0
                    dropped_angle = 0

                    for angle, distance in zip(angles, distances):
                        if distance < 0 or distance > max_distance_mm:
                            dropped_distance += 1
                            continue

                        if -ANGLE_WRAP_TOLERANCE_DEG <= angle < 0.0:
                            angle += 360.0
                            wrapped_angles += 1
                        elif 360.0 <= angle < (360.0 + ANGLE_WRAP_TOLERANCE_DEG):
                            angle -= 360.0
                            wrapped_angles += 1
                        elif angle < 0.0 or angle >= 360.0:
                            dropped_angle += 1
                            continue

                        filtered_angles.append(angle)
                        filtered_distances.append(distance)

                    if wrapped_angles > 0 or dropped_angle > 0 or dropped_distance > 0:
                        _debug_log(
                            f"scan_round sanitize profile={filter_profile} idx={round_index} wrapped={wrapped_angles} "
                            f"drop_angle={dropped_angle} drop_distance={dropped_distance} "
                            f"before={len(angles)} after={len(filtered_angles)} mode={mode}"
                        )

                    angles = filtered_angles
                    distances = filtered_distances

                count = len(distances)
                valid_nonzero = sum(1 for d in distances if d > 0)

                if count == 0:
                    _debug_log(
                        f"scan_round skipped_empty idx={round_index} mode={mode} "
                        f"raw_count={len(buff)}"
                    )
                    buff = [meas]
                    started = True
                    continue

                if count <= 1:
                    one_angle = angles[0]
                    one_distance = distances[0]
                    one_quality = getattr(buff[0], 'quality', 'n/a')
                    _debug_log(
                        f"scan_round suspicious count={count} valid_nonzero={valid_nonzero} "
                        f"angle={one_angle:.2f} distance={one_distance:.2f} quality={one_quality} "
                        f"mode={mode}"
                    )
                elif any((a < 0.0 or a > 360.0) for a in angles):
                    outlier_angles = [a for a in angles if (a < 0.0 or a > 360.0)]
                    _debug_log(
                        f"scan_round angle_out_of_range idx={round_index} count={count} "
                        f"outliers={len(outlier_angles)} first={outlier_angles[0]:.2f} mode={mode}"
                    )
                elif round_index % SCAN_DEBUG_EVERY_N == 0:
                    min_d = min(distances)
                    max_d = max(distances)
                    _debug_log(
                        f"scan_round ok idx={round_index} count={count} valid_nonzero={valid_nonzero} "
                        f"dist_range=[{min_d:.2f},{max_d:.2f}] mode={mode}"
                    )

                yield angles, distances
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


_arduino_ser = None
_estop_state = STATE_RUNNING
_camera_connected = False
CAMERA_CAPTURE_LIMIT = 15
_frames_remaining = CAMERA_CAPTURE_LIMIT
_motor_speed = 150

DEBUG_LIDAR = "--debug" in sys.argv #os.getenv("RP_LIDAR_DEBUG", "1") not in ("0", "false", "False", "no", "NO")
SCAN_DEBUG_EVERY_N = max(1, int(os.getenv("RP_LIDAR_SCAN_DEBUG_EVERY_N", "20")))
NETWORK_SLOW_DRAIN_MS = float(os.getenv("RP_LIDAR_SLOW_DRAIN_MS", "50"))
EXPRESS_FILTER_ENABLE = os.getenv("RP_LIDAR_EXPRESS_FILTER", "1") not in ("0", "false", "False", "no", "NO")
EXPRESS_MAX_DISTANCE_MM = float(os.getenv("RP_LIDAR_EXPRESS_MAX_DISTANCE_MM", "12000"))
NONEXPRESS_FILTER_ENABLE = os.getenv("RP_LIDAR_NONEXPRESS_FILTER", "1") not in ("0", "false", "False", "no", "NO")
NONEXPRESS_MAX_DISTANCE_MM = float(os.getenv("RP_LIDAR_NONEXPRESS_MAX_DISTANCE_MM", "12000"))
ANGLE_WRAP_TOLERANCE_DEG = float(os.getenv("RP_LIDAR_ANGLE_WRAP_TOLERANCE_DEG", "1.0"))


def _debug_log(msg):
    if DEBUG_LIDAR:
        print(f"[lidar-debug] {msg}")


def _debug_scan_mode_details(lidar, mode):
    try:
        scan_modes = lidar.get_scan_modes()
        if 0 <= mode < len(scan_modes):
            selected = scan_modes[mode]
            ans_type = getattr(selected, 'ans_type', None)
            ans_type_int = int(ans_type) if isinstance(ans_type, int) else -1
            ans_name = {
                0x81: 'NORMAL',
                0x82: 'CAPSULED',
                0x83: 'HQ',
                0x84: 'ULTRA_CAPSULED',
                0x85: 'DENSE_CAPSULED',
            }.get(ans_type_int, f'UNKNOWN({ans_type})')
            if DEBUG_LIDAR:
                _debug_log(
                    f"scan mode details mode={mode} ans_type=0x{ans_type_int:02X}({ans_name}) "
                    f"us_per_sample={getattr(selected, 'us_per_sample', 'n/a')} "
                    f"max_distance={getattr(selected, 'max_distance', 'n/a')} "
                    f"run_express={USE_EXPRESS_SCAN}"
                )
            return ans_type_int, ans_name
        else:
            if DEBUG_LIDAR:
                _debug_log(f"scan mode details mode={mode} out_of_range total_modes={len(scan_modes)}")
    except Exception as exc:
        if DEBUG_LIDAR:
            _debug_log(f"scan mode details unavailable mode={mode} err={exc}")
    return None, None


def _compute_checksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result


def _pack_frame(packet_type, command, data=b'', params=None):
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packet_type, command, data_padded, *params)
    checksum = _compute_checksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def _unpack_tpacket(raw):
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command': fields[1],
        'data': fields[2],
        'params': list(fields[3:]),
    }


def _open_arduino_serial(port=ARDUINO_PORT, baudrate=ARDUINO_BAUD):
    global _arduino_ser
    if _arduino_ser and _arduino_ser.is_open:
        return True
    try:
        _arduino_ser = serial.Serial(port, baudrate, timeout=0.2)
        time.sleep(2)
        _arduino_ser.reset_input_buffer()
        _arduino_ser.reset_output_buffer()
        return True
    except Exception as exc:
        print(f"[sensor] Failed to open serial {port}@{baudrate}: {exc}")
        _arduino_ser = None
        return False


def _close_arduino_serial():
    global _arduino_ser
    try:
        if _arduino_ser and _arduino_ser.is_open:
            _arduino_ser.close()
    finally:
        _arduino_ser = None


def _serial_ready():
    return _arduino_ser is not None and _arduino_ser.is_open


def _receive_frame():
    if not _serial_ready():
        return None

    magic_hi = MAGIC[0]
    magic_lo = MAGIC[1]

    while True:
        b = _arduino_ser.read(1)
        if not b:
            return None
        if b[0] != magic_hi:
            continue

        b = _arduino_ser.read(1)
        if not b:
            return None
        if b[0] != magic_lo:
            continue

        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _arduino_ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        cs_byte = _arduino_ser.read(1)
        if not cs_byte:
            return None
        if cs_byte[0] != _compute_checksum(raw):
            continue

        return _unpack_tpacket(raw)


def _consume_packet_state(pkt):
    global _estop_state
    if pkt['packetType'] == PACKET_TYPE_RESPONSE and pkt['command'] == RESP_STATUS:
        _estop_state = pkt['params'][0]


def _await_response(expected_cmd=None, timeout_sec=1.0):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        pkt = _receive_frame()
        if not pkt:
            continue
        _consume_packet_state(pkt)
        if expected_cmd is None:
            return pkt
        if pkt['packetType'] == PACKET_TYPE_RESPONSE and pkt['command'] == expected_cmd:
            return pkt
    return None


def _send_sensor_command(command_type, data=b'', params=None):
    if not _serial_ready():
        return False
    frame = _pack_frame(PACKET_TYPE_COMMAND, command_type, data=data, params=params)
    _arduino_ser.write(frame)
    return True


def _is_estop_active():
    return _estop_state == STATE_STOPPED


def _ensure_camera_connected():
    global _camera_connected
    if _camera_connected:
        return True
    try:
        CameraHandler.camera_connect()
        _camera_connected = True
        return True
    except Exception as exc:
        print(f"[sensor] Camera connect failed: {exc}")
        return False


def _parse_arm_text_command(text):
    if not isinstance(text, str):
        return None
    cmd = text.strip().upper()
    if not cmd:
        return None

    if cmd == 'H':
        return COMMAND_ARM_HOME, None

    if len(cmd) != 4 or not cmd[1:].isdigit():
        return None

    key = cmd[0]
    value = int(cmd[1:])
    mapping = {
        'B': COMMAND_ARM_BASE,
        'S': COMMAND_ARM_SHOULDER,
        'E': COMMAND_ARM_ELBOW,
        'G': COMMAND_ARM_GRIPPER,
        'V': COMMAND_ARM_SET_SPEED,
    }
    pkt_cmd = mapping.get(key)
    if pkt_cmd is None:
        return None
    return pkt_cmd, value




# api handler 

# Global state to manage LIDAR instances and generators
lidar_instances = {}
scan_generators = {}
next_lidar_id = 1
scan_mode_tracker = {}
scan_runtime_stats = {}


def _scan_stats_for(lidar_id):
    if lidar_id not in scan_runtime_stats:
        scan_runtime_stats[lidar_id] = {
            'requests': 0,
            'rounds_sent': 0,
            'one_point_rounds': 0,
            'last_request_ts': None,
            'last_send_ts': None,
            'bytes_sent': 0,
        }
    return scan_runtime_stats[lidar_id]

async def handle_client(reader, writer):
    global next_lidar_id, _frames_remaining, _motor_speed
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
                    ans_type_int, ans_name = _debug_scan_mode_details(lidar, mode)
                    if (ans_type_int in (0x82, 0x83, 0x84, 0x85)) and (not USE_EXPRESS_SCAN):
                        print(
                            f"[lidar-warning] mode={mode} ans_type={ans_name} requires --express; "
                            f"current run_express={USE_EXPRESS_SCAN} may produce malformed scans"
                        )
                    if (ans_type_int == 0x81) and USE_EXPRESS_SCAN:
                        print(
                            f"[lidar-warning] mode={mode} ans_type={ans_name} is NORMAL; "
                            f"running with --express may be suboptimal"
                        )
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
                stats = _scan_stats_for(lidar_id)
                now = time.time()
                stats['requests'] += 1
                if stats['last_request_ts'] is not None and stats['requests'] % SCAN_DEBUG_EVERY_N == 0:
                    dt_ms = (now - stats['last_request_ts']) * 1000.0
                    _debug_log(f"net request cadence lidar={lidar_id} req={stats['requests']} dt={dt_ms:.1f}ms")
                stats['last_request_ts'] = now
                
                if lidar_id in scan_generators:
                    try:
                        t_get0 = time.perf_counter()
                        angles, distances = next(scan_generators[lidar_id])
                        t_get1 = time.perf_counter()
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
                        t_drain0 = time.perf_counter()
                        await writer.drain()
                        t_drain1 = time.perf_counter()

                        payload_bytes = 1 + 4 + (count * 4) + (count * 4)
                        stats['rounds_sent'] += 1
                        stats['bytes_sent'] += payload_bytes
                        stats['last_send_ts'] = time.time()
                        if count <= 1:
                            stats['one_point_rounds'] += 1

                        if count <= 1 or stats['rounds_sent'] % SCAN_DEBUG_EVERY_N == 0:
                            valid_nonzero = sum(1 for d in distances if d > 0)
                            get_ms = (t_get1 - t_get0) * 1000.0
                            drain_ms = (t_drain1 - t_drain0) * 1000.0
                            transport = writer.transport
                            write_buf = transport.get_write_buffer_size() if transport else -1
                            _debug_log(
                                f"net send lidar={lidar_id} rounds={stats['rounds_sent']} count={count} "
                                f"valid_nonzero={valid_nonzero} payload={payload_bytes}B get={get_ms:.2f}ms "
                                f"drain={drain_ms:.2f}ms write_buf={write_buf} one_point_total={stats['one_point_rounds']}"
                            )
                            if drain_ms > NETWORK_SLOW_DRAIN_MS:
                                _debug_log(
                                    f"net warning slow drain lidar={lidar_id} drain={drain_ms:.2f}ms "
                                    f"threshold={NETWORK_SLOW_DRAIN_MS:.2f}ms"
                                )
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
                    if lidar_id in scan_runtime_stats:
                        stats = scan_runtime_stats.pop(lidar_id)
                        _debug_log(
                            f"disconnect stats lidar={lidar_id} req={stats['requests']} sent={stats['rounds_sent']} "
                            f"one_point={stats['one_point_rounds']} bytes={stats['bytes_sent']}"
                        )
                    response = struct.pack('b', 1)  # success
                    print(f"  Disconnected LIDAR {lidar_id}")
                else:
                    response = struct.pack('b', 0)  # failure
                    print(f"  LIDAR {lidar_id} not found")
                writer.write(response)
                await writer.drain()

            elif command == net_params.CMD_SENSOR_SERIAL_CONNECT:
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

                ok = _open_arduino_serial(port=port, baudrate=baudrate)
                #if ok:
                #    _frames_remaining = CAMERA_CAPTURE_LIMIT
                writer.write(struct.pack('b', 1 if ok else 0))
                await writer.drain()

            elif command == net_params.CMD_SENSOR_SERIAL_DISCONNECT:
                _close_arduino_serial()
                writer.write(struct.pack('b', 1))
                await writer.drain()

            elif command == net_params.CMD_SENSOR_ESTOP:
                ok = _send_sensor_command(COMMAND_ESTOP, data=b'Remote E-Stop')
                if ok:
                    _await_response(expected_cmd=None, timeout_sec=0.6)
                writer.write(struct.pack('b', 1 if ok else 0))
                await writer.drain()

            elif command == net_params.CMD_SENSOR_GET_STATUS:
                writer.write(struct.pack('<bi', 1, int(_estop_state)))
                await writer.drain()

            elif command == net_params.CMD_SENSOR_GET_COLOR:
                if _is_estop_active():
                    writer.write(struct.pack('<biii', 0, 0, 0, 0))
                    await writer.drain()
                    continue

                ok = _send_sensor_command(COMMAND_COLOR_SENSOR)
                if not ok:
                    writer.write(struct.pack('<biii', 0, 0, 0, 0))
                    await writer.drain()
                    continue

                pkt = _await_response(expected_cmd=RESP_COLOR_SENSOR, timeout_sec=1.2)
                if pkt:
                    r, g, b = int(pkt['params'][0]), int(pkt['params'][1]), int(pkt['params'][2])
                    writer.write(struct.pack('<biii', 1, r, g, b))
                else:
                    writer.write(struct.pack('<biii', 0, 0, 0, 0))
                await writer.drain()

            elif command == net_params.CMD_SENSOR_DRIVE:
                drive_cmd_data = await reader.readexactly(4)
                drive_cmd = struct.unpack('i', drive_cmd_data)[0]

                allowed = {
                    COMMAND_FORWARD,
                    COMMAND_BACKWARD,
                    COMMAND_LEFT,
                    COMMAND_RIGHT,
                    COMMAND_STOP,
                }
                ok = drive_cmd in allowed and (not _is_estop_active()) and _send_sensor_command(drive_cmd)
                if ok:
                    _await_response(expected_cmd=None, timeout_sec=0.5)
                writer.write(struct.pack('b', 1 if ok else 0))
                await writer.drain()

            elif command == net_params.CMD_SENSOR_SET_SPEED:
                speed_data = await reader.readexactly(4)
                speed = struct.unpack('i', speed_data)[0]
                speed = max(0, min(255, speed))

                ok = (not _is_estop_active()) and _send_sensor_command(
                    COMMAND_SET_SPEED,
                    params=[speed] + [0] * (PARAMS_COUNT - 1),
                )
                if ok:
                    _motor_speed = speed
                    _await_response(expected_cmd=None, timeout_sec=0.5)
                writer.write(struct.pack('b', 1 if ok else 0))
                await writer.drain()

            elif command == net_params.CMD_SENSOR_CAMERA_CAPTURE:
                if _is_estop_active() or _frames_remaining <= 0:
                    writer.write(struct.pack('b', 0))
                    await writer.drain()
                    continue

                if not _ensure_camera_connected():
                    writer.write(struct.pack('b', 0))
                    await writer.drain()
                    continue

                try:
                    ok = bool(CameraHandler.camera_capture())
                except Exception as exc:
                    print(f"[sensor] Camera capture failed: {exc}")
                    ok = False

                if ok:
                    _frames_remaining -= 1

                writer.write(struct.pack('b', 1 if ok else 0))
                await writer.drain()

            elif command == net_params.CMD_SENSOR_ARM_TEXT:
                cmd_len = struct.unpack('i', await reader.readexactly(4))[0]
                cmd_text = (await reader.readexactly(cmd_len)).decode('utf-8', errors='replace')

                parsed = _parse_arm_text_command(cmd_text)
                if parsed is None or _is_estop_active():
                    writer.write(struct.pack('b', 0))
                    await writer.drain()
                    continue

                pkt_cmd, value = parsed
                if value is None:
                    ok = _send_sensor_command(pkt_cmd)
                else:
                    ok = _send_sensor_command(pkt_cmd, params=[value] + [0] * (PARAMS_COUNT - 1))

                if ok:
                    _await_response(expected_cmd=None, timeout_sec=0.5)
                writer.write(struct.pack('b', 1 if ok else 0))
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
                if client_lidar_id in scan_runtime_stats:
                    stats = scan_runtime_stats.pop(client_lidar_id)
                    _debug_log(
                        f"client-close stats lidar={client_lidar_id} req={stats['requests']} sent={stats['rounds_sent']} "
                        f"one_point={stats['one_point_rounds']} bytes={stats['bytes_sent']}"
                    )
            except:
                pass
        _close_arduino_serial()
        if _camera_connected:
            try:
                CameraHandler.camera_close()
            except Exception:
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
        print(f"[lidar] Starting rp_lidar_api with express={USE_EXPRESS_SCAN}")
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server stopped manually.")
