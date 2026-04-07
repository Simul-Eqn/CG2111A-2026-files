#!/usr/bin/env python3
"""Main-controller side second terminal manager.

Receives framed TPacket commands from second_terminal relay and processes them
through lidar client helpers in this process.
"""

import struct
from pathlib import Path
import sys

import lidar
from packets import *
from second_terminal import relay


_estop_state = STATE_RUNNING
_motor_speed = motor_speed


def _compute_checksum(data: bytes) -> int:
    out = 0
    for b in data:
        out ^= b
    return out


def _pack_frame(packet_type, command, data=b'', params=None):
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    payload = struct.pack(TPACKET_FMT, packet_type, command, data_padded, *params)
    return MAGIC + payload + bytes([_compute_checksum(payload)])


def _unpack_frame(frame: bytes):
    if len(frame) != FRAME_SIZE or frame[:2] != MAGIC:
        return None
    raw = frame[2:2 + TPACKET_SIZE]
    if frame[-1] != _compute_checksum(raw):
        return None
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command': fields[1],
        'data': fields[2],
        'params': list(fields[3:]),
    }


def _send_ok():
    relay.onPacketReceived(_pack_frame(PACKET_TYPE_RESPONSE, RESP_OK))


def _send_status():
    relay.onPacketReceived(
        _pack_frame(PACKET_TYPE_RESPONSE, RESP_STATUS, params=[int(_estop_state)] + [0] * (PARAMS_COUNT - 1))
    )


def _send_color(r: int, g: int, b: int):
    # params[3] is coarse color index for compatibility: 0=red,1=green,2=blue
    m = min((r, 'r'), (g, 'g'), (b, 'b'))[1]
    idx = 0 if m == 'r' else (1 if m == 'g' else 2)
    params = [int(r), int(g), int(b), idx] + [0] * (PARAMS_COUNT - 4)
    relay.onPacketReceived(_pack_frame(PACKET_TYPE_RESPONSE, RESP_COLOR_SENSOR, params=params))


def _send_msg(text: str):
    relay.onPacketReceived(_pack_frame(PACKET_TYPE_MESSAGE, 0, data=text.encode('ascii', errors='replace')[:MAX_STR_LEN]))


def _arm_cmd_text(command: int, value: int | None):
    if command == COMMAND_ARM_HOME:
        return 'H'
    prefix = {
        COMMAND_ARM_BASE: 'B',
        COMMAND_ARM_SHOULDER: 'S',
        COMMAND_ARM_ELBOW: 'E',
        COMMAND_ARM_GRIPPER: 'G',
        COMMAND_ARM_SET_SPEED: 'V',
    }.get(command)
    if prefix is None or value is None:
        return None
    value = max(0, min(999, int(value)))
    return f'{prefix}{value:03d}'


def _process_command(pkt: dict):
    global _estop_state, _motor_speed

    cmd = int(pkt['command'])
    p0 = int(pkt['params'][0]) if pkt['params'] else 0

    if cmd == COMMAND_ESTOP:
        ok = lidar.sensor_estop()
        if ok:
            _estop_state = STATE_STOPPED
            _send_ok()
            _send_status()
        else:
            _send_msg('E-STOP failed')
        return

    if cmd == COMMAND_COLOR_SENSOR:
        rgb = lidar.sensor_get_color()
        if rgb is None:
            _send_msg('Color read failed')
            return
        _send_color(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return

    if cmd == COMMAND_FORWARD:
        ok = lidar.sensor_forward()
        _send_ok() if ok else _send_msg('Forward failed')
        return
    if cmd == COMMAND_BACKWARD:
        ok = lidar.sensor_backward()
        _send_ok() if ok else _send_msg('Backward failed')
        return
    if cmd == COMMAND_LEFT:
        ok = lidar.sensor_left()
        _send_ok() if ok else _send_msg('Left failed')
        return
    if cmd == COMMAND_RIGHT:
        ok = lidar.sensor_right()
        _send_ok() if ok else _send_msg('Right failed')
        return
    if cmd == COMMAND_STOP:
        ok = lidar.sensor_stop()
        _send_ok() if ok else _send_msg('Stop failed')
        return

    if cmd == COMMAND_SET_SPEED:
        _motor_speed = max(0, min(255, p0))
        ok = lidar.sensor_set_speed(_motor_speed)
        if ok:
            params = [_motor_speed] + [0] * (PARAMS_COUNT - 1)
            relay.onPacketReceived(_pack_frame(PACKET_TYPE_RESPONSE, RESP_MOTOR_STATUS, params=params))
        else:
            _send_msg('Set speed failed')
        return

    if cmd in (COMMAND_ARM_BASE, COMMAND_ARM_SHOULDER, COMMAND_ARM_ELBOW, COMMAND_ARM_GRIPPER, COMMAND_ARM_HOME, COMMAND_ARM_SET_SPEED):
        txt = _arm_cmd_text(cmd, None if cmd == COMMAND_ARM_HOME else p0)
        if txt is None:
            _send_msg('Invalid ARM cmd')
            return
        ok = lidar.sensor_arm_text(txt)
        _send_ok() if ok else _send_msg(f'Arm cmd failed: {txt}')
        return

    _send_msg(f'Unknown cmd {cmd}')


def pump():
    """Non-blocking poll for one second-terminal frame and process it."""
    frame = relay.recvFromSecondTerminal()
    if frame is None:
        return

    pkt = _unpack_frame(frame)
    if pkt is None:
        _send_msg('Dropped invalid frame')
        return
    if pkt['packetType'] != PACKET_TYPE_COMMAND:
        _send_msg('Dropped non-command frame')
        return

    _process_command(pkt)


def start():
    relay.start()


def shutdown():
    relay.shutdown()
