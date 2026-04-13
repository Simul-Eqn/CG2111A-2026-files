#!/usr/bin/env python3
"""Second operator terminal client for SLAM main controller relay."""

import os
import ssl
import struct
import sys
import threading
import time
from pathlib import Path

from net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame

sys.path.insert(0, str(Path(__file__).parent.parent))
from packets import *

PI_HOST = os.getenv('SECOND_TERM_HOST', os.getenv('ARM_RELAY_HOST', 'localhost'))
PI_PORT = int(os.getenv('SECOND_TERM_PORT', '65432'))
TLS_ENABLED = os.getenv('TLS_ENABLED', 'True').lower() in ('true', '1', 'yes')
TLS_CERT_PATH = Path(__file__).parent.parent / 'certs' / 'server.crt'
CONNECT_RETRIES = int(os.getenv('SECOND_TERM_CONNECT_RETRIES', '20'))
CONNECT_RETRY_DELAY_SEC = float(os.getenv('SECOND_TERM_CONNECT_RETRY_DELAY_SEC', '1.0'))

_shutdown = threading.Event()


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


def _make_ssl_ctx():
    if not TLS_ENABLED:
        return None
    if not TLS_CERT_PATH.is_file():
        print(f"[second_terminal] TLS cert not found: {TLS_CERT_PATH}")
        sys.exit(1)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_verify_locations(str(TLS_CERT_PATH))
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_REQUIRED
    return ctx


def _receiver(client: TCPClient):
    while not _shutdown.is_set():
        frame = recvTPacketFrame(client.sock)
        if frame is None:
            if not _shutdown.is_set():
                print('[second_terminal] connection closed by relay')
            _shutdown.set()
            break
        pkt = _unpack_frame(frame)
        if pkt is None:
            continue
        if pkt['packetType'] == PACKET_TYPE_RESPONSE and pkt['command'] == RESP_OK:
            print('[robot] OK')
        elif pkt['packetType'] == PACKET_TYPE_RESPONSE and pkt['command'] == RESP_STATUS:
            print(f"[robot] STATUS={pkt['params'][0]}")
        elif pkt['packetType'] == PACKET_TYPE_RESPONSE and pkt['command'] == RESP_COLOR_SENSOR:
            print(f"[robot] COLOR R={pkt['params'][0]} G={pkt['params'][1]} B={pkt['params'][2]}")
        elif pkt['packetType'] == PACKET_TYPE_MESSAGE:
            msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
            print(f'[robot] {msg}')


def _send(client: TCPClient, cmd: int, params=None, data=b''):
    frame = _pack_frame(PACKET_TYPE_COMMAND, cmd, data=data, params=params)
    if not sendTPacketFrame(client.sock, frame):
        print('[second_terminal] send failed')


def run():
    _shutdown.clear()
    client = TCPClient(PI_HOST, PI_PORT, ssl_context=_make_ssl_ctx(), server_hostname=PI_HOST)
    print(f'[second_terminal] connecting to relay at {PI_HOST}:{PI_PORT}...')

    connected = False
    for attempt in range(1, max(1, CONNECT_RETRIES) + 1):
        if client.connect():
            connected = True
            break
        if attempt < CONNECT_RETRIES:
            print(
                f'[second_terminal] connect failed (attempt {attempt}/{CONNECT_RETRIES}); '
                f'retrying in {CONNECT_RETRY_DELAY_SEC:.1f}s...'
            )
            time.sleep(CONNECT_RETRY_DELAY_SEC)

    if not connected:
        print('[second_terminal] failed to connect after retries')
        return

    print('[second_terminal] connected')
    print('[second_terminal] commands: e, c, w, a, s, d, x, +, -, h, b/s/e/g <angle>, v <speed>, set <KEY> <VALUE>, q')

    rx = threading.Thread(target=_receiver, args=(client,), daemon=True)
    rx.start()

    try:
        while not _shutdown.is_set():
            line = input('[second_terminal] > ').strip().lower()
            if not line:
                continue
            tok = line.split()
            cmd = tok[0]
            if cmd == 'q':
                _shutdown.set()
                break
            if cmd == 'e' and len(tok) == 1:
                _send(client, COMMAND_ESTOP, data=b'Second terminal E-Stop')
                continue
            if cmd == 'c':
                _send(client, COMMAND_COLOR_SENSOR)
                continue
            if cmd == 'set' and len(tok) >= 3:
                key = tok[1].strip().upper()
                value = line.split(None, 2)[2].strip()
                expr = f'{key}={value}'
                _send(client, COMMAND_SET_SETTING, data=expr.encode('ascii', errors='replace')[:MAX_STR_LEN])
                continue
            if cmd in ('w', 'a', 's', 'd', 'x'):
                mapping = {
                    'w': COMMAND_FORWARD,
                    'a': COMMAND_LEFT,
                    's': COMMAND_BACKWARD,
                    'd': COMMAND_RIGHT,
                    'x': COMMAND_STOP,
                }
                _send(client, mapping[cmd])
                continue
            if cmd in ('+', '-'):
                speed = 170 if cmd == '+' else 130
                _send(client, COMMAND_SET_SPEED, params=[speed] + [0] * (PARAMS_COUNT - 1))
                continue
            if cmd == 'h':
                _send(client, COMMAND_ARM_HOME)
                continue
            if cmd == 'v' and len(tok) == 2 and tok[1].isdigit():
                v = int(tok[1])
                _send(client, COMMAND_ARM_SET_SPEED, params=[v] + [0] * (PARAMS_COUNT - 1))
                continue
            if cmd in ('b', 'z', 'g') and len(tok) == 2 and tok[1].isdigit():
                v = int(tok[1])
                mapping = {
                    'b': COMMAND_ARM_BASE,
                    'z': COMMAND_ARM_SHOULDER,
                    'g': COMMAND_ARM_GRIPPER,
                }
                _send(client, mapping[cmd], params=[v] + [0] * (PARAMS_COUNT - 1))
                continue
            if cmd == 'e' and len(tok) == 2 and tok[1].isdigit():
                v = int(tok[1])
                _send(client, COMMAND_ARM_ELBOW, params=[v] + [0] * (PARAMS_COUNT - 1))
                continue
            print('[second_terminal] invalid command')
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown.set()
        client.close()
        print('[second_terminal] bye')


if __name__ == '__main__':
    run()
