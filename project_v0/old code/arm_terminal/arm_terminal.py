#!/usr/bin/env python3
"""
arm_terminal.py  -  Arm control terminal for the robot.

This terminal connects to the arm relay running in slam.py over TCP/TLS.
It allows the user to send arm commands interactively.

Commands
--------
  b <angle>    Set base to angle (0-180)
  s <angle>    Set shoulder to angle (70-120)
  e <angle>    Set elbow to angle (60-120)
  g <angle>    Set gripper to angle (70-100)
  v <speed>    Set arm speed (1-50)
  h            Home (move to default position)
  x            Stop motors
  e            E-Stop
  q            Quit

Example usage
-------------
    cd project_v0/slam/v0
    python3 arm_terminal/arm_terminal.py

The terminal connects to localhost:65433 by default. Set ARM_RELAY_HOST
and ARM_RELAY_PORT environment variables to connect to a remote system.

Usage
-----
    ARM_RELAY_HOST=127.0.0.1 python3 arm_terminal/arm_terminal.py
"""

import os
import ssl
import struct
import sys
import threading
import time
from pathlib import Path

# Import network utilities from the same package
from net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame

# Try to import default configuration from settings
try:
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from settings import ARM_SERVER_IP_DEFAULT, ARM_RELAY_PORT_DEFAULT
except ImportError:
    # Fallback defaults if settings.py is not available
    ARM_SERVER_IP_DEFAULT = '100.71.68.106'
    ARM_RELAY_PORT_DEFAULT = 65433


# ---------------------------------------------------------------------------
# Configuration (from environment or defaults)
# ---------------------------------------------------------------------------

ARM_RELAY_HOST = os.getenv('ARM_RELAY_HOST', ARM_SERVER_IP_DEFAULT)
ARM_RELAY_PORT = int(os.getenv('ARM_RELAY_PORT', str(ARM_RELAY_PORT_DEFAULT)))
TLS_ENABLED = os.getenv('TLS_ENABLED', 'True').lower() in ('true', '1', 'yes')

# Certificate path for TLS
TLS_CERT_PATH = Path(__file__).parent.parent / 'certs' / 'server.crt'


# ---------------------------------------------------------------------------
# TPacket Constants (must match packets.py)
# ---------------------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP = 0
COMMAND_ARM_BASE = 20
COMMAND_ARM_SHOULDER = 21
COMMAND_ARM_ELBOW = 22
COMMAND_ARM_GRIPPER = 23
COMMAND_ARM_HOME = 24
COMMAND_ARM_SET_SPEED = 25
COMMAND_STOP = 7

RESP_OK = 0
RESP_STATUS = 1
RESP_ARM_STATUS = 4

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN = 32
PARAMS_COUNT = 16

TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)
TPACKET_FMT = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

MAGIC = b'\xDE\xAD'
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1

# Arm safety ranges
ARM_BASE_MIN = 0
ARM_BASE_MAX = 180
ARM_SHOULDER_MIN = 70
ARM_SHOULDER_MAX = 120
ARM_ELBOW_MIN = 60
ARM_ELBOW_MAX = 120
ARM_GRIPPER_MIN = 70
ARM_GRIPPER_MAX = 100
ARM_SPEED_MIN = 1
ARM_SPEED_MAX = 50


# ---------------------------------------------------------------------------
# TPacket helpers
# ---------------------------------------------------------------------------

def _computeChecksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result


def _packFrame(packetType, command, data=b'', params=None):
    """Pack a TPacket into a 103-byte framed byte string."""
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    return MAGIC + packet_bytes + bytes([_computeChecksum(packet_bytes)])


def _unpackFrame(frame: bytes):
    """Validate checksum and unpack a 103-byte frame. Returns None if corrupt."""
    if len(frame) != FRAME_SIZE or frame[:2] != MAGIC:
        return None
    raw = frame[2:2 + TPACKET_SIZE]
    if frame[-1] != _computeChecksum(raw):
        return None
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command': fields[1],
        'data': fields[2],
        'params': list(fields[3:]),
    }


# ---------------------------------------------------------------------------
# Packet display & handling
# ---------------------------------------------------------------------------

_estop_active = False
_shutdown = threading.Event()


def _printPacket(pkt):
    """Pretty-print a TPacket received from the robot."""
    global _estop_active

    ptype = pkt['packetType']
    cmd = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("[robot] OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            print(f"[robot] Status: {'STOPPED' if _estop_active else 'RUNNING'}")
        elif cmd == RESP_ARM_STATUS:
            b, s, e, g, v = pkt['params'][0:5]
            tb, ts, te, tg = pkt['params'][5:9]
            print(
                f"[robot] Arm status: pos(B={b} S={s} E={e} G={g}) "
                f"target(B={tb} S={ts} E={te} G={tg}) V={v}"
            )
        else:
            print(f"[robot] Response: unknown command {cmd}")
    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"[robot] Message: {msg}")
    else:
        print(f"[robot] Packet: type={ptype}, cmd={cmd}")


# ---------------------------------------------------------------------------
# Command input handling
# ---------------------------------------------------------------------------

def _arm_range_for(cmd: str):
    """Return (min, max, pack_command, label) for an arm joint command."""
    if cmd == 'b':
        return ARM_BASE_MIN, ARM_BASE_MAX, COMMAND_ARM_BASE, "BASE"
    if cmd == 's':
        return ARM_SHOULDER_MIN, ARM_SHOULDER_MAX, COMMAND_ARM_SHOULDER, "SHOULDER"
    if cmd == 'e':
        return ARM_ELBOW_MIN, ARM_ELBOW_MAX, COMMAND_ARM_ELBOW, "ELBOW"
    if cmd == 'g':
        return ARM_GRIPPER_MIN, ARM_GRIPPER_MAX, COMMAND_ARM_GRIPPER, "GRIPPER"
    return None


def _send_arm_command(client: TCPClient, command: int, value: int = None, label: str = ""):
    """Send an arm command frame to the relay."""
    params = [0] * PARAMS_COUNT
    if value is not None:
        params[0] = value
    frame = _packFrame(PACKET_TYPE_COMMAND, command, params=params)
    if sendTPacketFrame(client.sock, frame):
        if value is not None:
            print(f"[arm_terminal] Sent {label}={value}")
        else:
            print(f"[arm_terminal] Sent {label}")
    else:
        print("[arm_terminal] Failed to send command")


def _handleInput(line: str, client: TCPClient):
    """Handle one line of keyboard input."""
    line = line.strip().lower()
    if not line:
        return

    tokens = line.split()
    cmd = tokens[0]

    if cmd == 'q':
        print("[arm_terminal] Quitting.")
        _shutdown.set()
        return

    if cmd == 'e' and len(tokens) == 1:
        # Special case: 'e' alone is E-Stop
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP, data=b'ARM Terminal E-Stop')
        if sendTPacketFrame(client.sock, frame):
            print("[arm_terminal] Sent E-STOP")
        else:
            print("[arm_terminal] Failed to send E-STOP")
        return

    if cmd == 'h':
        _send_arm_command(client, COMMAND_ARM_HOME, label="ARM_HOME")
        return

    if cmd == 'v':
        if len(tokens) != 2:
            print(f"[arm_terminal] Usage: v <{ARM_SPEED_MIN}-{ARM_SPEED_MAX}>")
            return
        try:
            speed = int(tokens[1])
        except ValueError:
            print("[arm_terminal] Speed must be an integer")
            return
        if speed < ARM_SPEED_MIN or speed > ARM_SPEED_MAX:
            print(f"[arm_terminal] Speed out of range: {ARM_SPEED_MIN}-{ARM_SPEED_MAX}")
            return
        _send_arm_command(client, COMMAND_ARM_SET_SPEED, speed, "SPEED")
        return

    if cmd == 'x':
        _send_arm_command(client, COMMAND_STOP, label="STOP")
        return

    arm = _arm_range_for(cmd)
    if arm:
        if len(tokens) != 2:
            lo, hi, _, _ = arm
            print(f"[arm_terminal] Usage: {cmd} <{lo}-{hi}>")
            return
        try:
            angle = int(tokens[1])
        except ValueError:
            print("[arm_terminal] Angle must be an integer")
            return
        lo, hi, pkt_cmd, label = arm
        if angle < lo or angle > hi:
            print(f"[arm_terminal] {label} out of range: {lo}-{hi}")
            return
        _send_arm_command(client, pkt_cmd, angle, label)
        return

    print("[arm_terminal] Unknown command. Valid: b/s/e/g <angle>, h, v <speed>, x, q, e")
    print("[arm_terminal]   b <0-180>     : set base")
    print("[arm_terminal]   s <70-120>    : set shoulder")
    print("[arm_terminal]   e <60-120>    : set elbow")
    print("[arm_terminal]   g <70-100>    : set gripper")
    print("[arm_terminal]   v <1-50>      : set speed")
    print("[arm_terminal]   h             : home position")
    print("[arm_terminal]   x             : stop motors")
    print("[arm_terminal]   e             : emergency stop")
    print("[arm_terminal]   q             : quit")


# ---------------------------------------------------------------------------
# Receiver thread
# ---------------------------------------------------------------------------

def _receiver_loop(client: TCPClient):
    """Background thread that receives and displays packets from the robot."""
    while not _shutdown.is_set():
        try:
            frame = recvTPacketFrame(client.sock)
            if frame is None:
                if not _shutdown.is_set():
                    print("[arm_terminal] Connection closed.")
                _shutdown.set()
                break

            pkt = _unpackFrame(frame)
            if pkt:
                _printPacket(pkt)
        except Exception as err:
            if not _shutdown.is_set():
                print(f"[arm_terminal] Receiver error: {err}")
            _shutdown.set()
            break


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _make_client_ssl_context():
    """Create a TLS client context for connecting to the relay."""
    if not TLS_ENABLED:
        return None
    try:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ctx.minimum_version = ssl.TLSVersion.TLSv1_2
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        if TLS_CERT_PATH.is_file():
            ctx.load_verify_locations(TLS_CERT_PATH)
            ctx.verify_mode = ssl.CERT_REQUIRED
        return ctx
    except (FileNotFoundError, ssl.SSLError) as err:
        print(f"[arm_terminal] Warning: TLS setup failed: {err}")
        print(f"[arm_terminal]   Expected cert at: {TLS_CERT_PATH}")
        return None


def main():
    print(f"[arm_terminal] Connecting to relay at {ARM_RELAY_HOST}:{ARM_RELAY_PORT}...")
    print(f"[arm_terminal] TLS: {'enabled' if TLS_ENABLED else 'disabled'}")

    ssl_context = _make_client_ssl_context() if TLS_ENABLED else None
    client = TCPClient(ARM_RELAY_HOST, ARM_RELAY_PORT, ssl_context=ssl_context)

    if not client.connect():
        print("[arm_terminal] Failed to connect to relay. Is slam.py running?")
        sys.exit(1)

    print("[arm_terminal] Connected to relay!")
    print("[arm_terminal] Commands: b/s/e/g <angle>, h, v <speed>, x, e, q")
    print("[arm_terminal] Type 'q' to quit, or press Ctrl+C\n")

    # Start receiver thread
    receiver_thread = threading.Thread(target=_receiver_loop, args=(client,), daemon=True)
    receiver_thread.start()

    # Main input loop
    try:
        while not _shutdown.is_set():
            try:
                line = input("[arm_terminal] > ")
                _handleInput(line, client)
            except EOFError:
                print("[arm_terminal] EOF (Ctrl+D). Quitting.")
                _shutdown.set()
                break
    except KeyboardInterrupt:
        print("\n[arm_terminal] Interrupted. Disconnecting.")
        _shutdown.set()
    finally:
        client.close()
        print("[arm_terminal] Disconnected.")


if __name__ == '__main__':
    main()
