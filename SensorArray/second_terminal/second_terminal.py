#!/usr/bin/env python3
"""
Studio 17: Transport Layer Security (TLS)
second_terminal.py  -  Second operator terminal client.

This terminal connects to the second-terminal relay on the Raspberry Pi over
TCP / TLS. It:
  - Displays every TPacket forwarded by the relay from the robot.
  - Sends a software E-Stop command when you type 'e'.

Recommended usage for this studio:
  - Run pi_sensor.py on the Raspberry Pi.
  - Run this client from a WebTop terminal.
  - Set PI_HOST to the Pi's Tailscale IP address (recommended), or to the
    exact Tailscale hostname you plan to verify in the certificate.

IMPORTANT: Update the TPacket constants below to match your pi_sensor.py.
---------------------------------------------------------------------------
The packet constants (PACKET_TYPE_*, COMMAND_*, RESP_*, STATE_*, sizes) are
duplicated here from pi_sensor.py.  They MUST stay in sync with your
pi_sensor.py (and with the Arduino sketch).  Update them whenever you change
your protocol.

Tip: consider abstracting all TPacket constants into a shared file (e.g.
packets.py) that both pi_sensor.py and second_terminal.py import, so there
is only one place to update them.  You do not have to do this now, but it
avoids hard-to-find bugs caused by constants getting out of sync.

Commands
--------
  e   Send a software E-Stop to the robot. For this studio, the packet also
      carries the text "secret information" in its data field so it is easy
      to spot in a plain-TCP capture.
  q   Quit.

Usage
-----
    python3 second_terminal/second_terminal.py

Press Ctrl+C to exit.
"""

import select
import ssl
import struct
import sys
import threading
from pathlib import Path

# net_utils is imported with an absolute import because this script is designed
# to be run directly (python3 second_terminal/second_terminal.py), which adds
# this file's directory to sys.path automatically.
from net_utils import TCPClient, sendTPacketFrame, recvTPacketFrame


# ---------------------------------------------------------------------------
# Connection settings
# ---------------------------------------------------------------------------
# Change PI_HOST to the Pi's Tailscale IP address or verified hostname when
# running this client from WebTop or another remote machine.
PI_HOST = 'raspberrypi'
PI_PORT = 65432
TLS_ENABLED = True

TLS_CERT_PATH = Path(__file__).parent.parent/'certs'/'server.crt'

# Demo payload used in Studio 17 so plaintext TCP captures show a readable
# application string before TLS is enabled.
TCPDUMP_DEMO_TEXT = b'secret information'


# import packets 
sys.path.insert(1, str(Path(__file__).parent.parent)) 
from packets import * 
# ---------------------------------------------------------------------------
# TPacket constants
# ---------------------------------------------------------------------------
# IMPORTANT: keep these in sync with your pi_sensor.py and the Arduino sketch.
'''
PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP = 0

RESP_OK     = 0
RESP_STATUS = 1

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16
TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)   # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

MAGIC      = b'\xDE\xAD'
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # = 103
'''

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
    data_padded  = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    return MAGIC + packet_bytes + bytes([_computeChecksum(packet_bytes)])


def _unpackFrame(frame: bytes):
    """Validate checksum and unpack a 103-byte frame.  Returns None if corrupt."""
    if len(frame) != FRAME_SIZE or frame[:2] != MAGIC:
        return None
    raw = frame[2:2 + TPACKET_SIZE]
    if frame[-1] != _computeChecksum(raw):
        return None
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


# ---------------------------------------------------------------------------
# Packet display
# ---------------------------------------------------------------------------

_estop_active = False
_shutdown = threading.Event()


def _printPacket(pkt):
    """Pretty-print a TPacket forwarded from the robot."""
    global _estop_active

    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("[robot] OK")
        elif cmd == RESP_STATUS:
            state         = pkt['params'][0]
            _estop_active = (state == STATE_STOPPED)
            print(f"[robot] Status: {'STOPPED' if _estop_active else 'RUNNING'}")
        elif cmd == RESP_ARM_STATUS:
            b, s, e, g, v = pkt['params'][0:5]
            print(f"[robot] Arm status: B={b} S={s} E={e} G={g} V={v}")
        else:
            print(f"[robot] Response: unknown command {cmd}")
        # Print any debug string embedded in the data field.
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"[robot] Debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"[robot] Message: {msg}")

    else:
        print(f"[robot] Packet: type={ptype}, cmd={cmd}")


# ---------------------------------------------------------------------------
# Input handling
# ---------------------------------------------------------------------------

def _arm_range_for(cmd: str):
    if cmd == 'b':
        return ARM_BASE_MIN, ARM_BASE_MAX, COMMAND_ARM_BASE, "BASE"
    if cmd == 's':
        return ARM_SHOULDER_MIN, ARM_SHOULDER_MAX, COMMAND_ARM_SHOULDER, "SHOULDER"
    if cmd == 'e':
        return ARM_ELBOW_MIN, ARM_ELBOW_MAX, COMMAND_ARM_ELBOW, "ELBOW"
    if cmd == 'g':
        return ARM_GRIPPER_MIN, ARM_GRIPPER_MAX, COMMAND_ARM_GRIPPER, "GRIPPER"
    return None


def _send_arm_command(client: TCPClient, command: int, value: int, label: str):
    params = [0] * PARAMS_COUNT
    params[0] = value
    frame = _packFrame(PACKET_TYPE_COMMAND, command, params=params)
    if sendTPacketFrame(client.sock, frame):
        print(f"[second_terminal] Sent {label}={value}")
    else:
        print("[second_terminal] Failed to send arm command")


def _handleInput(line: str, client: TCPClient):
    """Handle one line of keyboard input."""
    line = line.strip().lower()
    if not line:
        return

    tokens = line.split()
    cmd = tokens[0]
    
    if line == 'e':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ESTOP,
                           data=TCPDUMP_DEMO_TEXT)
        sendTPacketFrame(client.sock, frame)
        print("[second_terminal] Sent: E-STOP with demo text 'secret information'")


    if cmd == 'q':
        print("[second_terminal] Quitting.")
        raise KeyboardInterrupt

    if cmd == 'h':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_ARM_HOME)
        if sendTPacketFrame(client.sock, frame):
            print("[second_terminal] Sent ARM_HOME")
        else:
            print("[second_terminal] Failed to send ARM_HOME")
        return

    if cmd == 'v':
        if len(tokens) != 2:
            print(f"[second_terminal] Usage: v <{ARM_SPEED_MIN}-{ARM_SPEED_MAX}>")
            return
        try:
            speed = int(tokens[1])
        except ValueError:
            print("[second_terminal] Speed must be an integer")
            return
        if speed < ARM_SPEED_MIN or speed > ARM_SPEED_MAX:
            print(f"[second_terminal] Speed out of range: {ARM_SPEED_MIN}-{ARM_SPEED_MAX}")
            return
        _send_arm_command(client, COMMAND_ARM_SET_SPEED, speed, "SPEED")
        return

    arm = _arm_range_for(cmd)
    if arm:
        if len(tokens) != 2:
            lo, hi, _, _ = arm
            print(f"[second_terminal] Usage: {cmd} <{lo}-{hi}>")
            return
        try:
            angle = int(tokens[1])
        except ValueError:
            print("[second_terminal] Angle must be an integer")
            return
        lo, hi, pkt_cmd, label = arm
        if angle < lo or angle > hi:
            print(f"[second_terminal] {label} out of range: {lo}-{hi}")
            return
        _send_arm_command(client, pkt_cmd, angle, label)
        return

    if cmd == 'x':
        frame = _packFrame(PACKET_TYPE_COMMAND, COMMAND_STOP)
        if sendTPacketFrame(client.sock, frame):
            print("[second_terminal] Sent STOP")
        else:
            print("[second_terminal] Failed to send STOP")
        return

    print("[second_terminal] Unknown command. Valid: b/s/e/g <angle>, h, v <speed>, x, q")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def _receiver_loop(client: TCPClient):
    while not _shutdown.is_set():
        frame = recvTPacketFrame(client.sock)
        if frame is None:
            if not _shutdown.is_set():
                print("[second_terminal] Connection to the relay closed.")
            _shutdown.set()
            break

        pkt = _unpackFrame(frame)
        if pkt:
            _printPacket(pkt)


def _make_client_ssl_context():
    if not TLS_CERT_PATH.is_file():
        print(f"[second_terminal] ERROR: TLS certificate not found at '{TLS_CERT_PATH}'.")
        print("  From ~/cg2111a_tls/webtop_client, create certs/ and copy server.crt first.")
        sys.exit(1)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_verify_locations(str(TLS_CERT_PATH))
    ctx.check_hostname = True
    ctx.verify_mode = ssl.CERT_REQUIRED
    return ctx


def run():
    _shutdown.clear()
    ssl_ctx = _make_client_ssl_context() if TLS_ENABLED else None
    client = TCPClient(
        host=PI_HOST,
        port=PI_PORT,
        ssl_context=ssl_ctx,
        server_hostname=PI_HOST,
    )
    print(f"[second_terminal] Connecting to the relay at {PI_HOST}:{PI_PORT}...")

    if not client.connect(timeout=60.0):
        print("[second_terminal] Could not connect.")
        print("  Make sure pi_sensor.py is running and that its relay is"
              " waiting for a second terminal connection.")
        sys.exit(1)

    print("[second_terminal] Connected!")
    print("[second_terminal] Commands:")
    print("  b <angle>  base")
    print("  s <angle>  shoulder")
    print("  e <angle>  elbow")
    print("  g <angle>  gripper")
    print("  h          home")
    print("  v <speed>  speed")
    print("  x          stop")
    print("  q          quit")
    print("[second_terminal] Incoming robot packets will be printed below.\n")

    rx_thread = threading.Thread(target=_receiver_loop, args=(client,), daemon=True)
    rx_thread.start()

    try:
        while not _shutdown.is_set():
            # Check for keyboard input without blocking the receive thread.
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                line = sys.stdin.readline()
                if not line:
                    continue
                _handleInput(line, client)

    except KeyboardInterrupt:
        print("\n[second_terminal] Exiting.")
    finally:
        _shutdown.set()
        client.close()
        rx_thread.join(timeout=1.0)


if __name__ == '__main__':
    run()
