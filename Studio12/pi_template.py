#!/usr/bin/env python3
"""
Studio 12: Remote Control
Raspberry Pi Template - pi_template.py

Activity 2: Read a TData struct in Python and compare with the C version.
Activity 3: Send TPacket commands to the Arduino and receive responses.

Complete the TODO sections marked below.

Usage:
  python3 pi_template.py activity2   # run Activity 2 (TData receive loop)
  python3 pi_template.py             # run Activity 3 (command protocol)
"""

import struct
import serial
import time
import sys

# ================================================================
# SERIAL PORT SETUP
# ================================================================

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600

_ser = None


def openSerial():
    """Open the serial port and wait for the Arduino to boot."""
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino to boot...")
    time.sleep(2)
    print("Ready.")


def closeSerial():
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()
        print("Serial port closed.")


# ================================================================
# ACTIVITY 2 - Receiving TData in Python
# ================================================================
#
# The Arduino sends sizeof(TData) as the first byte, followed by
# the raw bytes of TData.  By this point (Activity 2) TData uses
# int32_t for both fields, so Arduino and Pi agree on the size.
#
# TData layout (int32_t x, int32_t y):
#   Field  Size  Offset
#   x       4 B    0
#   y       4 B    4
#   Total   8 B

TDATA_SIZE = 8


def readTData():
    """
    Send 's' to the Arduino and read back one TData struct.
    Prints the sizes reported by each platform and the unpacked x, y values.
    """
    # Send the trigger character to ask the Arduino to send TData.
    _ser.write(b's')

    # Read the first byte: the Arduino reports its own sizeof(TData).
    size_byte = _ser.read(1)
    if not size_byte:
        print("Timeout: no response from Arduino")
        return
    arduino_size = size_byte[0]

    # Print the struct size on each platform so we can compare them.
    print(f"sizeof(TData) on this machine : {TDATA_SIZE} bytes")
    print(f"sizeof(TData) on Arduino      : {arduino_size} bytes")

    # Read the remaining bytes until we have a full struct from the Arduino.
    raw = b''
    while len(raw) < arduino_size:
        chunk = _ser.read(arduino_size - len(raw))
        if chunk:
            raw += chunk

    # TODO (Activity 2): Replace the placeholder below with the correct
    # struct.unpack format string to unpack two signed 32-bit little-endian
    # integers.  The function will raise a TypeError until you fix this.
    FMT = None   # replace with your format string, e.g. '<XY'
    x, y = struct.unpack(FMT, raw[:TDATA_SIZE])
    print(f"x = {x}, y = {y}")


# ================================================================
# ACTIVITY 3 - Simple Command Protocol
# ================================================================
#
# TPacket layout (must match template.ino):
#   Field       Type        Size  Offset
#   packetType  uint8_t      1 B    0
#   command     uint8_t      1 B    1
#   dummy       uint8_t[2]   2 B    2   (padding)
#   data        char[32]    32 B    4
#   params      uint32_t[16] 64 B   36
#   Total                  100 B

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

# Total size of one serialised TPacket (bytes).
TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)   # = 100

# struct.pack / struct.unpack format string for TPacket:
#   <  - little-endian
#   B  - uint8  (packetType)
#   B  - uint8  (command)
#   2x - 2 padding bytes  (dummy[2])
#   32s- char[32]          (data)
#   16I- uint32_t[16]      (params)
TPACKET_FMT = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'


def packTPacket(packetType, command, data=b'', params=None):
    """
    Serialise a TPacket into bytes.

    Args:
        packetType: integer packet type constant
        command:    integer command / response constant
        data:       bytes for the data field (truncated or zero-padded to 32)
        params:     list of PARAMS_COUNT uint32 values (default: all zeros)

    Returns:
        bytes of length TPACKET_SIZE (100)
    """
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    return struct.pack(TPACKET_FMT, packetType, command, data_padded, *params)


def unpackTPacket(raw):
    """
    Deserialise a TPacket from bytes.

    Args:
        raw: bytes of length TPACKET_SIZE

    Returns:
        dict with keys: packetType, command, data, params
    """
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


def sendCommand(commandType, params=None):
    """
    Send a COMMAND packet to the Arduino.

    Builds a TPacket with packetType = PACKET_TYPE_COMMAND, serialises it
    with packTPacket(), and writes the bytes to the serial port.
    """
    raw = packTPacket(PACKET_TYPE_COMMAND, commandType, params=params)
    _ser.write(raw)


def receivePacket():
    """
    Read exactly TPACKET_SIZE bytes from the serial port and
    deserialise into a TPacket dict.

    Returns:
        A packet dict (see unpackTPacket), or None on timeout.
    """
    raw = b''
    while len(raw) < TPACKET_SIZE:
        chunk = _ser.read(TPACKET_SIZE - len(raw))
        if not chunk:
            print("Timeout waiting for packet")
            return None
        raw += chunk
    return unpackTPacket(raw)


def printPacket(pkt):
    """Print the contents of a received TPacket in a human-readable form."""
    ptype = pkt['packetType']
    cmd   = pkt['command']
    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            if state == STATE_RUNNING:
                print("Status: RUNNING")
            elif state == STATE_STOPPED:
                print("Status: STOPPED")
            else:
                print(f"Status: UNKNOWN ({state})")
        else:
            print(f"Response: unknown command {cmd}")
    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"Message: {msg}")
    else:
        print(f"Packet: type={ptype}, cmd={cmd}")


# ================================================================
# MAIN
# ================================================================

if __name__ == "__main__":
    openSerial()
    try:
        if len(sys.argv) > 1 and sys.argv[1] == "activity2":
            # ---- Activity 2: receive TData in a loop ----
            while True:
                readTData()
                time.sleep(1)
        else:
            # ---- Activity 3: command protocol + E-Stop ----
            print("Press Enter to send an E-Stop command to the Arduino.")
            print("Press Ctrl+C to exit.\n")
            while True:
                # Check for incoming packets from the Arduino (e.g. button status).
                if _ser.in_waiting >= TPACKET_SIZE:
                    pkt = receivePacket()
                    if pkt:
                        printPacket(pkt)

                # Non-blocking keypress detection: select.select() checks
                # whether stdin has data ready to read without blocking.
                # If the user presses Enter, rlist is non-empty, so we
                # consume the line and send a COMMAND_ESTOP packet to the Arduino.
                import select
                rlist, _, _ = select.select([sys.stdin], [], [], 0)
                if rlist:
                    sys.stdin.readline()   # consume the Enter key
                    print("Sending E-Stop command...")
                    sendCommand(COMMAND_ESTOP)
                    pkt = receivePacket()
                    if pkt:
                        printPacket(pkt)

                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        closeSerial()
