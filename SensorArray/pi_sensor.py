#!/usr/bin/env python3
"""
Studio 13: Sensor Mini-Project
Raspberry Pi Sensor Interface — pi_sensor.py

Extends the communication framework from Studio 12 with:
  - Magic number + checksum framing for reliable packet delivery
  - A command-line interface that reads user commands while displaying
    live Arduino data

Packet framing format (103 bytes total):
  MAGIC (2 B) | TPacket (100 B) | CHECKSUM (1 B)

Usage:
  source env/bin/activate
  python3 pi_sensor.py
"""

import struct
import serial
import time
import sys
import select

import camera_handler as CameraHandler  
import lidar_scanner as LidarScanner
from second_terminal import relay


# ----------------------------------------------------------------
# SERIAL PORT SETUP
# ----------------------------------------------------------------

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600

_ser = None


def openSerial():
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=0.2)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    time.sleep(2)

    _ser.reset_input_buffer()
    _ser.reset_output_buffer()

    print("Ready.\n")


def closeSerial():
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()


# ----------------------------------------------------------------
# TPACKET CONSTANTS
# (must match sensor_miniproject_template.ino)
# ----------------------------------------------------------------
from packets import *



def computeChecksum(data: bytes) -> int:
    """Return the XOR of all bytes in data."""
    result = 0
    for b in data:
        result ^= b
    return result


def packFrame(packetType, command, data=b'', params=None):
    """
    Build a framed packet: MAGIC | TPacket bytes | checksum.

    Args:
        packetType: integer packet type constant
        command:    integer command / response constant
        data:       bytes for the data field (padded/truncated to MAX_STR_LEN)
        params:     list of PARAMS_COUNT uint32 values (default: all zeros)

    Returns:
        bytes of length FRAME_SIZE (103)
    """
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def unpackTPacket(raw):
    """Deserialise a 100-byte TPacket into a dict."""
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


def receiveFrame():
    """
    Read bytes from the serial port until a valid framed packet is found.

    Synchronises to the magic number, reads the TPacket body, then
    validates the checksum.  Discards corrupt or out-of-sync bytes.

    Returns:
        A packet dict (see unpackTPacket), or None on timeout / error.
    """
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]

    while True:
        # Read and discard bytes until we see the first magic byte.
        b = _ser.read(1)
        if not b:
            return None          # timeout
        if b[0] != MAGIC_HI:
            continue

        # Read the second magic byte.
        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_LO:
            # Not the magic number; keep searching (don't skip the byte
            # we just read in case it is the first byte of another frame).
            continue

        # Magic matched; now read the TPacket body.
        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        # Read and verify the checksum.
        cs_byte = _ser.read(1)
        if not cs_byte:
            return None
        expected = computeChecksum(raw)
        if cs_byte[0] != expected:
            # Checksum mismatch: corrupted packet, try to resync.
            continue

        return unpackTPacket(raw)


def sendCommand(commandType, data=b'', params=None):
    """Send a framed COMMAND packet to the Arduino."""
    frame = packFrame(PACKET_TYPE_COMMAND, commandType, data=data, params=params)
    _ser.write(frame)


# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING


def isEstopActive():
    """Return True if the E-Stop is currently active (system stopped)."""
    return _estop_state == STATE_STOPPED


# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def printPacket(pkt):
    """Print a received TPacket in human-readable form."""
    global _estop_state
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")
        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_state = state
            if state == STATE_RUNNING:
                print("Status: RUNNING")
            else:
                print("Status: STOPPED")
        elif cmd == RESP_COLOR_SENSOR:
            # TODO (Activity 2): add an elif branch here to handle your color
            # response.  Display the three channel frequencies in Hz, e.g.:
            #   R: <params[0]> Hz, G: <params[1]> Hz, B: <params[2]> Hz
            r = pkt['params'][0]
            g = pkt['params'][1]
            b = pkt['params'][2]
            c = pkt['params'][3]
            if c == 0:
                d = 'red'
            elif c == 1:
                d = 'green'
            else:
                d = 'blue'
            print(f"Color: R={r} Hz, G={g} Hz, B={b} Hz, Color={d}")
        elif cmd == RESP_MOTOR_STATUS:
            speed = pkt['params'][0]
            print(f"Motor speed: {speed}")
        elif cmd == RESP_ARM_STATUS:
            base = pkt['params'][0]
            shoulder = pkt['params'][1]
            elbow = pkt['params'][2]
            gripper = pkt['params'][3]
            speed = pkt['params'][4]
            t_base = pkt['params'][5]
            t_shoulder = pkt['params'][6]
            t_elbow = pkt['params'][7]
            t_gripper = pkt['params'][8]
            print(
                f"Arm status: pos(B={base} S={shoulder} E={elbow} G={gripper}) "
                f"target(B={t_base} S={t_shoulder} E={t_elbow} G={t_gripper}) V={speed}"
            )
        else:
            print(f"Response: unknown command {cmd}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"Arduino: {msg}")
    else:
        print(f"Packet: type={ptype}, cmd={cmd}")


# ----------------------------------------------------------------
# ACTIVITY 2: COLOR SENSOR
# ----------------------------------------------------------------

def handleColorCommand():
    """
    TODO (Activity 2): request a color reading from the Arduino and display it.

    Check the E-Stop state first; if stopped, refuse with a clear message.
    Otherwise, send your color command to the Arduino.
    """
    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    sendCommand(COMMAND_COLOR_SENSOR)


# ----------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ----------------------------------------------------------------


#_camera = None          # TODO (Activity 3): open the camera (cameraOpen()) before first use.
_frames_remaining = 5   # frames remaining before further captures are refused


def handleCameraCommand():
    """
    TODO (Activity 3): capture and display a greyscale frame.

    Gate on E-Stop state and the remaining frame count.
    Use captureGreyscaleFrame() and renderGreyscaleFrame() from alex_camera.
    """
    global _frames_remaining

    if isEstopActive():
        print("Refused: E-Stop is active")
        return

    if _frames_remaining == 0:
        print("Could not capture frame: no frames remaining")
        return

    if CameraHandler.camera_capture(): # if succes 
        _frames_remaining -= 1 


# ----------------------------------------------------------------
# ACTIVITY 4: LIDAR
# ----------------------------------------------------------------


def handleLidarCommand():
    """
    Activity 4: perform a single LIDAR scan and render it.

    Gate on E-Stop state, then use the LIDAR library to capture one scan
    and the CLI plot helpers to display it.
    """

    if isEstopActive():
        print("Refused: E-Stop is active")
        return
    
    LidarScanner.lidar_scan()


def handleDriveCommand(line):
    global motor_speed

    if isEstopActive():
        print("Refused: E-Stop is active")
        return

    if line == 'w':
        print("Driving forward...")
        sendCommand(COMMAND_FORWARD)

    elif line == 's':
        print("Driving backward...")
        sendCommand(COMMAND_BACKWARD)

    elif line == 'a':
        print("Turning left...")
        sendCommand(COMMAND_LEFT)

    elif line == 'd':
        print("Turning right...")
        sendCommand(COMMAND_RIGHT)

    elif line == 'x':
        print("Stopping motors...")
        sendCommand(COMMAND_STOP)

    elif line == '+':
        motor_speed = min(255, motor_speed + 20)
        print(f"Setting motor speed to {motor_speed}...")
        sendCommand(COMMAND_SET_SPEED, params=[motor_speed] + [0] * (PARAMS_COUNT - 1))

    elif line == '-':
        motor_speed = max(0, motor_speed - 20)
        print(f"Setting motor speed to {motor_speed}...")
        sendCommand(COMMAND_SET_SPEED, params=[motor_speed] + [0] * (PARAMS_COUNT - 1))


# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ----------------------------------------------------------------

# User input -> action mapping:
#   e  send a software E-Stop command to the Arduino (pre-wired)
#   c  request color reading from the Arduino        (Activity 2 - implement yourself)
#   p  capture and display a camera frame            (Activity 3 - implement yourself)
#   l  perform a single LIDAR scan                   (Activity 4 - implement yourself)


def handleUserInput(line):
    if line == 'e':
        print("Sending E-Stop command...")
        sendCommand(COMMAND_ESTOP, data=b'This is a debug message')

    elif line == 'c':
        handleColorCommand()

    elif line == 'p':
        handleCameraCommand()

    elif line == 'l':
        handleLidarCommand()

    elif line in ['w', 'a', 's', 'd', 'x', '+', '-']:
        handleDriveCommand(line)

    else:
        print("Unknown input. Valid: e, c, p, l, w, a, s, d, x, +, -")


def runCommandInterface():
    """
    Main command loop.

    Uses select.select() to simultaneously receive packets from the Arduino
    and read typed user input from stdin without either blocking the other.
    """
    print("Sensor interface ready.")
    print("Type e / c / p / l / w / a / s / d / x / + / - and press Enter.")
    print("e = estop, c = color, p = camera, l = lidar")
    print("w = forward, s = backward, a = left, d = right, x = stop, + = faster, - = slower")
    print("Press Ctrl+C to exit.\n")

    while True:
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)
                relay.onPacketReceived(packFrame(pkt['packetType'], pkt['command'],
pkt['data'], pkt['params']))

        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if not line:
                time.sleep(0.05)
                continue
            handleUserInput(line)

        frame = relay.recvFromSecondTerminal()
        if frame is not None:
            if len(frame) != FRAME_SIZE or frame[:2] != MAGIC:
                print("[relay] Dropped invalid frame from second terminal")
            else:
                raw = frame[2:2 + TPACKET_SIZE]
                if frame[-1] != computeChecksum(raw):
                    print("[relay] Dropped bad-checksum frame from second terminal")
                else:
                    pkt = unpackTPacket(raw)
                    if pkt['packetType'] == PACKET_TYPE_COMMAND:
                        _ser.write(frame)
                    else:
                        print("[relay] Dropped non-command packet from second terminal")
        time.sleep(0.05)


# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    openSerial()
    relay.start()
    LidarScanner.lidar_connect()
    _camera = alex_camera.cameraOpen()
    CameraHandler.camera_connect() 
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        relay.shutdown()
        LidarScanner.lidar_disconnect()
        CameraHandler.camera_close()
        closeSerial()
