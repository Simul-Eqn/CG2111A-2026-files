#!/usr/bin/env python3
"""
sensor_station_cli.py - Communication-station CLI for non-LIDAR sensor control.

This mirrors pi_sensor.py controls except LIDAR, but sends commands to the
remote rp_lidar_api.py server via the client API in lidar.py.

Commands:
  e  trigger software E-Stop
  c  request color sensor reading
  p  capture camera frame
  w  drive forward
  s  drive backward
  a  turn left
  d  turn right
  x  stop motors
  +  increase speed by 20 (max 255)
  -  decrease speed by 20 (min 0)
  q  quit
"""

from packets import STATE_RUNNING, STATE_STOPPED
import lidar

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 9600

_speed = 150


def _state_text(state):
    if state == STATE_RUNNING:
        return "RUNNING"
    if state == STATE_STOPPED:
        return "STOPPED"
    return f"UNKNOWN({state})"


def _get_status():
    state = lidar.sensor_get_status()
    if state is None:
        print("Could not read status from server.")
        return None
    return state


def _is_estop_active():
    state = _get_status()
    if state is None:
        return True
    return state == STATE_STOPPED


def _handle_estop():
    print("Sending E-Stop command...")
    if not lidar.sensor_estop():
        print("E-Stop command failed.")
        return
    state = _get_status()
    if state is None:
        print("E-Stop sent, but status is unknown.")
    else:
        print(f"Status: {_state_text(state)}")


def _handle_color():
    if _is_estop_active():
        print("Refused: E-Stop is active")
        return

    rgb = lidar.sensor_get_color()
    if rgb is None:
        print("Color read failed.")
        return

    r, g, b = rgb
    print(f"Color: R={r} Hz, G={g} Hz, B={b} Hz")


def _handle_camera():
    if _is_estop_active():
        print("Refused: E-Stop is active")
        return

    if lidar.sensor_camera_capture():
        print("Camera frame captured and sent.")
    else:
        print("Could not capture frame.")


def _handle_drive(line):
    global _speed

    if _is_estop_active():
        print("Refused: E-Stop is active")
        return

    if line == 'w':
        print("Driving forward...")
        ok = lidar.sensor_forward()
    elif line == 's':
        print("Driving backward...")
        ok = lidar.sensor_backward()
    elif line == 'a':
        print("Turning left...")
        ok = lidar.sensor_left()
    elif line == 'd':
        print("Turning right...")
        ok = lidar.sensor_right()
    elif line == 'x':
        print("Stopping motors...")
        ok = lidar.sensor_stop()
    elif line == '+':
        _speed = min(255, _speed + 20)
        print(f"Setting motor speed to {_speed}...")
        ok = lidar.sensor_set_speed(_speed)
    elif line == '-':
        _speed = max(0, _speed - 20)
        print(f"Setting motor speed to {_speed}...")
        ok = lidar.sensor_set_speed(_speed)
    else:
        return

    if not ok:
        print("Command failed.")


def _handle_input(line):
    if line == 'e':
        _handle_estop()
    elif line == 'c':
        _handle_color()
    elif line == 'p':
        _handle_camera()
    elif line in ['w', 'a', 's', 'd', 'x', '+', '-']:
        _handle_drive(line)
    elif line == 'status':
        state = _get_status()
        if state is not None:
            print(f"Status: {_state_text(state)}")
    elif line == 'q':
        raise KeyboardInterrupt
    else:
        print("Unknown input. Valid: e, c, p, w, a, s, d, x, +, -, status, q")


def run_command_interface():
    print("Remote sensor interface ready.")
    print("Type e / c / p / w / a / s / d / x / + / - / status / q and press Enter.")
    print("e = estop, c = color, p = camera")
    print("w = forward, s = backward, a = left, d = right, x = stop, + = faster, - = slower")
    print("Press Ctrl+C to exit.\n")

    state = _get_status()
    if state is not None:
        print(f"Initial status: {_state_text(state)}")

    while True:
        line = input("> ").strip().lower()
        if not line:
            continue
        _handle_input(line)


def main():
    connected = lidar.sensor_serial_connect(port=ARDUINO_PORT, baudrate=ARDUINO_BAUD)
    if not connected:
        print(f"Could not open remote Arduino serial {ARDUINO_PORT}@{ARDUINO_BAUD}.")
        return

    print(f"Remote Arduino serial connected at {ARDUINO_PORT}@{ARDUINO_BAUD}.")
    try:
        run_command_interface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        lidar.sensor_serial_disconnect()


if __name__ == '__main__':
    main()
