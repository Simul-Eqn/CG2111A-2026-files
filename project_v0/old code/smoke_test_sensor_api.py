#!/usr/bin/env python3
"""
Smoke test for non-LIDAR sensor APIs exposed through lidar.py client library.

This script validates the newly added remote API path in rp_lidar_api.py:
- serial connect/disconnect
- status read
- set speed
- drive commands
- color sensor read
- camera capture
- optional E-Stop

Usage examples:
  python smoke_test_sensor_api.py
  python smoke_test_sensor_api.py --skip-camera
  python smoke_test_sensor_api.py --do-estop
"""

import argparse
import sys
import time

import lidar as sensor_api
from packets import (
    STATE_RUNNING,
    STATE_STOPPED,
)


def _status_name(value):
    if value == STATE_RUNNING:
        return "RUNNING"
    if value == STATE_STOPPED:
        return "STOPPED"
    return f"UNKNOWN({value})"


def _run_step(name, fn):
    try:
        ok, detail = fn()
    except Exception as exc:
        return False, f"{name}: EXCEPTION -> {exc}"

    state = "PASS" if ok else "FAIL"
    suffix = f" | {detail}" if detail else ""
    return ok, f"{name}: {state}{suffix}"


def main():
    parser = argparse.ArgumentParser(description="Smoke test for non-LIDAR sensor remote APIs")
    parser.add_argument("--speed", type=int, default=150, help="Motor speed to set (0..255)")
    parser.add_argument("--move-seconds", type=float, default=0.35, help="Duration for each movement command")
    parser.add_argument("--skip-color", action="store_true", help="Skip color sensor step")
    parser.add_argument("--skip-camera", action="store_true", help="Skip camera capture step")
    parser.add_argument("--do-estop", action="store_true", help="Trigger E-Stop as final step")
    args = parser.parse_args()

    speed = max(0, min(255, int(args.speed)))
    move_seconds = max(0.05, float(args.move_seconds))

    print("== Sensor API Smoke Test ==")
    print(f"speed={speed}, move_seconds={move_seconds:.2f}, skip_color={args.skip_color}, skip_camera={args.skip_camera}, do_estop={args.do_estop}")

    results = []
    serial_connected = False

    def step_connect_serial():
        nonlocal serial_connected
        ok = sensor_api.sensor_serial_connect()
        serial_connected = bool(ok)
        return bool(ok), "connected" if ok else "unable to open sensor serial on server"

    def step_get_status_initial():
        state = sensor_api.sensor_get_status()
        if state is None:
            return False, "status=None"
        return True, f"state={_status_name(state)}"

    def step_set_speed():
        ok = sensor_api.sensor_set_speed(speed)
        return bool(ok), f"speed={speed}"

    def _drive_once(label, fn):
        ok = fn()
        if not ok:
            return False, f"{label} command rejected"
        time.sleep(move_seconds)
        stopped = sensor_api.sensor_stop()
        return bool(stopped), f"{label} then STOP"

    def step_forward():
        return _drive_once("FORWARD", sensor_api.sensor_forward)

    def step_left():
        return _drive_once("LEFT", sensor_api.sensor_left)

    def step_color():
        rgb = sensor_api.sensor_get_color()
        if rgb is None:
            return False, "no color response"
        r, g, b = rgb
        return True, f"r={r}, g={g}, b={b}"

    def step_camera():
        ok = sensor_api.sensor_camera_capture()
        return bool(ok), "frame sent" if ok else "capture failed or frame limit reached"

    def step_estop():
        ok = sensor_api.sensor_estop()
        if not ok:
            return False, "command failed"
        state = sensor_api.sensor_get_status()
        if state == STATE_STOPPED:
            return True, "state=STOPPED"
        return False, f"expected STOPPED, got {_status_name(state)}"

    try:
        steps = [
            ("connect_serial", step_connect_serial),
            ("get_status_initial", step_get_status_initial),
            ("set_speed", step_set_speed),
            ("drive_forward", step_forward),
            ("drive_left", step_left),
        ]

        if not args.skip_color:
            steps.append(("get_color", step_color))
        if not args.skip_camera:
            steps.append(("camera_capture", step_camera))
        if args.do_estop:
            steps.append(("estop", step_estop))

        for name, fn in steps:
            ok, message = _run_step(name, fn)
            results.append((name, ok, message))
            print(message)

    finally:
        try:
            sensor_api.sensor_stop()
        except Exception:
            pass

        if serial_connected:
            try:
                sensor_api.sensor_serial_disconnect()
            except Exception:
                pass

    passed = sum(1 for _, ok, _ in results if ok)
    total = len(results)
    print(f"\nSummary: {passed}/{total} steps passed")

    if passed != total:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
