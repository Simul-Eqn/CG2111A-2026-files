#!/usr/bin/env python3
"""
reference_rp_lidar_api_sim.py - Local simulator for rp_lidar_api.py.

This is a drop-in local server for testing slam.py without the Raspberry Pi
or the real RPLidar hardware. It keeps the same TCP protocol as rp_lidar_api.py,
but simulates the hardware-dependent parts:

  - LIDAR connect/get mode/scan/disconnect
  - sensor serial connect/disconnect
  - E-Stop, movement, speed, color sensor, camera capture
  - robot arm commands, with arm state printed in the simulator log

Use this alongside lidar.py by pointing the client at 127.0.0.1.
You can do that with the MPSV0_SERVER_IP environment variable.
"""

from __future__ import annotations

import asyncio
import math
import os
import random
import socket
import struct
import threading
import time
from dataclasses import dataclass

import mpsv0_connection_params as net_params
from connection_params import CAMERA_CONNECTION_PARAMS
from packets import *
from settings import LIDAR_PORT, LIDAR_BAUD, MAX_DISTANCE_MM


# ---------------------------------------------------------------------------
# Simulated world state
# ---------------------------------------------------------------------------

WORLD_MIN_MM = 0.0
WORLD_MAX_MM = 6000.0


@dataclass
class SimState:
    x_mm: float = 3000.0
    y_mm: float = 3000.0
    theta_deg: float = 0.0
    motor_speed: int = 90
    estop_state: int = STATE_RUNNING
    camera_captures: int = 0
    drive_mode: str = 'STOP'
    last_update_ts: float = 0.0

    arm_base: int = 90
    arm_shoulder: int = 90
    arm_elbow: int = 90
    arm_gripper: int = 90
    arm_speed: int = 20

    def arm_status_text(self) -> str:
        return (
            f"Arm status: pos(B={self.arm_base} S={self.arm_shoulder} "
            f"E={self.arm_elbow} G={self.arm_gripper}) V={self.arm_speed}"
        )

@dataclass(frozen=True)
class ColorRegion:
    name: str
    rect: tuple[float, float, float, float]
    display_color: str
    sensor_bias: tuple[int, int, int]

    def contains(self, x_mm: float, y_mm: float) -> bool:
        x0, y0, x1, y1 = self.rect
        return x0 <= x_mm <= x1 and y0 <= y_mm <= y1


STATE = SimState()
STATE.last_update_ts = time.monotonic()

# A few simple rectangular obstacles to make the occupancy map interesting.
OBSTACLES = [
    (1400, 1200, 1900, 1800),
    (3800, 1300, 4500, 1700),
    (2500, 3200, 3200, 3700),
    (1200, 4300, 1700, 5000),
    (4200, 4200, 5000, 5000),
]

# Fixed color regions used by the synthetic color sensor and the ground-truth
# display.  The sensor bias values are frequency-like readings: lower numbers
# indicate a stronger response for that channel.
COLOR_REGIONS = [
    ColorRegion('red',   (   0,    0, 2000, 3000), 'red',          ( 420, 1100, 1100)),
    ColorRegion('green', (2000,    0, 4000, 3000), 'lime',         (1100,  420, 1100)),
    ColorRegion('blue',  (4000,    0, 6000, 3000), 'deepskyblue',   (1100, 1100,  420)),
    ColorRegion('blue',  (   0, 3000, 2000, 6000), 'deepskyblue',   (1100, 1100,  420)),
    ColorRegion('red',   (2000, 3000, 4000, 6000), 'red',           ( 420, 1100, 1100)),
    ColorRegion('green', (4000, 3000, 6000, 6000), 'lime',          (1100,  420, 1100)),
]

_display_thread = None


# ---------------------------------------------------------------------------
# LIDAR API surface
# ---------------------------------------------------------------------------


def connect(port=LIDAR_PORT, baudrate=LIDAR_BAUD):
    """Simulate opening and resetting the lidar."""
    print(f"[sim] LIDAR connect requested on {port}@{baudrate}")
    return {'port': port, 'baudrate': baudrate}



def get_scan_mode(lidar):
    """Return a stable scan mode for the simulator."""
    return 2



def disconnect(lidar):
    """Simulate closing the lidar."""
    print("[sim] LIDAR disconnected")


# ---------------------------------------------------------------------------
# Packet helpers for the TCP protocol used by lidar.py
# ---------------------------------------------------------------------------


def _compute_checksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result



def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _region_at(x_mm: float, y_mm: float):
    for region in COLOR_REGIONS:
        if region.contains(x_mm, y_mm):
            return region
    return None


def _simulate_color_sensor():
    """Return synthetic color sensor frequencies based on the robot position."""
    region = _region_at(STATE.x_mm, STATE.y_mm)
    if region is None:
        base = 1000
        return (base, base, base)

    r_bias, g_bias, b_bias = region.sensor_bias
    # Keep the region's target channel clearly strongest while adding noise.
    return (
        int(_clamp(r_bias + random.randint(-35, 35), 0, 4095)),
        int(_clamp(g_bias + random.randint(-35, 35), 0, 4095)),
        int(_clamp(b_bias + random.randint(-35, 35), 0, 4095)),
    )


def _build_dummy_camera_frame() -> bytes:
    """Build a grayscale dummy image for the camera receiver.

    The image is a top-down view of the simulator world so the receiver gets
    a stable, meaningful frame without needing real camera hardware.
    """
    from alex_camera import RENDER_HEIGHT, RENDER_WIDTH

    frame = bytearray(RENDER_HEIGHT * RENDER_WIDTH)
    for row in range(RENDER_HEIGHT):
        world_y = WORLD_MAX_MM - (row + 0.5) * (WORLD_MAX_MM - WORLD_MIN_MM) / RENDER_HEIGHT
        for col in range(RENDER_WIDTH):
            world_x = WORLD_MIN_MM + (col + 0.5) * (WORLD_MAX_MM - WORLD_MIN_MM) / RENDER_WIDTH
            idx = row * RENDER_WIDTH + col

            region = _region_at(world_x, world_y)
            if region is None:
                intensity = 110
            elif region.name == 'red':
                intensity = 70
            elif region.name == 'green':
                intensity = 150
            else:
                intensity = 220

            # Highlight the robot position so the dummy image changes as it moves.
            dx = abs(world_x - STATE.x_mm)
            dy = abs(world_y - STATE.y_mm)
            if dx <= 120.0 and dy <= 120.0:
                intensity = 255
            elif dx <= 260.0 and dy <= 260.0:
                intensity = min(255, intensity + 25)

            # Darken known obstacles so the frame has some structure.
            for x0, y0, x1, y1 in OBSTACLES:
                if x0 <= world_x <= x1 and y0 <= world_y <= y1:
                    intensity = 40
                    break

            frame[idx] = intensity

    return bytes(frame)


def _send_dummy_camera_frame() -> bool:
    """Send a dummy camera frame to the camera receiver server."""
    from alex_camera import RENDER_HEIGHT, RENDER_WIDTH

    payload = _build_dummy_camera_frame()
    try:
        with socket.create_connection(CAMERA_CONNECTION_PARAMS, timeout=2.0) as sock:
            header = struct.pack('!II', RENDER_HEIGHT, RENDER_WIDTH)
            sock.sendall(header + payload)
            sock.sendall(b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF')
        return True
    except OSError as exc:
        print(f'[sim] Dummy camera send failed: {exc}')
        return False



def _ray_rect_intersection(x0, y0, dx, dy, rect):
    """Return the nearest positive ray-rectangle intersection in param units."""
    xmin, ymin, xmax, ymax = rect
    candidates = []

    if abs(dx) > 1e-9:
        for x in (xmin, xmax):
            t = (x - x0) / dx
            if t <= 0:
                continue
            y = y0 + t * dy
            if ymin <= y <= ymax:
                candidates.append(t)

    if abs(dy) > 1e-9:
        for y in (ymin, ymax):
            t = (y - y0) / dy
            if t <= 0:
                continue
            x = x0 + t * dx
            if xmin <= x <= xmax:
                candidates.append(t)

    return min(candidates) if candidates else None



def _advance_pose():
    now = time.monotonic()
    dt = now - STATE.last_update_ts
    STATE.last_update_ts = now
    if dt <= 0:
        return

    v_mm_s = float(STATE.motor_speed) * 1.0
    w_deg_s = 15.0

    if STATE.drive_mode == 'FORWARD':
        theta = math.radians(STATE.theta_deg)
        STATE.x_mm += math.cos(theta) * v_mm_s * dt
        STATE.y_mm += math.sin(theta) * v_mm_s * dt
    elif STATE.drive_mode == 'BACKWARD':
        theta = math.radians(STATE.theta_deg)
        STATE.x_mm -= math.cos(theta) * v_mm_s * dt
        STATE.y_mm -= math.sin(theta) * v_mm_s * dt
    elif STATE.drive_mode == 'LEFT':
        STATE.theta_deg += w_deg_s * dt
    elif STATE.drive_mode == 'RIGHT':
        STATE.theta_deg -= w_deg_s * dt

    STATE.theta_deg %= 360.0
    STATE.x_mm = _clamp(STATE.x_mm, WORLD_MIN_MM + 50.0, WORLD_MAX_MM - 50.0)
    STATE.y_mm = _clamp(STATE.y_mm, WORLD_MIN_MM + 50.0, WORLD_MAX_MM - 50.0)


def _simulate_distance(raw_angle_deg):
    """Simulate one range reading for a CW angle measured from robot forward."""
    theta = math.radians(STATE.theta_deg - raw_angle_deg)
    dx = math.cos(theta)
    dy = math.sin(theta)

    x0 = STATE.x_mm
    y0 = STATE.y_mm

    hits = []

    # World boundary walls.
    walls = [
        (WORLD_MIN_MM, WORLD_MIN_MM, WORLD_MAX_MM, WORLD_MIN_MM),
        (WORLD_MAX_MM, WORLD_MIN_MM, WORLD_MAX_MM, WORLD_MAX_MM),
        (WORLD_MIN_MM, WORLD_MIN_MM, WORLD_MIN_MM, WORLD_MAX_MM),
        (WORLD_MIN_MM, WORLD_MAX_MM, WORLD_MAX_MM, WORLD_MAX_MM),
    ]
    for wall in walls:
        hit = _ray_rect_intersection(x0, y0, dx, dy, wall)
        if hit is not None:
            hits.append(hit)

    for rect in OBSTACLES:
        hit = _ray_rect_intersection(x0, y0, dx, dy, rect)
        if hit is not None:
            hits.append(hit)

    if not hits:
        return MAX_DISTANCE_MM

    dist_mm = min(hits) + random.uniform(-10.0, 10.0)
    return int(_clamp(dist_mm, 0, MAX_DISTANCE_MM))



def _scan_angles_and_distances():
    angles = [float(i) for i in range(360)]
    distances = []
    for angle in angles:
        distances.append(_simulate_distance(angle))
    return angles, distances



def scan_rounds(lidar, mode):
    """Yield synthetic 360-degree scans continuously."""
    scan_index = 0
    while True:
        _advance_pose()
        angles, distances = _scan_angles_and_distances()
        if scan_index % 8 == 0:
            print(
                f"[sim] Scan {scan_index}: pose x={STATE.x_mm:.0f} y={STATE.y_mm:.0f} "
                f"th={STATE.theta_deg:.1f} speed={STATE.motor_speed}"
            )
        scan_index += 1
        yield angles, distances



def _send_bool(writer, ok):
    writer.write(struct.pack('b', 1 if ok else 0))



def _update_drive(command):
    if command == COMMAND_FORWARD:
        STATE.drive_mode = 'FORWARD'
        print(f"[sim] DRIVE forward mode")
    elif command == COMMAND_BACKWARD:
        STATE.drive_mode = 'BACKWARD'
        print(f"[sim] DRIVE backward mode")
    elif command == COMMAND_LEFT:
        STATE.drive_mode = 'LEFT'
        print(f"[sim] DRIVE left mode")
    elif command == COMMAND_RIGHT:
        STATE.drive_mode = 'RIGHT'
        print(f"[sim] DRIVE right mode")
    elif command == COMMAND_STOP:
        STATE.drive_mode = 'STOP'
        print("[sim] DRIVE stop mode")



def _print_arm_state(prefix="[sim] ARM"):
    print(f"{prefix} {STATE.arm_status_text()} targets are current in simulator")



def _handle_arm_command(command, value=None):
    if STATE.estop_state == STATE_STOPPED:
        print('[sim] ARM command rejected: E-STOP active')
        return False

    if command == COMMAND_ARM_BASE and value is not None:
        STATE.arm_base = int(_clamp(value, ARM_BASE_MIN, ARM_BASE_MAX))
        print(f"[sim] ARM base -> {STATE.arm_base}")
    elif command == COMMAND_ARM_SHOULDER and value is not None:
        STATE.arm_shoulder = int(_clamp(value, ARM_SHOULDER_MIN, ARM_SHOULDER_MAX))
        print(f"[sim] ARM shoulder -> {STATE.arm_shoulder}")
    elif command == COMMAND_ARM_ELBOW and value is not None:
        STATE.arm_elbow = int(_clamp(value, ARM_ELBOW_MIN, ARM_ELBOW_MAX))
        print(f"[sim] ARM elbow -> {STATE.arm_elbow}")
    elif command == COMMAND_ARM_GRIPPER and value is not None:
        STATE.arm_gripper = int(_clamp(value, ARM_GRIPPER_MIN, ARM_GRIPPER_MAX))
        print(f"[sim] ARM gripper -> {STATE.arm_gripper}")
    elif command == COMMAND_ARM_HOME:
        STATE.arm_base = 90
        STATE.arm_shoulder = 90
        STATE.arm_elbow = 90
        STATE.arm_gripper = 90
        print("[sim] ARM home")
    elif command == COMMAND_ARM_SET_SPEED and value is not None:
        STATE.arm_speed = int(_clamp(value, ARM_SPEED_MIN, ARM_SPEED_MAX))
        print(f"[sim] ARM speed -> {STATE.arm_speed}")
    else:
        return False

    _print_arm_state()
    return True


def _start_sim_display():
    """Start a ground-truth simulator display in a background thread."""
    global _display_thread

    if _display_thread is not None:
        return

    if os.getenv('SIM_SHOW_DISPLAY', '1') in {'0', 'false', 'False'}:
        print('[sim] Ground-truth display disabled by SIM_SHOW_DISPLAY')
        return

    def _display_loop():
        try:
            import matplotlib.pyplot as plt
            from matplotlib.patches import Rectangle
        except Exception as exc:
            print(f'[sim] Ground-truth display unavailable: {exc}')
            return

        fig, ax = plt.subplots(figsize=(7, 7))
        fig.canvas.manager.set_window_title('Simulator Ground Truth')

        # Draw world boundary.
        ax.add_patch(
            Rectangle(
                (WORLD_MIN_MM, WORLD_MIN_MM),
                WORLD_MAX_MM - WORLD_MIN_MM,
                WORLD_MAX_MM - WORLD_MIN_MM,
                fill=False,
                edgecolor='black',
                linewidth=2,
            )
        )

        # Draw fixed color regions first so obstacles remain visible on top.
        for region in COLOR_REGIONS:
            x0, y0, x1, y1 = region.rect
            ax.add_patch(
                Rectangle(
                    (x0, y0),
                    x1 - x0,
                    y1 - y0,
                    facecolor=region.display_color,
                    edgecolor='black',
                    linewidth=1.0,
                    alpha=0.22,
                )
            )
            ax.text(
                (x0 + x1) / 2.0,
                (y0 + y1) / 2.0,
                region.name.upper(),
                ha='center',
                va='center',
                fontsize=10,
                fontweight='bold',
                color='black',
            )

        # Draw obstacles on top.
        for (x0, y0, x1, y1) in OBSTACLES:
            ax.add_patch(
                Rectangle(
                    (x0, y0),
                    x1 - x0,
                    y1 - y0,
                    facecolor='dimgray',
                    edgecolor='black',
                    alpha=0.8,
                )
            )

        robot_dot, = ax.plot([], [], marker='o', color='tab:blue', markersize=8)
        heading, = ax.plot([], [], color='tab:blue', linewidth=2)
        status_text = ax.text(
            0.02,
            0.98,
            '',
            transform=ax.transAxes,
            va='top',
            family='monospace',
            fontsize=9,
            bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': 'none'},
        )

        ax.set_xlim(WORLD_MIN_MM - 100, WORLD_MAX_MM + 100)
        ax.set_ylim(WORLD_MIN_MM - 100, WORLD_MAX_MM + 100)
        ax.set_aspect('equal', adjustable='box')
        ax.set_title('Simulator Environment + Ground Truth Robot State')
        ax.set_xlabel('x (mm)')
        ax.set_ylabel('y (mm)')
        ax.grid(alpha=0.2)

        while plt.fignum_exists(fig.number):
            x = STATE.x_mm
            y = STATE.y_mm
            theta = STATE.theta_deg

            robot_dot.set_data([x], [y])

            r = 180.0
            th = math.radians(theta)
            x2 = x + r * math.cos(th)
            y2 = y + r * math.sin(th)
            heading.set_data([x, x2], [y, y2])

            status_text.set_text(
                f'pose: x={x:7.1f} y={y:7.1f} th={theta:6.1f}\n'
                f'drive: {STATE.drive_mode:>8}  speed={STATE.motor_speed:3d}\n'
                f'estop: {"STOPPED" if STATE.estop_state == STATE_STOPPED else "RUNNING"}\n'
                f'camera_captures: {STATE.camera_captures:3d}\n'
                f'color: {(_region_at(x, y).name if _region_at(x, y) else "none"):>5}\n'
                f'arm: B={STATE.arm_base} S={STATE.arm_shoulder} '
                f'E={STATE.arm_elbow} G={STATE.arm_gripper} V={STATE.arm_speed}'
            )

            fig.canvas.draw_idle()
            plt.pause(0.05)

    _display_thread = threading.Thread(target=_display_loop, daemon=True)
    _display_thread.start()
    print('[sim] Ground-truth display started')


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

    mapping = {
        'B': COMMAND_ARM_BASE,
        'S': COMMAND_ARM_SHOULDER,
        'E': COMMAND_ARM_ELBOW,
        'G': COMMAND_ARM_GRIPPER,
        'V': COMMAND_ARM_SET_SPEED,
    }
    pkt_cmd = mapping.get(cmd[0])
    if pkt_cmd is None:
        return None
    return pkt_cmd, int(cmd[1:])


# ---------------------------------------------------------------------------
# TCP server that mirrors rp_lidar_api.py
# ---------------------------------------------------------------------------


async def handle_client(reader, writer):
    addr = writer.get_extra_info('peername')
    print(f"[sim] Connected by {addr}")

    lidar_instances = {}
    scan_generators = {}
    next_lidar_id = 1

    try:
        while True:
            command_data = await reader.readexactly(4)
            command = struct.unpack('i', command_data)[0]

            if command == net_params.CMD_QUIT:
                print("[sim] Client terminated connection")
                break

            elif command == net_params.CMD_CONNECT:
                first_data = await reader.readexactly(4)
                first_value = struct.unpack('i', first_data)[0]

                if 0 <= first_value <= 1024:
                    port_len = first_value
                    await reader.readexactly(port_len)
                    await reader.readexactly(4)
                else:
                    await reader.readexactly(4)

                lidar = connect()
                lidar_id = next_lidar_id
                next_lidar_id += 1
                lidar_instances[lidar_id] = lidar
                _send_bool(writer, True)
                writer.write(struct.pack('i', lidar_id))
                print(f"[sim] Connected lidar {lidar_id}")
                await writer.drain()

            elif command == net_params.CMD_GET_SCAN_MODE:
                lidar_id = struct.unpack('i', await reader.readexactly(4))[0]
                mode = get_scan_mode(lidar_instances.get(lidar_id)) if lidar_id in lidar_instances else -1
                writer.write(struct.pack('i', mode))
                await writer.drain()

            elif command == net_params.CMD_START_SCAN_ROUNDS:
                lidar_id = struct.unpack('i', await reader.readexactly(4))[0]
                mode = struct.unpack('i', await reader.readexactly(4))[0]
                if lidar_id in lidar_instances:
                    scan_generators[lidar_id] = scan_rounds(lidar_instances[lidar_id], mode)
                    _send_bool(writer, True)
                    print(f"[sim] Started scan rounds for lidar {lidar_id} (mode {mode})")
                else:
                    _send_bool(writer, False)
                await writer.drain()

            elif command == net_params.CMD_GET_NEXT_SCAN_ROUND:
                lidar_id = struct.unpack('i', await reader.readexactly(4))[0]
                gen = scan_generators.get(lidar_id)
                if gen is None:
                    _send_bool(writer, False)
                    await writer.drain()
                    continue

                try:
                    angles, distances = next(gen)
                    _send_bool(writer, True)
                    writer.write(struct.pack('i', len(angles)))
                    for angle in angles:
                        writer.write(struct.pack('f', float(angle)))
                    for distance in distances:
                        writer.write(struct.pack('f', float(distance)))
                    await writer.drain()
                except StopIteration:
                    _send_bool(writer, False)
                    await writer.drain()

            elif command == net_params.CMD_DISCONNECT:
                lidar_id = struct.unpack('i', await reader.readexactly(4))[0]
                lidar_instances.pop(lidar_id, None)
                scan_generators.pop(lidar_id, None)
                _send_bool(writer, True)
                await writer.drain()
                print(f"[sim] Disconnected lidar {lidar_id}")

            elif command == net_params.CMD_SENSOR_SERIAL_CONNECT:
                first_value = struct.unpack('i', await reader.readexactly(4))[0]
                if 0 <= first_value <= 1024:
                    port_len = first_value
                    await reader.readexactly(port_len)
                    await reader.readexactly(4)
                else:
                    await reader.readexactly(4)
                _send_bool(writer, True)
                await writer.drain()
                print("[sim] Sensor serial connect requested")

            elif command == net_params.CMD_SENSOR_SERIAL_DISCONNECT:
                _send_bool(writer, True)
                await writer.drain()
                print("[sim] Sensor serial disconnect requested")

            elif command == net_params.CMD_SENSOR_ESTOP:
                STATE.estop_state = STATE_STOPPED if STATE.estop_state == STATE_RUNNING else STATE_RUNNING
                if STATE.estop_state == STATE_STOPPED:
                    # E-stop latches the simulator into an immediate no-motion state.
                    STATE.drive_mode = 'STOP'
                _send_bool(writer, True)
                await writer.drain()
                if STATE.estop_state == STATE_STOPPED:
                    print('[sim] E-STOP toggled -> STOPPED (drive halted, arm movement blocked)')
                else:
                    print('[sim] E-STOP toggled -> RUNNING')

            elif command == net_params.CMD_SENSOR_GET_STATUS:
                writer.write(struct.pack('<bi', 1, int(STATE.estop_state)))
                await writer.drain()

            elif command == net_params.CMD_SENSOR_GET_COLOR:
                r, g, b = _simulate_color_sensor()
                writer.write(struct.pack('<biii', 1, r, g, b))
                await writer.drain()
                region = _region_at(STATE.x_mm, STATE.y_mm)
                region_name = region.name if region else 'none'
                print(f"[sim] Color sensor ({region_name}) -> R={r} G={g} B={b}")

            elif command == net_params.CMD_SENSOR_DRIVE:
                drive_cmd = struct.unpack('i', await reader.readexactly(4))[0]
                if STATE.estop_state == STATE_STOPPED:
                    _send_bool(writer, False)
                else:
                    _update_drive(drive_cmd)
                    _send_bool(writer, True)
                await writer.drain()

            elif command == net_params.CMD_SENSOR_SET_SPEED:
                speed = struct.unpack('i', await reader.readexactly(4))[0]
                STATE.motor_speed = int(_clamp(speed, 0, 255))
                _send_bool(writer, True)
                await writer.drain()
                print(f"[sim] Motor speed -> {STATE.motor_speed}")

            elif command == net_params.CMD_SENSOR_CAMERA_CAPTURE:
                if STATE.estop_state == STATE_STOPPED:
                    _send_bool(writer, False)
                else:
                    ok = _send_dummy_camera_frame()
                    if ok:
                        STATE.camera_captures += 1
                    _send_bool(writer, ok)
                await writer.drain()
                print(f"[sim] Camera captures -> {STATE.camera_captures}")

            elif command == net_params.CMD_SENSOR_ARM_TEXT:
                cmd_len = struct.unpack('i', await reader.readexactly(4))[0]
                text_cmd = (await reader.readexactly(cmd_len)).decode('utf-8', errors='replace')
                if STATE.estop_state == STATE_STOPPED:
                    _send_bool(writer, False)
                    await writer.drain()
                    print('[sim] ARM text command rejected: E-STOP active')
                    continue
                parsed = _parse_arm_text_command(text_cmd)
                if parsed is None:
                    _send_bool(writer, False)
                    await writer.drain()
                    continue
                pkt_cmd, value = parsed
                ok = _handle_arm_command(pkt_cmd, value)
                _send_bool(writer, ok)
                await writer.drain()
                if ok:
                    print(f"[sim] ARM text command accepted: {text_cmd.strip().upper()}")

            elif command in {
                COMMAND_ARM_BASE,
                COMMAND_ARM_SHOULDER,
                COMMAND_ARM_ELBOW,
                COMMAND_ARM_GRIPPER,
                COMMAND_ARM_HOME,
                COMMAND_ARM_SET_SPEED,
            }:
                if STATE.estop_state == STATE_STOPPED:
                    _send_bool(writer, False)
                    await writer.drain()
                    print('[sim] ARM packet command rejected: E-STOP active')
                    continue
                value = None
                if command != COMMAND_ARM_HOME:
                    value = struct.unpack('i', await reader.readexactly(4))[0]
                ok = _handle_arm_command(command, value)
                _send_bool(writer, ok)
                await writer.drain()

            else:
                print(f"[sim] Unknown command: {command}")

    except asyncio.IncompleteReadError:
        print(f"[sim] Connection closed by {addr}")
    except Exception as exc:
        print(f"[sim] Client error {addr}: {exc}")
    finally:
        writer.close()
        await writer.wait_closed()
        print(f"[sim] Closed connection for {addr}")


async def main():
    _start_sim_display()
    server = await asyncio.start_server(handle_client, '0.0.0.0', net_params.server_lidar_port)
    addrs = ', '.join(str(sock.getsockname()) for sock in server.sockets)
    print(f'[sim] Serving on {addrs}')

    async with server:
        await server.serve_forever()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[sim] Server stopped manually.")
