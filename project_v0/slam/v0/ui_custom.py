#!/usr/bin/env python3
"""
ui_custom.py - Windowed SLAM UI (non-terminal).

Features:
  - Live occupancy-map rendering in a matplotlib window.
  - Robot position + heading arrow for clearer direction.
  - Color sensing stamps: each manual color read places a dot on map.
  - Keyboard controls for movement/speed/estop/camera/pause.
  - Robot arm text command entry (X000 format, e.g. B090, S110, E080, G095, V020, H).
"""

from __future__ import annotations

import multiprocessing
import math
import time

import numpy as np

from settings import MAP_SIZE_PIXELS, MAP_SIZE_METERS, UI_REFRESH_HZ
from shared_state import ProcessSharedState
from slam_process import run_slam_process
import lidar

try:
    import second_terminal_manager as second_term
    _second_term_available = True
except ImportError:
    _second_term_available = False


CAMERA_CAPTURE_LIMIT_UI = 10
MAP_BYTES_SIZE = MAP_SIZE_PIXELS * MAP_SIZE_PIXELS
MIN_VIEW_SPAN_PX = 2.0
MIN_AXES_BOX_PX = 5.0
VIEW_HEALTH_CHECK_INTERVAL_SEC = 1.0


class SlamCustomUI:
    def __init__(self):
        self.pss = ProcessSharedState()
        self.slam_proc = multiprocessing.Process(
            target=run_slam_process,
            args=(self.pss,),
            name='slam-process',
            daemon=True,
        )

        self._last_map_version = -1
        self._last_pose_version = -1
        self._status_msg = 'starting...'
        self._camera_count = 0

        # Color stamps in map pixel coordinates.
        self._red_points = []
        self._green_points = []
        self._blue_points = []

        self._motor_speed = 150
        self._display_rotation_deg = 0.0
        self._last_rotation_deg = None

        n = MAP_SIZE_PIXELS - 1
        self._view_center_x = n / 2.0
        self._view_center_y = n / 2.0
        self._zoom_levels_px = [n / 2.0, n / 3.0, n / 5.0, n / 8.0]
        self._zoom_idx = 1

        self._fig = None
        self._ax_map = None
        self._ax_info = None
        self._img_artist = None
        self._robot_point = None
        self._robot_arrow = None
        self._red_scatter = None
        self._green_scatter = None
        self._blue_scatter = None
        self._info_text = None
        self._timer = None
        self._is_shutting_down = False
        self._next_view_health_check = 0.0

    def _snapshot(self):
        if self._is_shutting_down:
            return {
                'mapbytes': bytes(MAP_BYTES_SIZE),
                'x_mm': 0.0,
                'y_mm': 0.0,
                'theta_deg': 0.0,
                'valid_points': 0,
                'raw_points': 0,
                'status_note': 'shutting down',
                'rounds_seen': 0,
                'map_version': 0,
                'pose_version': 0,
                'connected': False,
                'paused': False,
                'stopped': True,
                'error_message': None,
            }

        shm = getattr(self.pss, 'shm', None)
        if shm is None:
            return {
                'mapbytes': bytes(MAP_BYTES_SIZE),
                'x_mm': 0.0,
                'y_mm': 0.0,
                'theta_deg': 0.0,
                'valid_points': 0,
                'raw_points': 0,
                'status_note': 'shared memory unavailable',
                'rounds_seen': 0,
                'map_version': 0,
                'pose_version': 0,
                'connected': False,
                'paused': False,
                'stopped': True,
                'error_message': 'shared memory unavailable',
            }

        error = self.pss.get_error()
        return {
            'mapbytes': bytes(shm.buf[:MAP_BYTES_SIZE]),
            'x_mm': self.pss.x_mm.value,
            'y_mm': self.pss.y_mm.value,
            'theta_deg': self.pss.theta_deg.value,
            'valid_points': self.pss.valid_points.value,
            'raw_points': self.pss.raw_points.value,
            'status_note': self.pss.get_status(),
            'rounds_seen': self.pss.rounds_seen.value,
            'map_version': self.pss.map_version.value,
            'pose_version': self.pss.pose_version.value,
            'connected': self.pss.connected.value,
            'paused': self.pss.paused.value,
            'stopped': self.pss.stopped.value,
            'error_message': error if error else None,
        }

    @staticmethod
    def _map_image_from_bytes(mapbytes: bytes) -> np.ndarray:
        arr = np.frombuffer(mapbytes[:MAP_BYTES_SIZE], dtype=np.uint8).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
        # Display with x to the right and y upward (north-up map view).
        arr = np.flipud(arr)
        return arr

    @staticmethod
    def _mm_to_display_px(x_mm: float, y_mm: float) -> tuple[float, float]:
        px_per_mm = MAP_SIZE_PIXELS / (MAP_SIZE_METERS * 1000.0)
        col = x_mm * px_per_mm
        row = (MAP_SIZE_PIXELS - 1) - (y_mm * px_per_mm)
        return col, row

    @staticmethod
    def _rotate_display_point(px: float, py: float, quarter_turns: int) -> tuple[float, float]:
        n = MAP_SIZE_PIXELS - 1
        cx = n / 2.0
        cy = n / 2.0
        theta = math.radians(quarter_turns)
        dx = px - cx
        dy = py - cy
        rx = cx + (math.cos(theta) * dx) - (math.sin(theta) * dy)
        ry = cy + (math.sin(theta) * dx) + (math.cos(theta) * dy)
        return rx, ry

    @staticmethod
    def _estimate_wall_angle_deg(mapbytes: bytes):
        """Estimate dominant wall direction in degrees modulo 180.

        Returns angle where 0 means walls mostly horizontal in image-space,
        90 means mostly vertical.
        """
        img = SlamCustomUI._map_image_from_bytes(mapbytes)
        wall = (img < 90).astype(np.float32)
        if float(wall.sum()) < 500.0:
            return None

        gy, gx = np.gradient(wall)
        mag = np.hypot(gx, gy)
        mask = mag > 0.12
        if int(mask.sum()) < 300:
            return None

        # Gradient direction is wall normal; add 90 deg to get wall direction.
        normal_deg = np.degrees(np.arctan2(gy[mask], gx[mask]))
        wall_deg = (normal_deg + 90.0) % 180.0
        w = mag[mask]

        # Circular mean on doubled angle for 180-degree periodicity.
        ang2 = np.radians(wall_deg * 2.0)
        c = float(np.sum(w * np.cos(ang2)))
        s = float(np.sum(w * np.sin(ang2)))
        if abs(c) < 1e-9 and abs(s) < 1e-9:
            return None
        mean = (math.degrees(math.atan2(s, c)) / 2.0) % 180.0
        return mean

    def _rezero_display_orientation(self):
        snap = self._snapshot()
        wall_angle = self._estimate_wall_angle_deg(snap['mapbytes'])
        if wall_angle is None:
            self._status_msg = '[VIEW] re-zero failed: not enough wall structure yet'
            return

        # Rotate display so dominant wall angle snaps to nearest room axis.
        nearest_axis = round(-wall_angle / 90.0) * 90.0
        self._display_rotation_deg = nearest_axis - wall_angle
        self._last_map_version = -1
        self._last_rotation_deg = None
        self._status_msg = (
            f'[VIEW] wall re-zero: wall={wall_angle:.1f} deg, '
            f'corr={self._display_rotation_deg:+.1f} deg'
        )

    def _clamp_view_center(self):
        n = MAP_SIZE_PIXELS - 1
        half = max(float(self._zoom_levels_px[self._zoom_idx]), MIN_VIEW_SPAN_PX / 2.0)
        self._view_center_x = max(half, min(n - half, self._view_center_x))
        self._view_center_y = max(half, min(n - half, self._view_center_y))

    def _apply_view_window(self):
        self._clamp_view_center()
        half = max(float(self._zoom_levels_px[self._zoom_idx]), MIN_VIEW_SPAN_PX / 2.0)
        cx = self._view_center_x
        cy = self._view_center_y
        # Keep x-axis mirrored as requested earlier.
        self._ax_map.set_xlim(cx + half, cx - half)
        self._ax_map.set_ylim(cy + half, cy - half)

    def _view_box_collapsed(self) -> bool:
        if self._fig is None or self._ax_map is None:
            return False

        try:
            x0, x1 = self._ax_map.get_xlim()
            y0, y1 = self._ax_map.get_ylim()
        except Exception:
            return True

        spans = (abs(float(x1) - float(x0)), abs(float(y1) - float(y0)))
        if any((not math.isfinite(span)) or span < MIN_VIEW_SPAN_PX for span in spans):
            return True

        canvas = getattr(self._fig, 'canvas', None)
        if canvas is None:
            return False

        try:
            renderer = canvas.get_renderer()
            bbox = self._ax_map.get_window_extent(renderer=renderer)
        except Exception:
            return False

        return bbox.width < MIN_AXES_BOX_PX or bbox.height < MIN_AXES_BOX_PX

    def _pan_view(self, dx_px: float, dy_px: float):
        self._view_center_x += dx_px
        self._view_center_y += dy_px
        self._apply_view_window()

    def _zoom_view(self, delta: int):
        old_idx = self._zoom_idx
        self._zoom_idx = max(0, min(len(self._zoom_levels_px) - 1, self._zoom_idx + delta))
        if self._zoom_idx != old_idx:
            self._apply_view_window()

    def _nudge_alignment(self, delta_deg: float):
        self._display_rotation_deg += delta_deg
        self._last_rotation_deg = None
        self._status_msg = f'[VIEW] manual align {self._display_rotation_deg:+.1f} deg'

    def _go_home_view(self):
        n = MAP_SIZE_PIXELS - 1
        self._view_center_x = n / 2.0
        self._view_center_y = n / 2.0
        self._zoom_idx = 1
        self._display_rotation_deg = 0.0
        self._last_rotation_deg = None
        self._next_view_health_check = 0.0
        self._apply_view_window()
        self._status_msg = '[VIEW] home'

    def _request_slam_reset(self):
        self.pss.reset_event.set()
        self._red_points.clear()
        self._green_points.clear()
        self._blue_points.clear()
        self._camera_count = 0
        self._last_map_version = -1
        self._last_pose_version = -1
        self._go_home_view()
        self._status_msg = '[SLAM] reset requested'

    @staticmethod
    def _classify_color(r: int, g: int, b: int) -> str:
        # Frequency-based sensor: often lower freq means stronger reflected channel.
        vals = {'red': r, 'green': g, 'blue': b}
        return min(vals, key=vals.get)

    def _stamp_color_at_robot(self, color_name: str, x_mm: float, y_mm: float):
        px, py = self._mm_to_display_px(x_mm, y_mm)
        point = (float(px), float(py))
        if color_name == 'red':
            self._red_points.append(point)
        elif color_name == 'green':
            self._green_points.append(point)
        elif color_name == 'blue':
            self._blue_points.append(point)

    def _current_pose_mm(self) -> tuple[float, float]:
        return float(self.pss.x_mm.value), float(self.pss.y_mm.value)

    @staticmethod
    def _points_to_xy(points):
        if not points:
            return [], []
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return xs, ys

    def _handle_key(self, event):
        key = (event.key or '').lower()
        if not key:
            return

        if key == 'q':
            self._status_msg = 'quitting...'
            self._shutdown()
            return

        if key == 'o':
            self._rezero_display_orientation()
            return

        if key == 'h':
            self._go_home_view()
            return

        if key == 'j':
            self._nudge_alignment(+1.0)
            return

        if key == 'l':
            self._nudge_alignment(-1.0)
            return

        if key == 'i':
            self._zoom_view(+1)
            self._status_msg = f'[VIEW] zoom {self._zoom_idx + 1}/{len(self._zoom_levels_px)}'
            return

        if key == 'k':
            self._zoom_view(-1)
            self._status_msg = f'[VIEW] zoom {self._zoom_idx + 1}/{len(self._zoom_levels_px)}'
            return

        pan_step = max(20.0, self._zoom_levels_px[self._zoom_idx] * 0.20)
        if key == 'left':
            self._pan_view(+pan_step, 0.0)
            self._status_msg = '[VIEW] pan left'
            return
        if key == 'right':
            self._pan_view(-pan_step, 0.0)
            self._status_msg = '[VIEW] pan right'
            return
        if key == 'up':
            self._pan_view(0.0, -pan_step)
            self._status_msg = '[VIEW] pan up'
            return
        if key == 'down':
            self._pan_view(0.0, +pan_step)
            self._status_msg = '[VIEW] pan down'
            return

        if key == 'p':
            self.pss.paused.value = not self.pss.paused.value
            self._status_msg = 'paused' if self.pss.paused.value else 'resumed'
            return

        if key == 'r':
            self._request_slam_reset()
            return

        if key == 'e':
            ok = lidar.sensor_estop()
            self._status_msg = '[E-STOP] sent' if ok else '[E-STOP] failed'
            return

        if key == 'f':
            if self._camera_count >= CAMERA_CAPTURE_LIMIT_UI:
                self._status_msg = f'[CAMERA] limit reached ({CAMERA_CAPTURE_LIMIT_UI})'
                return
            ok = lidar.sensor_camera_capture()
            if ok:
                self._camera_count += 1
                self._status_msg = (
                    f'[CAMERA] captured ({self._camera_count}/{CAMERA_CAPTURE_LIMIT_UI})'
                )
            else:
                self._status_msg = '[CAMERA] failed'
            return

        if key == 'c':
            rgb = lidar.sensor_get_color()
            if rgb:
                r, g, b = int(rgb[0]), int(rgb[1]), int(rgb[2])
                snap = self._snapshot()
                color_name = self._classify_color(r, g, b)
                self._stamp_color_at_robot(color_name, snap['x_mm'], snap['y_mm'])
                self._status_msg = f'[COLOR] R={r} G={g} B={b} -> {color_name} dot placed'
            else:
                self._status_msg = '[COLOR] read failed'
            return

        if key == 'w':
            ok = lidar.sensor_forward()
            self._status_msg = '[DRIVE] forward' if ok else '[DRIVE] forward failed'
            return
        if key == 's':
            ok = lidar.sensor_backward()
            self._status_msg = '[DRIVE] backward' if ok else '[DRIVE] backward failed'
            return
        if key == 'a':
            ok = lidar.sensor_left()
            self._status_msg = '[DRIVE] left' if ok else '[DRIVE] left failed'
            return
        if key == 'd':
            ok = lidar.sensor_right()
            self._status_msg = '[DRIVE] right' if ok else '[DRIVE] right failed'
            return
        if key == 'x':
            ok = lidar.sensor_stop()
            self._status_msg = '[DRIVE] stop' if ok else '[DRIVE] stop failed'
            return

        if key == '[':
            self._motor_speed = max(0, self._motor_speed - 20)
            ok = lidar.sensor_set_speed(self._motor_speed)
            self._status_msg = f'[SPEED] {self._motor_speed}' if ok else '[SPEED] failed'
            return
        if key == ']':
            self._motor_speed = min(255, self._motor_speed + 20)
            ok = lidar.sensor_set_speed(self._motor_speed)
            self._status_msg = f'[SPEED] {self._motor_speed}' if ok else '[SPEED] failed'
            return

    def _refresh(self):
        if _second_term_available:
            try:
                second_term.pump()
            except Exception:
                pass

        now = time.monotonic()
        if now >= self._next_view_health_check:
            self._next_view_health_check = now + VIEW_HEALTH_CHECK_INTERVAL_SEC
            if self._view_box_collapsed():
                self._status_msg = '[VIEW] collapsed view recovered'
                self._go_home_view()

        snap = self._snapshot()

        if snap['map_version'] != self._last_map_version:
            img = self._map_image_from_bytes(snap['mapbytes'])
            self._img_artist.set_data(img)
            self._last_map_version = snap['map_version']

        if self._last_rotation_deg != self._display_rotation_deg:
            from matplotlib.transforms import Affine2D
            n = MAP_SIZE_PIXELS - 1
            cx = n / 2.0
            cy = n / 2.0
            tr = Affine2D().rotate_deg_around(cx, cy, self._display_rotation_deg)
            self._img_artist.set_transform(tr + self._ax_map.transData)
            self._last_rotation_deg = self._display_rotation_deg

        if snap['pose_version'] != self._last_pose_version:
            px, py = self._mm_to_display_px(snap['x_mm'], snap['y_mm'])
            px, py = self._rotate_display_point(px, py, self._display_rotation_deg)
            self._robot_point.set_data([px], [py])

            # theta_deg is CCW from +x; invert y for screen coordinates.
            heading = math.radians(snap['theta_deg'] - self._display_rotation_deg)
            u = 40.0 * math.cos(heading)
            v = -40.0 * math.sin(heading)
            self._robot_arrow.set_offsets(np.array([[px, py]]))
            self._robot_arrow.set_UVC(np.array([u]), np.array([v]))
            self._last_pose_version = snap['pose_version']

        rx, ry = self._points_to_xy(self._red_points)
        gx, gy = self._points_to_xy(self._green_points)
        bx, by = self._points_to_xy(self._blue_points)
        if abs(self._display_rotation_deg) > 1e-6:
            red_points = [self._rotate_display_point(x, y, self._display_rotation_deg) for x, y in self._red_points]
            green_points = [self._rotate_display_point(x, y, self._display_rotation_deg) for x, y in self._green_points]
            blue_points = [self._rotate_display_point(x, y, self._display_rotation_deg) for x, y in self._blue_points]
            rx, ry = self._points_to_xy(red_points)
            gx, gy = self._points_to_xy(green_points)
            bx, by = self._points_to_xy(blue_points)
        self._red_scatter.set_offsets(np.column_stack([rx, ry]) if rx else np.empty((0, 2)))
        self._green_scatter.set_offsets(np.column_stack([gx, gy]) if gx else np.empty((0, 2)))
        self._blue_scatter.set_offsets(np.column_stack([bx, by]) if bx else np.empty((0, 2)))

        state = 'PAUSED' if snap['paused'] else 'LIVE'
        if snap['error_message']:
            state = 'ERROR'
        elif snap['stopped'] and not snap['connected']:
            state = 'STOPPED'

        info = (
            f"State: {state}\n"
            f"Pose: x={snap['x_mm']:.0f} y={snap['y_mm']:.0f} th={snap['theta_deg']:+.1f} deg | "
            f"Rounds={snap['rounds_seen']} Valid={snap['valid_points']} Raw={snap['raw_points']}\n"
            f"View: z{self._zoom_idx + 1}/{len(self._zoom_levels_px)} "
            f"ctr=({self._view_center_x:.0f},{self._view_center_y:.0f}) rot={self._display_rotation_deg:+.1f} deg\n"
            f"Camera={self._camera_count} Colors(R/G/B)={len(self._red_points)}/{len(self._green_points)}/{len(self._blue_points)}\n"
            f"Action: {self._status_msg}\n"
            f"Keys: WASD/X [] C F E P R | Arrows I K O J L H Q | 2nd terminal: second_terminal/second_terminal.py"
        )
        if snap['error_message']:
            info += f"\nError: {snap['error_message']}"

        self._info_text.set_text(info)
        self._fig.canvas.draw_idle()

    def _shutdown(self):
        if self._is_shutting_down:
            return
        self._is_shutting_down = True

        try:
            if self._timer is not None:
                self._timer.stop()
        except Exception:
            pass
        try:
            if _second_term_available:
                second_term.shutdown()
        except Exception:
            pass
        try:
            self.pss.stop_event.set()
            if self.slam_proc.is_alive():
                self.slam_proc.join(timeout=3.0)
            if self.slam_proc.is_alive():
                self.slam_proc.terminate()
            self.pss.cleanup()
        except Exception:
            pass

        try:
            lidar.sensor_serial_disconnect()
        except Exception:
            pass

    def run(self):
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print('[slam] ERROR: matplotlib is not installed.')
            print('Install it in your environment, then run slam.py again.')
            raise SystemExit(1)

        # Prevent matplotlib's default shortcuts from stealing the keys we use
        # for robot control in this UI window.
        plt.rcParams['keymap.save'] = []
        plt.rcParams['keymap.fullscreen'] = []

        # Start second terminal relay first so remote clients can connect
        # even while LIDAR/API initialization is still in progress.
        if _second_term_available:
            try:
                second_term.configure_ui_callbacks(
                    dot_callback=self._stamp_color_at_robot,
                    pose_callback=self._current_pose_mm,
                )
            except Exception:
                pass
            second_term.start()

        self.slam_proc.start()
        try:
            lidar.sensor_serial_connect()
        except Exception:
            # SLAM map should still run even if sensor API side is unavailable.
            self._status_msg = 'sensor serial connect failed (map still running)'

        self._fig = plt.figure(figsize=(9, 10))
        self._fig.canvas.manager.set_window_title('SLAM Custom UI')
        gs = self._fig.add_gridspec(2, 1, height_ratios=[18, 5])

        self._ax_map = self._fig.add_subplot(gs[0, 0])
        self._ax_info = self._fig.add_subplot(gs[1, 0])

        self._ax_map.set_title('Occupancy Map')
        self._ax_map.set_xlim(MAP_SIZE_PIXELS - 1, 0)
        self._ax_map.set_ylim(MAP_SIZE_PIXELS - 1, 0)
        self._ax_map.set_aspect('equal', adjustable='datalim')
        self._ax_map.set_autoscale_on(False)

        init_img = np.full((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), 127, dtype=np.uint8)
        self._img_artist = self._ax_map.imshow(init_img, cmap='gray', vmin=0, vmax=255, origin='upper')

        self._robot_point, = self._ax_map.plot([], [], marker='o', color='cyan', markersize=8)
        self._robot_arrow = self._ax_map.quiver([0], [0], [0], [0], color='cyan', scale_units='xy', scale=1)

        self._red_scatter = self._ax_map.scatter([], [], c='red', s=20, label='Red')
        self._green_scatter = self._ax_map.scatter([], [], c='lime', s=20, label='Green')
        self._blue_scatter = self._ax_map.scatter([], [], c='blue', s=20, label='Blue')
        self._ax_map.legend(loc='upper right', fontsize=8)

        self._ax_info.axis('off')
        self._info_text = self._ax_info.text(0.0, 1.0, '', va='top', family='monospace', fontsize=9)

        self._fig.canvas.mpl_connect('key_press_event', self._handle_key)
        self._fig.canvas.mpl_connect('close_event', lambda evt: None)

        self._timer = self._fig.canvas.new_timer(interval=max(20, int(1000 / UI_REFRESH_HZ)))
        self._timer.add_callback(self._refresh)
        self._timer.start()

        self._apply_view_window()
        self._refresh()
        # Keep a stable layout: tight_layout can occasionally over-compress
        # the map axis when info text changes or the window is resized.
        self._fig.subplots_adjust(left=0.05, right=0.98, top=0.95, bottom=0.05, hspace=0.10)

        try:
            plt.show()
        finally:
            self._shutdown()


def run() -> None:
    SlamCustomUI().run()
