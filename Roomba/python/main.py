# SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
#
# SPDX-License-Identifier: MPL-2.0
#
# ARIA — Unified main.py
# Combines:
#   • Object detection (original WebUI app — untouched)
#   • Phase 3: EKF localization + occupancy grid (background thread)
#   • Camera MJPEG stream + video/audio recording
#   • Manual robot control via WebUI
#
# Run:   python main.py               ← simulator mode
#        python main.py --sim         ← explicit simulator
#        python main.py --port /dev/ttyUSB0  ← real hardware

# ════════════════════════════════════════════════════════════════════════════
# 1. STANDARD LIBRARY
# ════════════════════════════════════════════════════════════════════════════
import io
import os
import base64
import time
import threading
import logging
import subprocess
import sys

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(message)s')
log = logging.getLogger('ARIA')

# ════════════════════════════════════════════════════════════════════════════
# 2. ARDUINO APP LAB (original imports — untouched)
# ════════════════════════════════════════════════════════════════════════════
from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.object_detection import ObjectDetection
from PIL import Image

# ════════════════════════════════════════════════════════════════════════════
# 3. ARIA NAVIGATION (Phase 3 + 4)
# ════════════════════════════════════════════════════════════════════════════
from aria import (
    ARIALocalization, OccupancyGrid, BridgeStub,
    BoustrophedonPlanner, PotentialFieldSteering,
    CleaningStateMachine, CleanState, MotorCommand,
)
from aria.config import (
    BRIDGE_UPLINK_HZ, GRID_VISUALIZE_HZ,
    US_WALL_SNAP_CM, CELL_SIZE_CM,
)

# ════════════════════════════════════════════════════════════════════════════
# 4. OPTIONAL: CAMERA (gracefully skipped if OpenCV not installed)
# ════════════════════════════════════════════════════════════════════════════
try:
    import cv2
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False
    log.warning("opencv-python not installed — camera stream disabled. "
                "Install with: pip install opencv-python-headless")

# ════════════════════════════════════════════════════════════════════════════
# 5. RUNTIME CONFIG
# ════════════════════════════════════════════════════════════════════════════
GRID_SAVE_PATH     = os.path.expanduser('~/.aria/aria_grid.npy')
GRID_SAVE_INTERVAL = 60.0          # seconds between autosaves
NAV_PUSH_HZ        = 2.0           # how often to push state to WebUI
CAM_DEVICE         = 0             # /dev/video0 — change if needed
CAM_STREAM_WIDTH   = 320
CAM_STREAM_HEIGHT  = 240
CAM_STREAM_QUALITY = 70            # JPEG quality (0–100)
RECORDING_DIR      = os.path.expanduser('~/.aria/recordings')

# Bridge mode: set --port on command line, or force sim here
_PORT = None
_argv = sys.argv[1:]
for i, arg in enumerate(_argv):
    if arg == '--port' and i + 1 < len(_argv):
        _PORT = _argv[i + 1]   # value immediately after --port
    elif arg == '--sim':
        _PORT = None


# ════════════════════════════════════════════════════════════════════════════
# 6. HELPER: BRIDGE FACTORY  (must be defined before shared-state block)
# ════════════════════════════════════════════════════════════════════════════
def _get_hw_bridge(port: str, baud: int = 115200):
    """Try to load the real hardware bridge; fall back to simulator."""
    try:
        from aria.bridge_hw import BridgeHW   # type: ignore
        log.info(f"Hardware bridge: {port} @ {baud} baud")
        return BridgeHW(port, baud)
    except ImportError:
        log.warning("bridge_hw.py not found — using simulator")
        return BridgeStub()


# ════════════════════════════════════════════════════════════════════════════
# 7. SHARED STATE  (guarded by _state_lock where needed)
# ════════════════════════════════════════════════════════════════════════════
# Ensure save directory exists
os.makedirs(os.path.dirname(GRID_SAVE_PATH), exist_ok=True)
os.makedirs(RECORDING_DIR, exist_ok=True)

object_detection = ObjectDetection()

ekf     = ARIALocalization(start_x=0.0, start_y=0.0, start_theta=0.0)
grid    = (OccupancyGrid.load(GRID_SAVE_PATH)
           if os.path.exists(GRID_SAVE_PATH) else OccupancyGrid())
bridge  = BridgeStub() if _PORT is None else _get_hw_bridge(_PORT)

# Phase 4 — navigation stack (created once, shared with state machine)
_planner  = BoustrophedonPlanner(grid)
_steering = PotentialFieldSteering()
cleaner   = CleaningStateMachine(grid, _planner, _steering)

_state_lock      = threading.Lock()
_manual_mode     = False        # True = WebUI drives robot; False = autonomous
_manual_left     = 0            # last manual PWM values
_manual_right    = 0
_battery_pct     = 100.0        # Phase 5: overridden by on_set_battery / real ADC
_camera_cap      = None         # cv2.VideoCapture instance (when open)
_recording_proc  = None         # ffmpeg subprocess (when recording)
_recording_file  = None         # current recording filename
_nav_running     = True


# ════════════════════════════════════════════════════════════════════════════
# 7. OBJECT DETECTION HANDLER  (original code — untouched)
# ════════════════════════════════════════════════════════════════════════════
def on_detect_objects(client_id, data):
    """Callback function to handle object detection requests."""
    try:
        image_data = data.get('image')
        confidence = data.get('confidence', 0.5)
        if not image_data:
            ui.send_message('detection_error', {'error': 'No image data'})
            return

        image_bytes = base64.b64decode(image_data)
        pil_image   = Image.open(io.BytesIO(image_bytes))

        start_time = time.time() * 1000
        results    = object_detection.detect(pil_image, confidence=confidence)
        diff       = time.time() * 1000 - start_time

        if results is None:
            ui.send_message('detection_error', {'error': 'No results returned'})
            return

        img_with_boxes = object_detection.draw_bounding_boxes(pil_image, results)

        if img_with_boxes is not None:
            img_buffer = io.BytesIO()
            img_with_boxes.save(img_buffer, format="PNG")
            img_buffer.seek(0)
            b64_result = base64.b64encode(img_buffer.getvalue()).decode("utf-8")
        else:
            # If drawing fails, send back the original image
            img_buffer = io.BytesIO()
            pil_image.save(img_buffer, format="PNG")
            img_buffer.seek(0)
            b64_result = base64.b64encode(img_buffer.getvalue()).decode("utf-8")

        response = {
            'success':         True,
            'result_image':    b64_result,
            'detection_count': len(results.get("detection", [])) if results else 0,
            'processing_time': f"{diff:.2f} ms",
        }
        ui.send_message('detection_result', response)

    except Exception as e:
        ui.send_message('detection_error', {'error': str(e)})


# ════════════════════════════════════════════════════════════════════════════
# 8. NAVIGATION LOOP  (Phase 3 — runs in daemon thread)
# ════════════════════════════════════════════════════════════════════════════
def _navigation_loop() -> None:
    """
    20 Hz EKF + occupancy grid loop.
    Reads sensor packets from Bridge, updates EKF & grid,
    and periodically pushes state to the WebUI.
    """
    global _nav_running

    dt          = 1.0 / BRIDGE_UPLINK_HZ
    push_dt     = 1.0 / NAV_PUSH_HZ
    save_dt     = GRID_SAVE_INTERVAL

    next_tick   = time.monotonic()
    last_push   = 0.0
    last_save   = time.monotonic()
    last_viz    = 0.0
    viz_dt      = 1.0 / GRID_VISUALIZE_HZ

    prev_enc_l  = 0
    prev_enc_r  = 0
    first_pkt   = True

    log.info("Navigation loop started (%.0f Hz)" % BRIDGE_UPLINK_HZ)

    while _nav_running:
        now = time.monotonic()
        sleep_for = next_tick - now
        if sleep_for > 0:
            time.sleep(sleep_for)
        next_tick += dt

        # ── 1. Get sensors ────────────────────────────────────────────────
        try:
            pkt = bridge.get_sensors()
        except Exception as exc:
            log.error(f"Bridge read error: {exc}")
            continue

        # ── 2. Encoder deltas ─────────────────────────────────────────────
        if first_pkt:
            prev_enc_l = pkt.enc_left
            prev_enc_r = pkt.enc_right
            first_pkt  = False
            continue

        delta_l    = pkt.enc_left  - prev_enc_l
        delta_r    = pkt.enc_right - prev_enc_r
        prev_enc_l = pkt.enc_left
        prev_enc_r = pkt.enc_right

        # ── 3. EKF predict + correct ──────────────────────────────────────
        with _state_lock:
            ekf.predict(delta_l, delta_r)
            ekf.correct_imu(pkt.gyro_z, dt)

            us = pkt.ultrasonics

            # ── 4. Wall snap ──────────────────────────────────────────────
            x, y, theta = ekf.pose
            if us.get('left',  999) < US_WALL_SNAP_CM:
                ekf.wall_snap('left',  x - us['left']  + CELL_SIZE_CM / 2)
            if us.get('right', 999) < US_WALL_SNAP_CM:
                ekf.wall_snap('right', x + us['right'] - CELL_SIZE_CM / 2)

            # ── 5. Occupancy grid ─────────────────────────────────────────
            x, y, theta = ekf.pose
            grid.mark_cleaned(x, y)
            grid.update_from_ultrasonics(x, y, theta, us)

            coverage = grid.coverage_percent()

        # ── 6. Autonomous motor commands (Phase 4) ────────────────────────
        if not _manual_mode:
            with _state_lock:
                cmd = cleaner.step(
                    robot_x     = x,
                    robot_y     = y,
                    robot_theta = theta,
                    ultrasonics = us,
                    battery_pct = _battery_pct,   # updated by on_set_battery
                )
            bridge.set_motors(cmd.left, cmd.right)
            bridge.set_vacuum(cmd.vacuum)
            bridge.set_brush(cmd.brush)
        else:
            # Manual mode: WebUI drives directly
            bridge.set_motors(_manual_left, _manual_right)

        # ── 7. Push nav state to WebUI (2 Hz) ────────────────────────────
        now = time.monotonic()
        if now - last_push >= push_dt:
            _push_nav_state(x, y, theta, coverage, us)
            last_push = now

        # ── 8. Terminal viz ───────────────────────────────────────────────
        if now - last_viz >= viz_dt:
            grid.print_terminal(robot_x=x, robot_y=y)
            last_viz = now

        # ── 9. Autosave grid ──────────────────────────────────────────────
        if now - last_save >= save_dt:
            try:
                grid.save(GRID_SAVE_PATH)
                log.info(f"Grid saved → {GRID_SAVE_PATH}")
            except Exception as exc:
                log.warning(f"Grid save failed: {exc}")
            last_save = now

    # Final save on exit
    try:
        grid.save(GRID_SAVE_PATH)
        log.info(f"Final grid saved → {GRID_SAVE_PATH}  "
                 f"coverage={grid.coverage_percent():.1f}%")
    except Exception:
        pass


def _push_nav_state(x, y, theta, coverage, us):
    """Send current navigation state to WebUI (non-blocking best-effort)."""
    try:
        ui.send_message('nav_state', {
            'x':        round(x,        1),
            'y':        round(y,        1),
            'theta':    round(theta,    3),
            'coverage': round(coverage, 1),
            'state':    cleaner.state_name,
            'manual':   _manual_mode,
            'us':       {k: round(v, 1) for k, v in us.items()},
        })
    except Exception:
        pass   # UI may not be connected yet — silently skip


# ════════════════════════════════════════════════════════════════════════════
# 9. CAMERA STREAM LOOP  (daemon thread — skipped if cv2 not installed)
# ════════════════════════════════════════════════════════════════════════════
def _camera_stream_loop() -> None:
    """
    Captures frames from the USB camera and pushes MJPEG to the WebUI
    as base64 JPEG at ~10 fps.  Each frame is sent as a 'camera_frame'
    socket message so the browser can display a live feed.
    """
    global _camera_cap

    if not CAMERA_AVAILABLE:
        return

    cap = cv2.VideoCapture(CAM_DEVICE)
    if not cap.isOpened():
        log.warning(f"Camera /dev/video{CAM_DEVICE} not found — stream disabled")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_STREAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_STREAM_HEIGHT)

    with _state_lock:
        _camera_cap = cap

    log.info(f"Camera stream started ({CAM_STREAM_WIDTH}×{CAM_STREAM_HEIGHT})")
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, CAM_STREAM_QUALITY]
    frame_dt = 1.0 / 10.0   # 10 fps to the browser

    while _nav_running:
        t0 = time.monotonic()
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue

        _, buf = cv2.imencode('.jpg', frame, encode_params)
        b64 = base64.b64encode(buf.tobytes()).decode('utf-8')
        try:
            ui.send_message('camera_frame', {'frame': b64})
        except Exception:
            pass

        elapsed = time.monotonic() - t0
        sleep = frame_dt - elapsed
        if sleep > 0:
            time.sleep(sleep)

    cap.release()
    with _state_lock:
        _camera_cap = None
    log.info("Camera stream stopped")


# ════════════════════════════════════════════════════════════════════════════
# 10. WEBUI MESSAGE HANDLERS — Navigation & Manual Control
# ════════════════════════════════════════════════════════════════════════════

def on_manual_drive(client_id, data):
    """
    Browser sends: { left: <int -255..255>, right: <int -255..255> }
    Robot must be in manual mode (on_set_mode called first).
    """
    global _manual_left, _manual_right
    if not _manual_mode:
        return
    left  = max(-255, min(255, int(data.get('left',  0))))
    right = max(-255, min(255, int(data.get('right', 0))))
    with _state_lock:
        _manual_left  = left
        _manual_right = right
    bridge.set_motors(left, right)


def on_set_mode(client_id, data):
    """
    Browser sends: { mode: 'manual' } or { mode: 'auto' }
    Switches between manual WebUI control and autonomous navigation.
    """
    global _manual_mode, _manual_left, _manual_right
    mode = data.get('mode', 'auto')
    with _state_lock:
        _manual_mode  = (mode == 'manual')
        _manual_left  = 0
        _manual_right = 0
    bridge.set_motors(0, 0)   # safe stop on mode switch
    log.info(f"Mode set to: {mode.upper()}")
    ui.send_message('mode_ack', {'mode': mode})


def on_set_vacuum(client_id, data):
    """Browser sends: { power: <int 0..255> }"""
    pwm = max(0, min(255, int(data.get('power', 0))))
    bridge.set_vacuum(pwm)


def on_set_brush(client_id, data):
    """Browser sends: { speed: <int 0..255> }"""
    pwm = max(0, min(255, int(data.get('speed', 0))))
    bridge.set_brush(pwm)


def on_take_snapshot(client_id, data):
    """
    Capture a still JPEG from the camera and send it back to the browser.
    Also saves to /tmp/aria_snapshot_<timestamp>.jpg
    """
    if not CAMERA_AVAILABLE or _camera_cap is None:
        ui.send_message('snapshot_error', {'error': 'Camera not available'})
        return
    try:
        ret, frame = _camera_cap.read()
        if not ret:
            raise RuntimeError("Camera read failed")

        x, y, theta = ekf.pose
        ts   = int(time.time())
        snap_dir = os.path.expanduser('~/.aria/snapshots')
        os.makedirs(snap_dir, exist_ok=True)
        path = os.path.join(snap_dir, f'aria_snapshot_{ts}.jpg')
        cv2.imwrite(path, frame)

        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        b64 = base64.b64encode(buf.tobytes()).decode('utf-8')

        ui.send_message('snapshot_result', {
            'image':     b64,
            'timestamp': ts,
            'position':  {'x': round(x, 1), 'y': round(y, 1)},
            'saved_to':  path,
        })
    except Exception as e:
        ui.send_message('snapshot_error', {'error': str(e)})


def on_start_recording(client_id, data):
    """
    Start recording video + audio using ffmpeg.
    Browser sends: { filename: 'optional_name' }  (default = timestamp)
    """
    global _recording_proc, _recording_file

    if _recording_proc is not None:
        ui.send_message('recording_error', {'error': 'Already recording'})
        return

    os.makedirs(RECORDING_DIR, exist_ok=True)
    name = data.get('filename') or f"aria_{int(time.time())}"
    _recording_file = os.path.join(RECORDING_DIR, f"{name}.mp4")

    cmd = [
        'ffmpeg', '-y',
        '-f', 'v4l2',   '-i', f'/dev/video{CAM_DEVICE}',  # video
        '-f', 'alsa',   '-i', 'hw:0',                      # audio
        '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '28',
        '-c:a', 'aac', '-b:a', '128k',
        '-shortest',
        _recording_file
    ]
    try:
        _recording_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        log.info(f"Recording started → {_recording_file}")
        ui.send_message('recording_started', {'file': _recording_file})
    except FileNotFoundError:
        ui.send_message('recording_error',
                        {'error': 'ffmpeg not found. Install: sudo apt install ffmpeg'})
    except Exception as e:
        ui.send_message('recording_error', {'error': str(e)})


def on_stop_recording(client_id, data):
    """Stop an active ffmpeg recording and notify the browser."""
    global _recording_proc, _recording_file

    if _recording_proc is None:
        ui.send_message('recording_error', {'error': 'Not currently recording'})
        return
    try:
        _recording_proc.terminate()
        _recording_proc.wait(timeout=5)
    except Exception:
        _recording_proc.kill()
    finally:
        saved = _recording_file
        _recording_proc = None
        _recording_file = None
        log.info(f"Recording saved → {saved}")
        ui.send_message('recording_stopped', {'file': saved})


def on_get_map(client_id, data):
    """
    Send the current occupancy grid as a base64 PNG heatmap to the browser.
    Browser sends: {} (no payload needed)
    """
    try:
        import matplotlib
        matplotlib.use('Agg')   # non-interactive backend
        import matplotlib.pyplot as plt
        import numpy as np

        with _state_lock:
            raw = grid._grid.copy()
            rx, ry, _ = ekf.pose

        fig, ax = plt.subplots(figsize=(6, 6), dpi=100)
        ax.imshow(raw, origin='lower', cmap='Blues',
                  vmin=0, vmax=4, interpolation='nearest')

        # Robot dot
        rrow, rcol = (int(ry / CELL_SIZE_CM) + raw.shape[0]//2,
                      int(rx / CELL_SIZE_CM) + raw.shape[1]//2)
        ax.plot(rcol, rrow, 'ro', markersize=6)

        ax.set_title(f"Coverage: {grid.coverage_percent():.1f}%")
        ax.axis('off')
        fig.tight_layout(pad=0)

        buf = io.BytesIO()
        fig.savefig(buf, format='png')
        plt.close(fig)
        buf.seek(0)
        b64 = base64.b64encode(buf.getvalue()).decode('utf-8')
        ui.send_message('map_image', {'image': b64})
    except ImportError:
        ui.send_message('map_error', {'error': 'matplotlib not installed'})
    except Exception as e:
        ui.send_message('map_error', {'error': str(e)})


def on_reset_pose(client_id, data):
    """Reset EKF to origin and clear the grid (fresh start)."""
    with _state_lock:
        ekf.reset(0.0, 0.0, 0.0)
        cleaner.stop()   # safe stop before any position reset
    log.info("EKF pose reset to origin")
    ui.send_message('pose_reset_ack', {'ok': True})


# ────────────────────────────────────────────────────────────────────────────────
# 10b. PHASE 4 WEBUI HANDLERS — Cleaning Control
# ────────────────────────────────────────────────────────────────────────────────

def on_start_clean(client_id, data):
    """
    Begin autonomous cleaning run.
    Browser sends: {} (no payload needed)
    Robot must NOT be in manual mode.
    """
    if _manual_mode:
        ui.send_message('clean_error', {'error': 'Switch to AUTO mode first'})
        return
    with _state_lock:
        x, y, _ = ekf.pose
        cleaner.start(x, y)
    log.info("WebUI: cleaning run started")
    ui.send_message('clean_ack', {'state': cleaner.state_name})


def on_stop_clean(client_id, data):
    """
    Emergency stop — return to IDLE immediately.
    Browser sends: {} (no payload needed)
    """
    with _state_lock:
        cleaner.stop()
        bridge.set_motors(0, 0)
        bridge.set_vacuum(0)
        bridge.set_brush(0)
    log.info("WebUI: cleaning run stopped")
    ui.send_message('clean_ack', {'state': cleaner.state_name})


def on_get_state(client_id, data):
    """
    Immediate state snapshot (not rate-limited like the 2 Hz push).
    Browser sends: {} — useful on first connect to populate the dashboard.
    """
    with _state_lock:
        x, y, theta = ekf.pose
        cov  = grid.coverage_percent()
    ui.send_message('nav_state', {
        'x':        round(x,    1),
        'y':        round(y,    1),
        'theta':    round(theta, 3),
        'coverage': round(cov,  1),
        'state':    cleaner.state_name,
        'manual':   _manual_mode,
    })


# ────────────────────────────────────────────────────────────────────────────────
# 10c. PHASE 5 WEBUI HANDLERS — Dock & A* Control
# ────────────────────────────────────────────────────────────────────────────────

def on_set_dock(client_id, data):
    """
    Save the dock station position.
    Two ways to call:
      1. Browser sends: { x: <float>, y: <float> }  — explicit coordinates
      2. Browser sends: { here: true }               — use current robot position
    The dock position persists in the cleaner object for the session.
    """
    with _state_lock:
        rx, ry, _ = ekf.pose
        if data.get('here'):
            dock_x, dock_y = rx, ry
        else:
            dock_x = float(data.get('x', 0.0))
            dock_y = float(data.get('y', 0.0))
        cleaner.set_dock_position(dock_x, dock_y)
    log.info(f"Dock position set to ({dock_x:.1f}, {dock_y:.1f}) cm")
    ui.send_message('dock_ack', {'x': round(dock_x, 1), 'y': round(dock_y, 1)})


def on_trigger_dock(client_id, data):
    """
    Send robot to dock immediately (regardless of battery level).
    Browser sends: {} — no payload needed.
    Robot must be in AUTO mode and not already IDLE/DOCK.
    """
    if _manual_mode:
        ui.send_message('clean_error', {'error': 'Switch to AUTO mode first'})
        return
    with _state_lock:
        if cleaner.state in (CleanState.IDLE, CleanState.DOCK):
            ui.send_message('clean_error',
                            {'error': f'Already {cleaner.state_name}'})
            return
        cleaner.notify_low_battery()   # triggers DOCK on next step() call
    log.info("WebUI: dock triggered manually")
    ui.send_message('clean_ack', {'state': 'DOCK'})


def on_set_battery(client_id, data):
    """
    Manually override the battery percentage (for testing dock trigger).
    Browser sends: { pct: <float 0–100> }
    In production this will read from a real ADC on the STM32.
    """
    global _battery_pct
    pct = max(0.0, min(100.0, float(data.get('pct', 100.0))))
    _battery_pct = pct
    log.info(f"Battery override: {pct:.0f}%")
    if pct < 15.0:
        with _state_lock:
            cleaner.notify_low_battery()
    ui.send_message('battery_ack', {'pct': pct})


def on_plan_path_debug(client_id, data):
    """
    Debug: run A* from current position to a target and send the path back.
    Browser sends: { x: <float>, y: <float> }
    Response: { path: [(x,y), …] } — useful for visualising A* in the WebUI.
    """
    from aria import plan_path as _plan
    goal_x = float(data.get('x', 0.0))
    goal_y = float(data.get('y', 0.0))
    with _state_lock:
        rx, ry, _ = ekf.pose
        path = _plan(grid, rx, ry, goal_x, goal_y)
    if path is None:
        ui.send_message('path_debug', {'error': 'No path found'})
    else:
        ui.send_message('path_debug', {
            'path':      [{'x': round(x,1), 'y': round(y,1)} for x, y in path],
            'waypoints': len(path),
        })


# ════════════════════════════════════════════════════════════════════════════
# 11. WEBUI SETUP + BACKGROUND THREADS
# ════════════════════════════════════════════════════════════════════════════
ui = WebUI()

# — Original object detection (untouched) —
ui.on_message('detect_objects', on_detect_objects)

# — Navigation & manual control —
ui.on_message('manual_drive',    on_manual_drive)
ui.on_message('set_mode',        on_set_mode)
ui.on_message('set_vacuum',      on_set_vacuum)
ui.on_message('set_brush',       on_set_brush)

# — Camera & recording —
ui.on_message('take_snapshot',   on_take_snapshot)
ui.on_message('start_recording', on_start_recording)
ui.on_message('stop_recording',  on_stop_recording)

# — Map & state —
ui.on_message('get_map',         on_get_map)
ui.on_message('reset_pose',      on_reset_pose)

# — Phase 4: cleaning control —
ui.on_message('start_clean',     on_start_clean)
ui.on_message('stop_clean',      on_stop_clean)
ui.on_message('get_state',       on_get_state)

# — Phase 5: dock & A* —
ui.on_message('set_dock',        on_set_dock)
ui.on_message('trigger_dock',    on_trigger_dock)
ui.on_message('set_battery',     on_set_battery)
ui.on_message('plan_path_debug', on_plan_path_debug)

# — Start navigation loop in background thread —
_nav_thread = threading.Thread(
    target=_navigation_loop,
    name='ARIA-nav',
    daemon=True,
)
_nav_thread.start()
log.info("Navigation thread started")

# — Start camera stream in background thread (if cv2 available) —
if CAMERA_AVAILABLE:
    _cam_thread = threading.Thread(
        target=_camera_stream_loop,
        name='ARIA-cam',
        daemon=True,
    )
    _cam_thread.start()
    log.info("Camera thread started")
else:
    log.info("Camera thread skipped (opencv not installed)")

# ════════════════════════════════════════════════════════════════════════════
# 12. START APP  (blocking — App Lab takes over from here)
# ════════════════════════════════════════════════════════════════════════════
log.info("ARIA starting — WebUI at http://<UNO-Q-IP>:7000")
App.run()