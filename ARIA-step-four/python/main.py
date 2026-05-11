# SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l.
# SPDX-License-Identifier: MPL-2.0
# ARIA-step-four: Merged robot brain + Telegram bot

from arduino.app_bricks.telegram_bot import TelegramBot, Sender, Message
from arduino.app_bricks.object_detection import ObjectDetection
from arduino.app_bricks.mood_detector import MoodDetector
from arduino.app_bricks.web_ui import WebUI
from arduino.app_utils import App
from PIL import Image
from io import BytesIO
import threading, logging, json, os, time, math

import serial_bridge, motor, telemetry, navigator, camera, vacuum

from arduino.app_bricks.video_objectdetection import VideoObjectDetection
from datetime import datetime, timezone

logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

# ── Bricks ───────────────────────────────────────────────────────────────────
bot            = TelegramBot()
obj_detection  = ObjectDetection()
mood           = MoodDetector()
ui             = WebUI()
detection_stream = VideoObjectDetection(confidence=0.5, debounce_sec=0.0)
camera.register_stream(detection_stream)

# ── Robot state ───────────────────────────────────────────────────────────────
state = {
    "motors_on": False,
    "speed": 160,
    "navigating": False,
    "mode": "manual",
    "vacuum": 0,   # 0-255 PWM
}
nav   = navigator.Navigator()
serial_bridge.connect()

AREAS_FILE    = "saved_areas.json"
ROUTINES_FILE = "routines.json"
DOCK_FILE     = "dock.json"

def _load_json(path):
    if not os.path.exists(path): return {}
    try:
        with open(path) as f: return json.load(f)
    except: return {}

def _save_json(path, data):
    try:
        with open(path, "w") as f: json.dump(data, f, indent=2)
    except: pass


# ══════════════════════════════════════════════════════════════════════════════
# TELEGRAM — basic commands
# ══════════════════════════════════════════════════════════════════════════════
def greet(sender: Sender, message: Message):
    sender.reply(f"👋 Hi {sender.first_name}! This is ARIA on Arduino UNO Q!")

def help_cmd(sender: Sender, message: Message):
    sender.reply(
        "🤖 *ARIA Commands*\n━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
        "🎮 /forward /backward /left /right /stop /speed /mode\n"
        "🧹 /clean /stopclean /dock /setdock /cleanzone\n"
        "📍 /goto /areas /savearea /deletearea /cancelpath\n"
        "📊 /status /pose /sensors /battery /coverage\n"
        "📷 /photo /detect /record\n"
        "🗺️ /map /heatmap /resetpose\n"
        "🔧 /vacuum /brush /ping /log\n"
        "🔔 /alerts\n"
        "📝 Text → mood   📷 Photo → object detection"
    )

def ping_cmd(sender: Sender, message: Message):
    sender.reply(f"🏓 *Pong!*\nBot alive. Time: {time.strftime('%H:%M:%S')}")

def sentiment(sender: Sender, message: Message):
    sender.reply(f"Your mood is: {mood.get_sentiment(message.text)}")

def detect_objects(sender: Sender, message: Message, photo: bytes, filename: str, size: int):
    sender.reply("📷 Detecting objects...")
    image = Image.open(BytesIO(photo))
    results = obj_detection.detect(image, confidence=0.1)
    out = BytesIO()
    obj_detection.draw_bounding_boxes(image, results).save(out, format="PNG")
    out.seek(0)
    caption = f"✅ Found {len(results['detection'])} object(s)!" if results else "Nothing detected"
    if not sender.reply_photo(out.getvalue(), caption):
        sender.reply("❌ Failed to send image")

# ══════════════════════════════════════════════════════════════════════════════
# MOTOR CONTROL
# ══════════════════════════════════════════════════════════════════════════════
def forward_cmd(sender: Sender, message: Message):
    args = message.text.strip().split()
    spd = int(args[1]) if len(args) > 1 and args[1].isdigit() else state["speed"]
    spd = max(0, min(255, spd))
    state["motors_on"] = True
    motor.send_motor_cmd(spd, spd)
    sender.reply(f"⬆️ Driving forward at speed {spd}")

def backward_cmd(sender: Sender, message: Message):
    args = message.text.strip().split()
    spd = int(args[1]) if len(args) > 1 and args[1].isdigit() else state["speed"]
    spd = max(0, min(255, spd))
    state["motors_on"] = True
    motor.send_motor_cmd(-spd, -spd)
    sender.reply(f"⬇️ Driving backward at speed {spd}")

def left_cmd(sender: Sender, message: Message):
    spd = state["speed"]
    state["motors_on"] = True
    motor.send_motor_cmd(-spd, spd)
    time.sleep(0.4)
    motor.send_motor_cmd(0, 0)
    sender.reply("↩️ Spun left")

def right_cmd(sender: Sender, message: Message):
    spd = state["speed"]
    state["motors_on"] = True
    motor.send_motor_cmd(spd, -spd)
    time.sleep(0.4)
    motor.send_motor_cmd(0, 0)
    sender.reply("↪️ Spun right")

def stop_cmd(sender: Sender, message: Message):
    state["motors_on"] = False
    state["navigating"] = False
    nav.clear_goal()
    motor.send_motor_cmd(0, 0)
    ui.send_message("state_update", state)
    sender.reply("🛑 Emergency stop! Motors off.")

def speed_cmd(sender: Sender, message: Message):
    args = message.text.strip().split()
    if len(args) < 2 or not args[1].isdigit():
        sender.reply("Usage: /speed <0-255>"); return
    state["speed"] = max(0, min(255, int(args[1])))
    sender.reply(f"🔢 Speed set to {state['speed']}")

def mode_cmd(sender: Sender, message: Message):
    args = message.text.strip().split()
    if len(args) < 2 or args[1] not in ("auto", "manual"):
        sender.reply("Usage: /mode <auto|manual>"); return
    state["mode"] = args[1]
    sender.reply(f"🔄 Mode set to *{state['mode']}*")


# ══════════════════════════════════════════════════════════════════════════════
# NAVIGATION
# ══════════════════════════════════════════════════════════════════════════════
def goto_cmd(sender: Sender, message: Message):
    args = message.text.strip().split(maxsplit=1)
    areas = _load_json(AREAS_FILE)
    if len(args) < 2:
        listing = "\n".join(f"  • {n} ({c['x']},{c['y']}cm)" for n,c in areas.items()) if areas else "  None saved yet."
        sender.reply(f"📍 Usage: /goto <name>\n\n{listing}"); return
    name = args[1].strip().lower()
    if name not in areas:
        sender.reply(f"❌ Area '{name}' not found. Use /areas."); return
    c = areas[name]
    nav.set_goal(c["x"], c["y"], state["speed"])
    state["navigating"] = True; state["motors_on"] = True
    ui.send_message("state_update", state)
    sender.reply(f"🚗 Navigating to *{name}* ({c['x']}, {c['y']} cm)")

def areas_cmd(sender: Sender, message: Message):
    areas = _load_json(AREAS_FILE)
    if not areas: sender.reply("📍 No saved areas. Use /savearea <name>."); return
    lines = ["📍 *Saved Areas:*\n"] + [f"  • *{n}* — ({c['x']}, {c['y']}) cm" for n,c in areas.items()]
    sender.reply("\n".join(lines))

def savearea_cmd(sender: Sender, message: Message):
    args = message.text.strip().split(maxsplit=1)
    if len(args) < 2: sender.reply("Usage: /savearea <name>"); return
    name = args[1].strip().lower()
    pose = telemetry.get_pose()
    areas = _load_json(AREAS_FILE)
    areas[name] = {"x": pose["x_cm"], "y": pose["y_cm"]}
    _save_json(AREAS_FILE, areas)
    sender.reply(f"✅ Area *{name}* saved at ({pose['x_cm']}, {pose['y_cm']}) cm")

def deletearea_cmd(sender: Sender, message: Message):
    args = message.text.strip().split(maxsplit=1)
    if len(args) < 2: sender.reply("Usage: /deletearea <name>"); return
    name = args[1].strip().lower()
    areas = _load_json(AREAS_FILE)
    if name not in areas: sender.reply(f"❌ Area '{name}' not found."); return
    del areas[name]; _save_json(AREAS_FILE, areas)
    sender.reply(f"🗑️ Area *{name}* deleted.")

def cancelpath_cmd(sender: Sender, message: Message):
    nav.clear_goal(); state["navigating"] = False
    motor.send_motor_cmd(0, 0); ui.send_message("state_update", state)
    sender.reply("🚫 Navigation cancelled.")

# ══════════════════════════════════════════════════════════════════════════════
# CLEANING
# ══════════════════════════════════════════════════════════════════════════════
def _run_clean_zone(x_min, y_min, x_max, y_max):
    path = []
    path += [(x_min,y_min),(x_max,y_min),(x_max,y_max),(x_min,y_max),(x_min,y_min)]
    lane = 20.0; y = y_min + lane/2; right = True
    while y <= y_max:
        path += [(x_min,y),(x_max,y)] if right else [(x_max,y),(x_min,y)]
        y += lane; right = not right
    nav.set_path(path, state["speed"])
    state["navigating"] = True; state["motors_on"] = True
    ui.send_message("state_update", state)
    ui.send_message("path_update", [{"x":p[0],"y":p[1]} for p in path])

def clean_cmd(sender: Sender, message: Message):
    _run_clean_zone(-150, -150, 150, 150)
    sender.reply("🧹 Full-room clean started!")

def stopclean_cmd(sender: Sender, message: Message):
    nav.clear_goal(); state["navigating"] = False; motor.send_motor_cmd(0, 0)
    ui.send_message("state_update", state); sender.reply("🛑 Cleaning stopped.")

def dock_cmd(sender: Sender, message: Message):
    dock = _load_json(DOCK_FILE)
    if not dock: sender.reply("❌ No dock saved. Use /setdock first."); return
    nav.set_goal(dock["x"], dock["y"], state["speed"])
    state["navigating"] = True; state["motors_on"] = True
    ui.send_message("state_update", state)
    sender.reply(f"🔌 Returning to dock ({dock['x']}, {dock['y']}) cm")

def setdock_cmd(sender: Sender, message: Message):
    pose = telemetry.get_pose()
    _save_json(DOCK_FILE, {"x": pose["x_cm"], "y": pose["y_cm"]})
    sender.reply(f"🔌 Dock saved at ({pose['x_cm']}, {pose['y_cm']}) cm")

def cleanzone_cmd(sender: Sender, message: Message):
    args = message.text.strip().split()
    if len(args) < 5: sender.reply("Usage: /cleanzone <x_min> <y_min> <x_max> <y_max>"); return
    try:
        x1,y1,x2,y2 = float(args[1]),float(args[2]),float(args[3]),float(args[4])
        _run_clean_zone(min(x1,x2),min(y1,y2),max(x1,x2),max(y1,y2))
        sender.reply(f"🧹 Zone clean started: ({x1},{y1}) → ({x2},{y2})")
    except ValueError: sender.reply("❌ Invalid numbers. Usage: /cleanzone x_min y_min x_max y_max")


# ══════════════════════════════════════════════════════════════════════════════
# TELEMETRY & STATUS
# ══════════════════════════════════════════════════════════════════════════════
def pose_cmd(sender: Sender, message: Message):
    p = telemetry.get_pose()
    deg = round(math.degrees(p["theta_rad"]), 1)
    dist = round(math.hypot(p["x_cm"], p["y_cm"]), 1)
    sender.reply(f"📐 *EKF Pose*\nX: {p['x_cm']} cm\nY: {p['y_cm']} cm\nθ: {deg}°\nDist from origin: {dist} cm")

def sensors_cmd(sender: Sender, message: Message):
    t = telemetry.telemetry
    sender.reply(
        f"📡 *Raw Sensors*\n"
        f"Enc L: {t['enc_l']}  Enc R: {t['enc_r']}\n"
        f"Accel X:{t['accel_x']:.2f} Y:{t['accel_y']:.2f} Z:{t['accel_z']:.2f}\n"
        f"Gyro  X:{t['gyro_x']:.2f} Y:{t['gyro_y']:.2f} Z:{t['gyro_z']:.2f}"
    )

def coverage_cmd(sender: Sender, message: Message):
    snap = telemetry.get_grid_snapshot()
    sender.reply(f"🗺️ Grid coverage: *{snap.get('coverage', 0)}%*")

def status_cmd(sender: Sender, message: Message):
    p = telemetry.get_pose()
    snap = telemetry.get_grid_snapshot()
    deg = round(math.degrees(p["theta_rad"]), 1)
    sender.reply(
        f"📊 *ARIA Status*\n"
        f"Mode: {state['mode']} | Motors: {'ON' if state['motors_on'] else 'OFF'}\n"
        f"Speed: {state['speed']} | Nav: {'active' if state['navigating'] else 'idle'}\n"
        f"Pose: ({p['x_cm']}, {p['y_cm']}) cm  θ={deg}°\n"
        f"Coverage: {snap.get('coverage', 0)}%"
    )

def battery_cmd(sender: Sender, message: Message):
    sender.reply("🔋 Battery monitoring not yet wired to hardware.")

def map_cmd(sender: Sender, message: Message):
    try:
        import io as _io
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
        snap = telemetry.get_grid_snapshot()
        data = np.array(snap["data"])
        fig, ax = plt.subplots(figsize=(6,6))
        cmap = matplotlib.colors.ListedColormap(["#4CAF50","#111111","#EEEEEE"])
        bounds = [-1,64,126,128]; norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)
        ax.imshow(data, cmap=cmap, norm=norm, origin="upper")
        p = telemetry.get_pose()
        origin = snap["origin_row"]
        cell = snap["cell_cm"]
        rx = origin + p["x_cm"]/cell; ry = origin - p["y_cm"]/cell
        ax.plot(rx, ry, "bo", markersize=8)
        ax.set_title(f"ARIA Map — {snap.get('coverage',0)}% covered")
        buf = _io.BytesIO(); fig.savefig(buf, format="PNG", bbox_inches="tight"); plt.close(fig)
        buf.seek(0)
        if not sender.reply_photo(buf.getvalue(), f"🗺️ {snap.get('coverage',0)}% explored"):
            sender.reply("❌ Could not send map image.")
    except Exception as e:
        sender.reply(f"❌ Map render error: {e}")

def heatmap_cmd(sender: Sender, message: Message):
    sender.reply("🚧 Dirt heatmap not yet implemented.")

def resetpose_cmd(sender: Sender, message: Message):
    telemetry._dr.__init__()
    if telemetry.ekf: telemetry.ekf.__init__(0.0, 0.0, 0.0)
    sender.reply("🔄 Pose reset to origin (0, 0, 0°)")

# ─────────────────────────────────────────────────────────────────────────────
# BROWSER-SIDE FRAME RELAY
# The browser already displays the live camera via iframe. We ask it to capture
# a Canvas frame and send it back via Socket.IO. This bypasses all HTTP/MJPEG
# endpoint guessing.
# ─────────────────────────────────────────────────────────────────────────────
_frame_store = {"data": None, "event": None}

def _request_frame_from_browser(timeout=10):
    """Ask connected browser to fetch one JPEG frame from :4912 and return it via Socket.IO.
    Returns raw bytes or None."""
    import base64 as _b64
    evt = threading.Event()
    _frame_store["data"] = None
    _frame_store["event"] = evt
    ui.send_message("request_frame", {})  # frontend must reply with frame_from_browser
    try:
        if not evt.wait(timeout=timeout):
            return None
        raw_b64 = _frame_store.get("data")
        if not raw_b64:
            return None
        try:
            out = _b64.b64decode(raw_b64)
        except Exception:
            return None
        try:
            camera._latest_frame_jpeg = out
        except Exception:
            pass
        return out
    finally:
        _frame_store["event"] = None

def _get_frame(timeout=10):
    """Get a JPEG frame: try camera module first, then ask browser.
    Returns raw bytes or None."""
    # Fast path: camera module already has a frame
    raw = camera.get_snapshot_jpeg()
    if raw:
        return raw
    # Slow path: ask the open browser to relay one
    raw = _request_frame_from_browser(timeout=timeout)
    if raw:
        # Cache it in camera module for next time
        camera._latest_frame_jpeg = raw
    return raw

def photo_cmd(sender: Sender, message: Message):
    # Telegram requires valid UTF-8; use BMP or single non-BMP escapes (\U), never UTF-16 surrogate pairs (\ud800-\udfff).
    sender.reply("\U0001f4f7 Capturing snapshot...")
    raw = _get_frame(timeout=10)
    if raw is None:
        dets = camera.get_latest_detections()
        if dets:
            lines = ["No video frame yet. Last detections from the Brick:"]
            for label, entries in dets.items():
                if isinstance(entries, list):
                    for e in entries:
                        conf = round(e.get("confidence", 0) * 100, 1) if isinstance(e, dict) else round(float(e) * 100, 1)
                        lines.append(f"  - {label} ({conf}%)")
                else:
                    lines.append(f"  - {label} ({round(float(entries)*100,1)}%)")
            sender.reply("\n".join(lines))
        else:
            sender.reply(
                "Camera not ready: no MJPEG from :4912 and no USB grab. "
                "Open http://YOUR_IP:4912/embed in a browser on the LAN; "
                "use http (not https) for the dashboard; ensure Network Mode + powered hub."
            )
        return
    if not sender.reply_photo(raw, "\U0001f4f8 ARIA live snapshot"):
        sender.reply("Snapshot captured but could not be sent via Telegram.")

def detect_cmd(sender: Sender, message: Message):
    detections = camera.get_latest_detections()
    lines = ["*ARIA Detection Report:*"]
    if detections:
        for label, entries in detections.items():
            if isinstance(entries, list):
                for e in entries:
                    conf = round(e.get("confidence",0)*100,1) if isinstance(e,dict) else round(float(e)*100,1)
                    lines.append(f"  - {label} ({conf}%)")
            else:
                lines.append(f"  - {label} ({round(float(entries)*100,1)}%)")
    else:
        lines.append("  Nothing detected yet.")
    caption = "\n".join(lines)
    raw = _get_frame(timeout=8)
    if raw:
        if not sender.reply_photo(raw, caption):
            sender.reply(caption)
    else:
        sender.reply(caption)

def record_cmd(sender: Sender, message: Message):
    args = message.text.strip().split()
    secs = int(args[1]) if len(args) > 1 and args[1].isdigit() else 5
    secs = max(1, min(15, secs))
    sender.reply(f"Recording {secs}s GIF...")
    def _do_record():
        import io as _io
        from PIL import Image as _Img
        frames = []
        interval = 0.25  # 4 fps
        total = secs * 4
        for _ in range(total):
            raw = _get_frame(timeout=3)
            if raw:
                try:
                    frames.append(_Img.open(_io.BytesIO(raw)).convert("P", palette=_Img.ADAPTIVE))
                except: pass
            time.sleep(interval)
        if not frames:
            sender.reply("No frames captured. Try :4912 embed in browser, http dashboard, or check USB webcam.")
            return
        buf = _io.BytesIO()
        frames[0].save(buf, format="GIF", save_all=True,
                       append_images=frames[1:], duration=250, loop=0)
        buf.seek(0)
        if not sender.reply_photo(buf.getvalue(), f"{secs}s clip ({len(frames)} frames)"):
            sender.reply("GIF too large for Telegram. Try /record 3.")
    threading.Thread(target=_do_record, daemon=True).start()

def vacuum_cmd(sender: Sender, message: Message):
    args = message.text.strip().split()
    if len(args) < 2:
        sender.reply(
            "🌀 *Vacuum control*\n"
            "Usage:\n"
            "  /vacuum <0-255>\n"
            "  /vacuum on\n"
            "  /vacuum off\n"
            f"Current: {state.get('vacuum', 0)}"
        )
        return

    v = args[1].strip().lower()
    if v in ("on", "start", "1", "true"):
        pwm = 255
    elif v in ("off", "stop", "0", "false"):
        pwm = 0
    elif v.isdigit():
        pwm = int(v)
    else:
        sender.reply("❌ Invalid. Use `/vacuum on`, `/vacuum off`, or `/vacuum <0-255>`.")
        return

    pwm = vacuum.set_vacuum(pwm)
    state["vacuum"] = pwm
    ui.send_message("state_update", state)
    sender.reply(f"🌀 Vacuum PWM set to *{pwm}*")

def brush_cmd(sender: Sender, message: Message):
    sender.reply("🪥 Brush hardware not yet wired.")

def log_cmd(sender: Sender, message: Message):
    sender.reply("📋 Log streaming not yet implemented.")

def alerts_cmd(sender: Sender, message: Message):
    sender.reply("🔔 Alert system not yet implemented.")


# ══════════════════════════════════════════════════════════════════════════════
# WEB UI HANDLERS (from step-two)
# ══════════════════════════════════════════════════════════════════════════════
def _load_routines():
    if not os.path.exists(ROUTINES_FILE): return []
    try:
        with open(ROUTINES_FILE) as f: return json.load(f)
    except: return []

def toggle_power(client, data):
    state["motors_on"] = not state["motors_on"]
    if not state["motors_on"]:
        state["navigating"] = False; nav.clear_goal(); motor.send_motor_cmd(0, 0)
    else:
        if not state["navigating"]: motor.send_motor_cmd(state["speed"], state["speed"])
    ui.send_message("state_update", state)

def set_speed(client, data):
    state["speed"] = data.get("speed", 160)
    ui.send_message("state_update", state)
    if state["motors_on"]: motor.send_motor_cmd(state["speed"], state["speed"])

def on_get_initial_state(client, data):
    ui.send_message("state_update", state, client)
    ui.send_message("routines_list", _load_routines(), client)

def set_goal(client, data):
    x = data.get("x", 0.0); y = data.get("y", 0.0)
    nav.set_goal(x, y, state["speed"])
    state["navigating"] = True; state["motors_on"] = True
    ui.send_message("state_update", state)

def set_path(client, data):
    points = data.get("path", [])
    if not points: return
    path = [(p["x"], p["y"]) for p in points]
    nav.set_path(path, state["speed"])
    state["navigating"] = True; state["motors_on"] = True
    ui.send_message("state_update", state)
    ui.send_message("path_update", [{"x":p[0],"y":p[1]} for p in path])

def clean_zone_ui(client, data):
    zone = data.get("zone")
    if not zone: return
    x1,x2 = min(zone["x_min"],zone["x_max"]),max(zone["x_min"],zone["x_max"])
    y1,y2 = min(zone["y_min"],zone["y_max"]),max(zone["y_min"],zone["y_max"])
    _run_clean_zone(x1, y1, x2, y2)

def clear_goal(client, data):
    nav.clear_goal(); state["navigating"] = False; state["motors_on"] = False
    motor.send_motor_cmd(0, 0); ui.send_message("state_update", state)
    ui.send_message("path_update", [])


def set_vacuum_ui(client, data):
    pwm = data.get("pwm", 0)
    pwm = vacuum.set_vacuum(pwm)
    state["vacuum"] = pwm
    ui.send_message("state_update", state)

def save_routine(client, data):
    name = data.get("name")
    if not name: return
    routines = _load_routines()
    routines.append({"name": name, "type": data.get("type"), "data": data.get("data")})
    try:
        with open(ROUTINES_FILE, "w") as f: json.dump(routines, f, indent=2)
        ui.send_message("routines_list", routines)
    except Exception as e: log.error(f"Save routine error: {e}")


def manual_drive_ui(client, data):
    """Manual drive commands from the website (forward/back/left/right/stop)."""
    payload = data or {}
    action = (payload.get("action") or "").strip().lower()
    try:
        spd = int(payload.get("speed", state.get("speed", 160)))
    except Exception:
        spd = int(state.get("speed", 160))
    spd = max(0, min(255, spd))

    if action == "stop":
        state["motors_on"] = False
        state["navigating"] = False
        nav.clear_goal()
        motor.send_motor_cmd(0, 0)
    elif action == "forward":
        state["motors_on"] = True
        motor.send_motor_cmd(spd, spd)
    elif action == "backward":
        state["motors_on"] = True
        motor.send_motor_cmd(-spd, -spd)
    elif action == "left":
        state["motors_on"] = True
        motor.send_motor_cmd(-spd, spd)
    elif action == "right":
        state["motors_on"] = True
        motor.send_motor_cmd(spd, -spd)
    else:
        return

    ui.send_message("state_update", state)

# ══════════════════════════════════════════════════════════════════════════════
# NAVIGATION LOOP (background thread)
# ══════════════════════════════════════════════════════════════════════════════
def navigation_loop():
    last_count = -1
    while True:
        if state["navigating"] and state["motors_on"]:
            pose = telemetry.get_pose()
            if pose:
                l, r, arrived = nav.step(pose["x_cm"], pose["y_cm"], pose["theta_rad"])
                motor.send_motor_cmd(l, r)
                count = len(nav.waypoints)
                if count != last_count:
                    path = ([{"x": nav.goal[0], "y": nav.goal[1]}] if nav.goal else [])
                    path += [{"x":p[0],"y":p[1]} for p in nav.waypoints]
                    ui.send_message("path_update", path); last_count = count
                if arrived:
                    state["navigating"] = False; state["motors_on"] = False
                    motor.send_motor_cmd(0, 0); ui.send_message("state_update", state)
                    ui.send_message("path_update", [])
        time.sleep(0.05)

# ══════════════════════════════════════════════════════════════════════════════
# REGISTER TELEGRAM COMMANDS
# ══════════════════════════════════════════════════════════════════════════════
bot.add_command("hello",      greet,         "Get a greeting")
bot.add_command("help",       help_cmd,      "Show all commands")
bot.add_command("ping",       ping_cmd,      "Check bot is alive")
bot.add_command("forward",    forward_cmd,   "Drive forward")
bot.add_command("backward",   backward_cmd,  "Drive backward")
bot.add_command("left",       left_cmd,      "Spin left")
bot.add_command("right",      right_cmd,     "Spin right")
bot.add_command("stop",       stop_cmd,      "Emergency stop")
bot.add_command("speed",      speed_cmd,     "Set motor speed 0-255")
bot.add_command("mode",       mode_cmd,      "Switch auto/manual mode")
bot.add_command("goto",       goto_cmd,      "Navigate to a named area")
bot.add_command("areas",      areas_cmd,     "List saved areas")
bot.add_command("savearea",   savearea_cmd,  "Save current position as area")
bot.add_command("deletearea", deletearea_cmd,"Delete a saved area")
bot.add_command("cancelpath", cancelpath_cmd,"Cancel navigation")
bot.add_command("clean",      clean_cmd,     "Start full-room clean")
bot.add_command("stopclean",  stopclean_cmd, "Abort cleaning")
bot.add_command("dock",       dock_cmd,      "Return to dock")
bot.add_command("setdock",    setdock_cmd,   "Save dock position")
bot.add_command("cleanzone",  cleanzone_cmd, "Clean a rectangular zone")
bot.add_command("status",     status_cmd,    "Full status dashboard")
bot.add_command("pose",       pose_cmd,      "Current EKF position")
bot.add_command("sensors",    sensors_cmd,   "Raw IMU + encoder data")
bot.add_command("battery",    battery_cmd,   "Battery percentage")
bot.add_command("coverage",   coverage_cmd,  "Grid coverage percent")
bot.add_command("photo",      photo_cmd,     "Take a snapshot")
bot.add_command("detect",     detect_cmd,    "Snapshot + object detection")
bot.add_command("record",     record_cmd,    "Record video clip")
bot.add_command("map",        map_cmd,       "Send occupancy grid image")
bot.add_command("heatmap",    heatmap_cmd,   "Send dirt heatmap")
bot.add_command("resetpose",  resetpose_cmd, "Reset EKF to origin")
bot.add_command("vacuum",     vacuum_cmd,    "Set vacuum power")
bot.add_command("brush",      brush_cmd,     "Set brush speed")
bot.add_command("log",        log_cmd,       "Show recent log lines")
bot.add_command("alerts",     alerts_cmd,    "Toggle push alerts")
bot.on_text(sentiment)
bot.on_photo(detect_objects)

# ══════════════════════════════════════════════════════════════════════════════
# CAMERA WEB UI HANDLERS
# ══════════════════════════════════════════════════════════════════════════════
def ui_take_snapshot(client, data):
    raw = _get_frame(timeout=12)
    if raw is None:
        ui.send_message(
            "snapshot_error",
            {"error": "Camera stream not ready. Open the Camera tab, check USB webcam + powered hub (Network Mode), or wait ~10s for the stream probe."},
            client,
        )
        return
    import base64 as _b64
    b64 = _b64.b64encode(raw).decode()
    ui.send_message("snapshot_result", {"image": b64}, client)

def ui_camera_detect(client, data):
    # Upload + detect only (live stream detect is done by the brick)
    img_b64 = data.get("image")
    confidence = data.get("confidence", 0.5)
    import base64 as _b64, io as _io
    try:
        if not img_b64:
            ui.send_message("detection_error", {"error": "No image provided — use Upload+Detect"}, client); return
        raw_bytes = _b64.b64decode(img_b64)
        pil_img = Image.open(_io.BytesIO(raw_bytes))
        results = obj_detection.detect(pil_img, confidence=confidence)
        annotated = obj_detection.draw_bounding_boxes(pil_img, results)
        buf = _io.BytesIO(); annotated.save(buf, format="PNG"); buf.seek(0)
        b64_result = _b64.b64encode(buf.getvalue()).decode()
        count = len(results.get("detection", [])) if results else 0
        ui.send_message("detection_result", {"success": True, "result_image": b64_result, "detection_count": count}, client)
    except Exception as e:
        ui.send_message("detection_error", {"error": str(e)}, client)

# Live detection callback from VideoObjectDetection brick
def on_live_detection(detections: dict):
    camera.on_detections(detections)
    for label, entries in detections.items():
        for e in entries:
            conf = e.get("confidence", 0) if isinstance(e, dict) else float(e)
            entry = {
                "content": label,
                "confidence": conf,
                "timestamp": datetime.now(timezone.utc).isoformat()
            }
            ui.send_message("detection", entry)

detection_stream.on_detect_all(on_live_detection)

def ui_record(client, data):
    """Record a short GIF — server-side MJPEG grab when possible, browser relay as fallback."""
    duration = int(data.get("duration", 5))
    duration = max(1, min(15, duration))
    def _run():
        import io as _io, base64 as _b64
        from PIL import Image as _Img
        frames = []
        total = duration * 4   # 4 fps
        for _ in range(total):
            raw = _get_frame(timeout=4)
            if raw:
                try:
                    img = _Img.open(_io.BytesIO(raw)).convert("P", palette=_Img.ADAPTIVE)
                    frames.append(img)
                except: pass
            time.sleep(0.25)
        if not frames:
            ui.send_message("record_error", {"error": "No frames captured — keep Camera tab open"}, client)
            return
        buf = _io.BytesIO()
        frames[0].save(buf, format="GIF", save_all=True,
                       append_images=frames[1:], duration=250, loop=0)
        buf.seek(0)
        ui.send_message("record_result", {"gif": _b64.b64encode(buf.getvalue()).decode(), "frames": len(frames)}, client)
    threading.Thread(target=_run, daemon=True).start()

def frame_from_browser(client, data):
    """Receives a JPEG base64 from the browser (fetch from :4912) for Telegram / relay."""
    evt = _frame_store.get("event")
    if evt is None:
        return
    _frame_store["data"] = (data or {}).get("image")
    evt.set()


# ══════════════════════════════════════════════════════════════════════════════
# REGISTER WEB UI HANDLERS
# ══════════════════════════════════════════════════════════════════════════════
ui.on_message("toggle_power",      toggle_power)
ui.on_message("set_speed",         set_speed)
ui.on_message("get_initial_state", on_get_initial_state)
ui.on_message("set_goal",          set_goal)
ui.on_message("set_path",          set_path)
ui.on_message("clean_zone",        clean_zone_ui)
ui.on_message("clear_goal",        clear_goal)
ui.on_message("save_routine",      save_routine)
ui.on_message("set_vacuum",        set_vacuum_ui)
ui.on_message("manual_drive",      manual_drive_ui)
ui.on_message("take_snapshot",     ui_take_snapshot)
ui.on_message("camera_detect",     ui_camera_detect)
ui.on_message("camera_record",     ui_record)
ui.on_message("frame_from_browser", frame_from_browser)

_diag_last_summary = None


def diag_result(client, data):
    import logging, re
    global _diag_last_summary
    log = logging.getLogger("camera_diag")
    blob = str(data) if data is not None else ""
    if any(
        k in blob.lower()
        for k in ("metamask", "chainchanged", "metamask-provider", "walletconnect", "ethereum")
    ):
        return
    if isinstance(data, dict) and data.get("source") == "probe_display":
        sm = data.get("summary")
        if sm and sm == _diag_last_summary:
            return
        _diag_last_summary = sm
    log.info(f"BROWSER_DIAG: {blob[:500]}")
    if not isinstance(data, dict):
        return
    html = data.get("html", "") or ""
    if html:
        ws_urls    = re.findall(r'ws[s]?://[^\'">\s]+', html)
        fetch_urls = re.findall(r"fetch\(['\"]([^'\"]+)['\"]", html)
        log.info(f"EMBED_WS_URLS: {ws_urls}")
        log.info(f"EMBED_FETCH_URLS: {fetch_urls}")
        ui.send_message("diag_result_ack",
            {"ws_urls": ws_urls, "fetch_urls": fetch_urls,
             "html_preview": html[:500]}, client)
    else:
        ui.send_message("diag_result_ack", data, client)

ui.on_message("diag_result",       diag_result)

ui.on_message("override_th",       lambda sid, v: detection_stream.override_threshold(float(v)))


# ══════════════════════════════════════════════════════════════════════════════
# START BACKGROUND THREADS & RUN
# ══════════════════════════════════════════════════════════════════════════════
_probe_results = []

def _probe_camera_stream():
    """Probe VideoObjectDetection HTTP: try common paths after Brick warm-up.
    Must run after embed binds (same race as camera.start_frame_grabber)."""
    import urllib.request, logging, re
    import camera as cam
    global _probe_results
    log = logging.getLogger("camera_probe")
    wait = cam.HTTP_WARMUP_SEC + 5.0
    time.sleep(wait)
    port = cam.PORT
    hosts = cam.video_http_hosts()
    paths = ["/stream", "/embed", "/", "/snapshot", "/video", "/mjpeg", "/frame", "/cam"]
    log.info("camera_probe hosts: %s", ", ".join(hosts))
    _probe_results = []
    for host in hosts:
        for path in paths:
            url = f"http://{host}:{port}{path}"
            try:
                resp = urllib.request.urlopen(url, timeout=5)
                ct = resp.headers.get("Content-Type", "?")
                first = resp.read(4096)
                has_jpeg = b"\xff\xd8" in first
                ws_urls = re.findall(rb'ws[s]?://[^\'">\s]+', first)
                fetch_urls = re.findall(rb"fetch\(['\"]([^'\"]+)['\"]", first)
                img_srcs = re.findall(rb'src=["\']([^"\']{4,})["\']', first)
                result = {
                    "url": url, "ct": ct, "jpeg": has_jpeg,
                    "ws": [u.decode(errors="replace") for u in ws_urls],
                    "fetch": [u.decode(errors="replace") for u in fetch_urls],
                    "srcs": [u.decode(errors="replace") for u in img_srcs[:5]],
                    "preview": first[:300].decode(errors="replace"),
                }
                _probe_results.append(result)
                log.info(f"PROBE {url}: ct={ct} jpeg={has_jpeg} ws={ws_urls} srcs={img_srcs[:3]}")
                resp.close()
                if has_jpeg and b"<html" not in first.lower():
                    log.info(f"PROBE: Raw JPEG at {url}! Storing frame.")
                    soi = first.index(b"\xff\xd8")
                    eoi = first.rfind(b"\xff\xd9")
                    if eoi > soi:
                        camera._latest_frame_jpeg = first[soi:eoi + 2]
            except Exception as e:
                log.info(f"PROBE {url}: {e}")
                _probe_results.append({"url": url, "error": str(e)})
    try:
        ui.send_message("probe_results", {"results": _probe_results.copy()})
    except Exception:
        pass
    log.info("PROBE done.")

def _on_client_connect(sid):
    """When a browser connects, send any existing probe results."""
    if _probe_results:
        ui.send_message("probe_results", {"results": _probe_results})

ui.on_connect(_on_client_connect)

threading.Thread(target=telemetry.telemetry_loop, args=(ui,), daemon=True).start()
threading.Thread(target=navigation_loop, daemon=True).start()
camera.start_frame_grabber()  # auto-discovers MJPEG/WebSocket at port 4912
threading.Thread(target=_probe_camera_stream, daemon=True).start()
App.run()


