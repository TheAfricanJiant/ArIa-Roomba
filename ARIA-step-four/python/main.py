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

import serial_bridge, motor, telemetry, navigator, camera

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
state = {"motors_on": False, "speed": 160, "navigating": False, "mode": "manual"}
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
    """Ask connected browser to capture and return a JPEG frame.
    Returns raw bytes or None."""
    import base64 as _b64
    evt = threading.Event()
    _frame_store["data"] = None
    _frame_store["event"] = evt
    ui.send_message("request_frame", {})  # broadcast to all connected browsers
    if evt.wait(timeout=timeout) and _frame_store["data"]:
        try:
            return _b64.b64decode(_frame_store["data"])
        except Exception:
            return None
    return None

def photo_cmd(sender: Sender, message: Message):
    sender.reply("📷 Requesting snapshot… (keep Camera tab open in the web UI)")
    raw = _request_frame_from_browser(timeout=10)
    if raw is None:
        dets = camera.get_latest_detections()
        if dets:
            lines = ["📷 No browser frame available. Last detections:"]
            for label, entries in dets.items():
                if isinstance(entries, list):
                    for e in entries:
                        conf = round(e.get("confidence", 0) * 100, 1) if isinstance(e, dict) else round(float(e) * 100, 1)
                        lines.append(f"  • {label} ({conf}%)")
                else:
                    lines.append(f"  • {label} ({round(float(entries)*100,1)}%)")
            sender.reply("\n".join(lines))
        else:
            sender.reply("❌ No browser connected. Open the web UI and click the Camera tab, then retry.")
        return
    if not sender.reply_photo(raw, "📸 ARIA live snapshot"):
        sender.reply("❌ Snapshot captured but could not be sent.")

def detect_cmd(sender: Sender, message: Message):
    detections = camera.get_latest_detections()
    lines = ["🔍 *ARIA Detection Report:*"]
    if detections:
        for label, entries in detections.items():
            for e in entries:
                conf = round(e.get("confidence",0)*100,1) if isinstance(e,dict) else round(float(e)*100,1)
                lines.append(f"  • {label} ({conf}%)")
    else:
        lines.append("  Nothing detected yet.")
    caption = "\n".join(lines)
    raw = _request_frame_from_browser(timeout=8)
    if raw:
        if not sender.reply_photo(raw, caption):
            sender.reply(caption)
    else:
        sender.reply(caption + "\n\n_Snapshot: open web UI Camera tab for live frame._")

def record_cmd(sender: Sender, message: Message):
    args = message.text.strip().split()
    secs = int(args[1]) if len(args) > 1 and args[1].isdigit() else 5
    secs = max(1, min(15, secs))
    sender.reply(f"🎥 Recording {secs}s… keep the web UI Camera tab open.")
    def _do_record():
        import io as _io, base64 as _b64
        from PIL import Image as _Img
        frames = []
        interval = 0.25  # 4 fps
        total = secs * 4
        for _ in range(total):
            raw = _request_frame_from_browser(timeout=3)
            if raw:
                try:
                    img = _Img.open(_io.BytesIO(raw)).convert("P", palette=_Img.ADAPTIVE)
                    frames.append(img)
                except: pass
            time.sleep(interval)
        if not frames:
            sender.reply("❌ No frames captured. Ensure web UI Camera tab is open.")
            return
        buf = _io.BytesIO()
        frames[0].save(buf, format="GIF", save_all=True,
                       append_images=frames[1:], duration=250, loop=0)
        buf.seek(0)
        if not sender.reply_photo(buf.getvalue(), f"🎥 {secs}s clip ({len(frames)} frames)"):
            sender.reply("❌ GIF captured but too large. Try /record 3.")
    threading.Thread(target=_do_record, daemon=True).start()

def vacuum_cmd(sender: Sender, message: Message):
    sender.reply("🌀 Vacuum hardware not yet wired.")

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

def save_routine(client, data):
    name = data.get("name")
    if not name: return
    routines = _load_routines()
    routines.append({"name": name, "type": data.get("type"), "data": data.get("data")})
    try:
        with open(ROUTINES_FILE, "w") as f: json.dump(routines, f, indent=2)
        ui.send_message("routines_list", routines)
    except Exception as e: log.error(f"Save routine error: {e}")

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
    raw = camera.get_snapshot_jpeg()
    if raw is None:
        ui.send_message("snapshot_error", {"error": "Camera stream not available"}, client); return
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
    """Web UI Record button — uses browser-side frame relay."""
    duration = int(data.get("duration", 5))
    duration = max(1, min(15, duration))
    def _run():
        import io as _io, base64 as _b64
        from PIL import Image as _Img
        frames = []
        total = duration * 4   # 4 fps
        for _ in range(total):
            raw = _request_frame_from_browser(timeout=3)
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
    """Receives a Canvas-captured JPEG base64 from the browser for Telegram commands."""
    if _frame_store.get("event") is not None:
        _frame_store["data"] = data.get("image")
        # Also store in camera module for future calls
        if _frame_store["data"]:
            import base64 as _b64
            try:
                camera._latest_frame_jpeg = _b64.b64decode(_frame_store["data"])
            except Exception:
                pass
        _frame_store["event"].set()


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
ui.on_message("take_snapshot",     ui_take_snapshot)
ui.on_message("camera_detect",     ui_camera_detect)
ui.on_message("camera_record",     ui_record)
ui.on_message("frame_from_browser", frame_from_browser)

def diag_result(client, data):
    import logging, re
    log = logging.getLogger("camera_diag")
    log.info(f"BROWSER_DIAG: {str(data)[:500]}")
    html = data.get("html", "")
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
threading.Thread(target=telemetry.telemetry_loop, args=(ui,), daemon=True).start()
threading.Thread(target=navigation_loop, daemon=True).start()
camera.start_frame_grabber()   # start pulling JPEG frames from the brick stream
App.run()
