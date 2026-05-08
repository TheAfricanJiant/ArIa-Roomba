# SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
#
# SPDX-License-Identifier: MPL-2.0

#

# EXAMPLE_NAME = "ARIA Telegram Bot — Step Three"

from arduino.app_bricks.telegram_bot import TelegramBot, Sender, Message
from arduino.app_bricks.object_detection import ObjectDetection
from arduino.app_bricks.mood_detector import MoodDetector
from arduino.app_utils import App
from PIL import Image
from io import BytesIO
import json
import os

# Initialize bricks
bot = TelegramBot()
obj_detection = ObjectDetection()
mood = MoodDetector()

# ── Named Areas Storage ────────────────────────────────────────────────────────
# Areas are saved as { "name": { "x": float, "y": float } }
AREAS_FILE = "saved_areas.json"


def _load_areas() -> dict:
    """Load saved named areas from disk."""
    if not os.path.exists(AREAS_FILE):
        return {}
    try:
        with open(AREAS_FILE, "r") as f:
            return json.load(f)
    except Exception:
        return {}


def _save_areas(areas: dict):
    """Persist named areas to disk."""
    try:
        with open(AREAS_FILE, "w") as f:
            json.dump(areas, f, indent=2)
    except Exception:
        pass


# ══════════════════════════════════════════════════════════════════════════════
# STUB HELPER — used by all commands that are not yet wired to ARIA
# ══════════════════════════════════════════════════════════════════════════════
NOT_IMPL = "🚧 *Not yet implemented.*\nThis command will be connected to ARIA in a future update."


# ══════════════════════════════════════════════════════════════════════════════
# EXISTING COMMANDS (working)
# ══════════════════════════════════════════════════════════════════════════════

def greet(sender: Sender, message: Message):
    """Handle /hello command - super simple API with reply helper!"""
    sender.reply(f"👋 Hi {sender.first_name}! This is ARIA on Arduino UNO Q!")


def help_cmd(sender: Sender, message: Message):
    """Handle /help command — shows the full ARIA command list."""
    help_text = (
        "🤖 *ARIA Telegram Commands*\n"
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n"
        "🎮 *Control*\n"
        "/forward — Drive forward\n"
        "/backward — Drive backward\n"
        "/left — Spin left\n"
        "/right — Spin right\n"
        "/stop — Emergency stop\n"
        "/speed <0-255> — Set motor speed\n"
        "/mode <auto|manual> — Switch mode\n\n"
        "🧹 *Cleaning*\n"
        "/clean — Start cleaning\n"
        "/stopclean — Abort cleaning\n"
        "/dock — Return to dock\n"
        "/setdock — Save current pos as dock\n"
        "/cleanzone — Clean a rectangular area\n\n"
        "📍 *Navigation*\n"
        "/goto <area> — Navigate to a named area\n"
        "/areas — List saved areas\n"
        "/savearea <name> — Save current pos as area\n"
        "/deletearea <name> — Delete a saved area\n"
        "/cancelpath — Cancel navigation\n\n"
        "📊 *Status*\n"
        "/status — Full status dashboard\n"
        "/pose — Current position\n"
        "/sensors — Raw IMU data\n"
        "/battery — Battery level\n"
        "/coverage — Grid coverage %\n\n"
        "📷 *Camera*\n"
        "/photo — Take snapshot\n"
        "/detect — Photo + object detection\n"
        "/record <sec> — Record video clip\n\n"
        "🗺️ *Map*\n"
        "/map — Send occupancy grid\n"
        "/heatmap — Send dirt heatmap\n"
        "/resetpose — Reset to origin\n\n"
        "🔧 *Maintenance*\n"
        "/vacuum <0-255> — Set vacuum power\n"
        "/brush <0-255> — Set brush speed\n"
        "/ping — Check bot is alive\n"
        "/log — Show recent log lines\n\n"
        "🔔 *Alerts*\n"
        "/alerts <on|off> — Toggle push alerts\n\n"
        "ℹ️ /hello — Greeting  |  /help — This menu\n"
        "📝 Send text → mood detection\n"
        "📷 Send photo → object detection\n"
    )
    sender.reply(help_text)


def sentiment(sender: Sender, message: Message):
    """Reply sentiment analysis for text messages."""
    result = mood.get_sentiment(message.text)
    sender.reply(f"Your mood is: {result}")


def detect_objects(
    sender: Sender,
    message: Message,
    photo: bytes,
    filename: str,
    size: int,
):
    """Detect objects in photos - photo data passed as parameter!"""
    sender.reply("📷 Detecting objects...")
    image = Image.open(BytesIO(photo))
    results = obj_detection.detect(image, confidence=0.1)
    img_with_boxes = obj_detection.draw_bounding_boxes(image, results)

    output = BytesIO()
    img_with_boxes.save(output, format="PNG")
    output.seek(0)

    caption = f"✅ Found {len(results['detection'])} object(s)!" if results else "No objects detected"

    if not sender.reply_photo(output.getvalue(), caption):
        sender.reply("❌ Failed to send processed image")


# ══════════════════════════════════════════════════════════════════════════════
# NAVIGATION — /goto, /areas, /savearea, /deletearea  (area-aware)
# ══════════════════════════════════════════════════════════════════════════════

def goto_cmd(sender: Sender, message: Message):
    """Navigate to a named area.  Usage: /goto <area_name>"""
    args = message.text.strip().split(maxsplit=1)

    # No argument → show available areas as a hint
    if len(args) < 2 or not args[1].strip():
        areas = _load_areas()
        if not areas:
            sender.reply(
                "📍 *Usage:* `/goto <area_name>`\n\n"
                "You have no saved areas yet.\n"
                "Use `/savearea <name>` to save the robot's current position."
            )
        else:
            lines = [f"📍 *Usage:* `/goto <area_name>`\n\nAvailable areas:"]
            for name, coords in areas.items():
                lines.append(f"  • *{name}* — ({coords['x']}, {coords['y']}) cm")
            lines.append(f"\nExample: `/goto {list(areas.keys())[0]}`")
            sender.reply("\n".join(lines))
        return

    area_name = args[1].strip().lower()
    areas = _load_areas()

    if area_name not in areas:
        # Suggest closest matches
        available = ", ".join(f"*{n}*" for n in areas.keys()) if areas else "none"
        sender.reply(
            f"❌ Area *\"{area_name}\"* not found.\n\n"
            f"Available areas: {available}\n"
            f"Use `/areas` to see all saved areas."
        )
        return

    coords = areas[area_name]
    # TODO: Wire to navigator.set_goal(coords["x"], coords["y"], speed)
    sender.reply(
        f"🚧 *Not yet implemented.*\n\n"
        f"Would navigate to *{area_name}* at ({coords['x']}, {coords['y']}) cm.\n"
        f"This will call `navigator.set_goal()` once wired to ARIA."
    )


def areas_cmd(sender: Sender, message: Message):
    """List all saved named areas."""
    areas = _load_areas()
    if not areas:
        sender.reply(
            "📍 *No saved areas.*\n\n"
            "Use `/savearea <name>` to save the robot's current position as a named area."
        )
        return

    lines = ["📍 *Saved Areas:*\n"]
    for name, coords in areas.items():
        lines.append(f"  • *{name}* — ({coords['x']}, {coords['y']}) cm")
    lines.append(f"\n_{len(areas)} area(s) total._")
    lines.append("Use `/goto <name>` to navigate to an area.")
    sender.reply("\n".join(lines))


def savearea_cmd(sender: Sender, message: Message):
    """Save the current robot position as a named area.  Usage: /savearea <name>"""
    args = message.text.strip().split(maxsplit=1)

    if len(args) < 2 or not args[1].strip():
        sender.reply("📍 *Usage:* `/savearea <name>`\nExample: `/savearea kitchen`")
        return

    area_name = args[1].strip().lower()
    areas = _load_areas()

    # TODO: Get real position from telemetry.get_pose()
    # For now, store a placeholder (0, 0) — will be replaced with EKF pose
    areas[area_name] = {"x": 0.0, "y": 0.0}
    _save_areas(areas)

    sender.reply(
        f"✅ Area *\"{area_name}\"* saved!\n\n"
        f"🚧 _Position is placeholder (0, 0) — will use real EKF pose once wired to ARIA._\n"
        f"Navigate here later with `/goto {area_name}`"
    )


def deletearea_cmd(sender: Sender, message: Message):
    """Delete a saved area.  Usage: /deletearea <name>"""
    args = message.text.strip().split(maxsplit=1)

    if len(args) < 2 or not args[1].strip():
        sender.reply("📍 *Usage:* `/deletearea <name>`")
        return

    area_name = args[1].strip().lower()
    areas = _load_areas()

    if area_name not in areas:
        sender.reply(f"❌ Area *\"{area_name}\"* not found. Use `/areas` to see saved areas.")
        return

    del areas[area_name]
    _save_areas(areas)
    sender.reply(f"🗑️ Area *\"{area_name}\"* deleted.")


def cancelpath_cmd(sender: Sender, message: Message):
    """Cancel current navigation path."""
    # TODO: Wire to navigator.clear_goal()
    sender.reply(NOT_IMPL)


# ══════════════════════════════════════════════════════════════════════════════
# ROBOT CONTROL — /forward, /backward, /left, /right, /stop, /speed, /mode
# ══════════════════════════════════════════════════════════════════════════════

def forward_cmd(sender: Sender, message: Message):
    """Drive forward.  Usage: /forward [speed]"""
    # TODO: Wire to motor.send_motor_cmd(speed, speed)
    sender.reply(NOT_IMPL)


def backward_cmd(sender: Sender, message: Message):
    """Drive backward.  Usage: /backward [speed]"""
    # TODO: Wire to motor.send_motor_cmd(-speed, -speed)
    sender.reply(NOT_IMPL)


def left_cmd(sender: Sender, message: Message):
    """Spin left.  Usage: /left [degrees]"""
    # TODO: Wire to timed motor.send_motor_cmd(-speed, speed)
    sender.reply(NOT_IMPL)


def right_cmd(sender: Sender, message: Message):
    """Spin right.  Usage: /right [degrees]"""
    # TODO: Wire to timed motor.send_motor_cmd(speed, -speed)
    sender.reply(NOT_IMPL)


def stop_cmd(sender: Sender, message: Message):
    """Emergency stop — kill all motors immediately."""
    # TODO: Wire to motor.send_motor_cmd(0, 0) — THIS MUST ALWAYS WORK
    sender.reply(NOT_IMPL)


def speed_cmd(sender: Sender, message: Message):
    """Set base motor speed.  Usage: /speed <0-255>"""
    # TODO: Wire to state["speed"] = value
    sender.reply(NOT_IMPL)


def mode_cmd(sender: Sender, message: Message):
    """Switch mode.  Usage: /mode <auto|manual>"""
    # TODO: Wire to on_set_mode()
    sender.reply(NOT_IMPL)


# ══════════════════════════════════════════════════════════════════════════════
# CLEANING — /clean, /stopclean, /dock, /setdock, /cleanzone
# ══════════════════════════════════════════════════════════════════════════════

def clean_cmd(sender: Sender, message: Message):
    """Start autonomous cleaning run."""
    # TODO: Wire to cleaner.start(x, y)
    sender.reply(NOT_IMPL)


def stopclean_cmd(sender: Sender, message: Message):
    """Abort cleaning, return to IDLE."""
    # TODO: Wire to cleaner.stop()
    sender.reply(NOT_IMPL)


def dock_cmd(sender: Sender, message: Message):
    """Send robot to dock immediately."""
    # TODO: Wire to cleaner.notify_low_battery()
    sender.reply(NOT_IMPL)


def setdock_cmd(sender: Sender, message: Message):
    """Save current position as dock station."""
    # TODO: Wire to cleaner.set_dock_position(x, y)
    sender.reply(NOT_IMPL)


def cleanzone_cmd(sender: Sender, message: Message):
    """Clean a rectangular area.  Usage: /cleanzone <x_min> <y_min> <x_max> <y_max>"""
    # TODO: Wire to clean_zone() lawnmower pattern
    sender.reply(NOT_IMPL)


# ══════════════════════════════════════════════════════════════════════════════
# TELEMETRY & STATUS — /status, /pose, /sensors, /battery, /coverage
# ══════════════════════════════════════════════════════════════════════════════

def status_cmd(sender: Sender, message: Message):
    """Full status dashboard."""
    # TODO: Wire to get_state(), get_pose(), cleaner.state_name, _battery_pct
    sender.reply(NOT_IMPL)


def pose_cmd(sender: Sender, message: Message):
    """Current EKF position (x, y, θ)."""
    # TODO: Wire to telemetry.get_pose()
    sender.reply(NOT_IMPL)


def sensors_cmd(sender: Sender, message: Message):
    """Raw IMU + encoder readings."""
    # TODO: Wire to telemetry dict (accel_*, gyro_*, enc_*)
    sender.reply(NOT_IMPL)


def battery_cmd(sender: Sender, message: Message):
    """Battery percentage."""
    # TODO: Wire to _battery_pct
    sender.reply(NOT_IMPL)


def coverage_cmd(sender: Sender, message: Message):
    """Grid coverage percentage."""
    # TODO: Wire to grid.coverage_percent()
    sender.reply(NOT_IMPL)


# ══════════════════════════════════════════════════════════════════════════════
# CAMERA & MEDIA — /photo, /record
# ══════════════════════════════════════════════════════════════════════════════

def photo_cmd(sender: Sender, message: Message):
    """Capture a snapshot and send as photo."""
    # TODO: Wire to on_take_snapshot() → camera capture → reply_photo
    sender.reply(NOT_IMPL)


def record_cmd(sender: Sender, message: Message):
    """Record N seconds of video.  Usage: /record <seconds>"""
    # TODO: Wire to on_start_recording() → wait → on_stop_recording()
    sender.reply(NOT_IMPL)


# ══════════════════════════════════════════════════════════════════════════════
# MAP & GRID — /map, /heatmap, /resetpose
# ══════════════════════════════════════════════════════════════════════════════

def map_cmd(sender: Sender, message: Message):
    """Send current occupancy grid as image."""
    # TODO: Wire to on_get_map() → matplotlib render → reply_photo
    sender.reply(NOT_IMPL)


def heatmap_cmd(sender: Sender, message: Message):
    """Send dirt heatmap overlay."""
    # TODO: Wire to DirtHeatmap export (Phase 6)
    sender.reply(NOT_IMPL)


def resetpose_cmd(sender: Sender, message: Message):
    """Reset EKF to origin (0, 0, 0°)."""
    # TODO: Wire to ekf.reset(0, 0, 0)
    sender.reply(NOT_IMPL)


# ══════════════════════════════════════════════════════════════════════════════
# MAINTENANCE — /vacuum, /brush, /ping, /log
# ══════════════════════════════════════════════════════════════════════════════

def vacuum_cmd(sender: Sender, message: Message):
    """Set vacuum motor power.  Usage: /vacuum <0-255>"""
    # TODO: Wire to bridge.set_vacuum(pwm)
    sender.reply(NOT_IMPL)


def brush_cmd(sender: Sender, message: Message):
    """Set brush motor speed.  Usage: /brush <0-255>"""
    # TODO: Wire to bridge.set_brush(pwm)
    sender.reply(NOT_IMPL)


def ping_cmd(sender: Sender, message: Message):
    """Check bot is alive + uptime."""
    # This one works right away — no hardware needed!
    import time
    sender.reply(f"🏓 *Pong!*\nBot is alive.\nServer time: {time.strftime('%H:%M:%S')}")


def log_cmd(sender: Sender, message: Message):
    """Show recent log lines."""
    # TODO: Read from log buffer
    sender.reply(NOT_IMPL)


# ══════════════════════════════════════════════════════════════════════════════
# ALERTS — /alerts
# ══════════════════════════════════════════════════════════════════════════════

def alerts_cmd(sender: Sender, message: Message):
    """Toggle push alerts.  Usage: /alerts <on|off>"""
    # TODO: Wire to alert system
    sender.reply(NOT_IMPL)


# ══════════════════════════════════════════════════════════════════════════════
# REGISTER ALL COMMANDS
# ══════════════════════════════════════════════════════════════════════════════

# -- Original (working) --
bot.add_command("hello", greet, "Get a personalized greeting")
bot.add_command("help", help_cmd, "Show available commands")

# -- Robot Control --
bot.add_command("forward", forward_cmd, "Drive forward")
bot.add_command("backward", backward_cmd, "Drive backward")
bot.add_command("left", left_cmd, "Spin left")
bot.add_command("right", right_cmd, "Spin right")
bot.add_command("stop", stop_cmd, "Emergency stop")
bot.add_command("speed", speed_cmd, "Set motor speed (0-255)")
bot.add_command("mode", mode_cmd, "Switch auto/manual mode")

# -- Navigation (area-aware) --
bot.add_command("goto", goto_cmd, "Navigate to a named area")
bot.add_command("areas", areas_cmd, "List saved areas")
bot.add_command("savearea", savearea_cmd, "Save current position as area")
bot.add_command("deletearea", deletearea_cmd, "Delete a saved area")
bot.add_command("cancelpath", cancelpath_cmd, "Cancel current navigation")

# -- Cleaning --
bot.add_command("clean", clean_cmd, "Start autonomous cleaning")
bot.add_command("stopclean", stopclean_cmd, "Abort cleaning")
bot.add_command("dock", dock_cmd, "Return to dock")
bot.add_command("setdock", setdock_cmd, "Save current pos as dock")
bot.add_command("cleanzone", cleanzone_cmd, "Clean a rectangular area")

# -- Telemetry & Status --
bot.add_command("status", status_cmd, "Full status dashboard")
bot.add_command("pose", pose_cmd, "Current EKF position")
bot.add_command("sensors", sensors_cmd, "Raw IMU + encoder data")
bot.add_command("battery", battery_cmd, "Battery percentage")
bot.add_command("coverage", coverage_cmd, "Grid coverage percent")

# -- Camera & Media --
bot.add_command("photo", photo_cmd, "Take a snapshot")
bot.add_command("record", record_cmd, "Record video clip")

# -- Map & Grid --
bot.add_command("map", map_cmd, "Send occupancy grid image")
bot.add_command("heatmap", heatmap_cmd, "Send dirt heatmap")
bot.add_command("resetpose", resetpose_cmd, "Reset EKF to origin")

# -- Maintenance --
bot.add_command("vacuum", vacuum_cmd, "Set vacuum power (0-255)")
bot.add_command("brush", brush_cmd, "Set brush speed (0-255)")
bot.add_command("ping", ping_cmd, "Check bot is alive")
bot.add_command("log", log_cmd, "Show recent log lines")

# -- Alerts --
bot.add_command("alerts", alerts_cmd, "Toggle push alerts on/off")

# -- Text & Photo handlers (original — working) --
bot.on_text(sentiment)
bot.on_photo(detect_objects)

# Start the Arduino App framework
App.run()