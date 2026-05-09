from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.telegram_bot import TelegramBot, Sender, Message
from arduino.app_bricks.object_detection import ObjectDetection
from arduino.app_bricks.mood_detector import MoodDetector
from arduino.app_bricks.video_objectdetection import VideoObjectDetection
from PIL import Image
from io import BytesIO
from datetime import datetime, UTC
import threading
import logging
import time
import json
import os

import serial_bridge
import motor
import telemetry
import navigator

# ── Logging ────────────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

# ── Initialize bricks ──────────────────────────────────────────────────────────
ui = WebUI()
bot = TelegramBot()
obj_detection = ObjectDetection()
mood = MoodDetector()
video_detection = VideoObjectDetection(confidence=0.5, debounce_sec=0.0)

# ── Global state ───────────────────────────────────────────────────────────────
state = {
    "motors_on": False,
    "speed": 160,
    "navigating": False
}

nav = navigator.Navigator()
AREAS_FILE = "saved_areas.json"
ROUTINES_FILE = "routines.json"

# ── Connect to Arduino via socat bridge ────────────────────────────────────────
serial_bridge.connect()

# ── Video Detection Handlers ───────────────────────────────────────────────────
ui.on_message("override_th", lambda sid, threshold: video_detection.override_threshold(threshold))

def send_detections_to_ui(detections: dict):
    """Callback for live USB camera object detection"""
    for key, values in detections.items():
        for value in values:
            entry = {
                "content": key,
                "confidence": value.get("confidence"),
                "timestamp": datetime.now(UTC).isoformat()
            }
            ui.send_message("detection", message=entry)
            
            # Example optional Telegram integration: 
            # If confidence is very high for specific items, could trigger alert

video_detection.on_detect_all(send_detections_to_ui)

# ── UI event handlers ──────────────────────────────────────────────────────────
def get_state():
    return state

def toggle_power(client, data):
    global state
    state["motors_on"] = not state["motors_on"]
    
    if not state["motors_on"]:
        state["navigating"] = False
        nav.clear_goal()
        motor.send_motor_cmd(0, 0)
    else:
        if not state["navigating"]:
            motor.send_motor_cmd(state["speed"], state["speed"])
        
    ui.send_message('state_update', get_state())
    log.info(f"Motors toggled: {state['motors_on']}")

def set_speed(client, data):
    global state
    state["speed"] = data.get("speed", 160)
    ui.send_message('state_update', get_state())
    log.info(f"Speed set to: {state['speed']}")
    if state["motors_on"] and not state["navigating"]:
        motor.send_motor_cmd(state["speed"], state["speed"])

def on_get_initial_state(client, data):
    ui.send_message('state_update', get_state(), client)
    ui.send_message('routines_list', _load_routines(), client)

def _load_routines():
    if not os.path.exists(ROUTINES_FILE):
        return []
    try:
        with open(ROUTINES_FILE, "r") as f:
            return json.load(f)
    except Exception as e:
        log.error(f"Failed to load routines: {e}")
        return []

def save_routine(client, data):
    routine_name = data.get("name")
    if not routine_name: return
    routines = _load_routines()
    routines.append({
        "name": routine_name,
        "type": data.get("type"),
        "data": data.get("data")
    })
    try:
        with open(ROUTINES_FILE, "w") as f:
            json.dump(routines, f, indent=2)
        ui.send_message('routines_list', routines)
    except Exception as e:
        log.error(f"Failed to save routine: {e}")

def set_goal(client, data):
    global state
    x = data.get("x", 0.0)
    y = data.get("y", 0.0)
    nav.set_goal(x, y, state["speed"])
    state["navigating"] = True
    state["motors_on"] = True
    ui.send_message('state_update', get_state())
    log.info(f"Goal set to: {x}, {y}")

def clear_goal(client, data):
    global state
    nav.clear_goal()
    state["navigating"] = False
    state["motors_on"] = False
    motor.send_motor_cmd(0, 0)
    ui.send_message('state_update', get_state())
    log.info("Goal cleared")

# ── Telegram Bot Command Handlers ──────────────────────────────────────────────
def _load_areas() -> dict:
    if not os.path.exists(AREAS_FILE): return {}
    try:
        with open(AREAS_FILE, "r") as f:
            return json.load(f)
    except Exception:
        return {}

def _save_areas(areas: dict):
    try:
        with open(AREAS_FILE, "w") as f:
            json.dump(areas, f, indent=2)
    except Exception:
        pass

def greet(sender: Sender, message: Message):
    sender.reply(f"👋 Hi {sender.first_name}! This is ARIA (Integrated Master Controller)!")

def status_cmd(sender: Sender, message: Message):
    pose = telemetry.get_pose()
    sender.reply(
        f"📊 *ARIA Status*\n"
        f"Motors On: {state['motors_on']}\n"
        f"Navigating: {state['navigating']}\n"
        f"Speed: {state['speed']}\n"
        f"Pose: X:{pose['x_cm']}cm, Y:{pose['y_cm']}cm, θ:{pose['theta_rad']}rad"
    )

def forward_cmd(sender: Sender, message: Message):
    global state
    state["motors_on"] = True
    state["navigating"] = False
    motor.send_motor_cmd(state["speed"], state["speed"])
    sender.reply("🚀 Driving forward!")

def stop_cmd(sender: Sender, message: Message):
    global state
    state["motors_on"] = False
    state["navigating"] = False
    nav.clear_goal()
    motor.send_motor_cmd(0, 0)
    sender.reply("🛑 Emergency stop executed!")

def goto_cmd(sender: Sender, message: Message):
    args = message.text.strip().split(maxsplit=1)
    if len(args) < 2 or not args[1].strip():
        sender.reply("📍 Usage: `/goto <area_name>`")
        return
    
    area_name = args[1].strip().lower()
    areas = _load_areas()
    if area_name not in areas:
        sender.reply(f"❌ Area '{area_name}' not found.")
        return
        
    coords = areas[area_name]
    nav.set_goal(coords["x"], coords["y"], state["speed"])
    state["navigating"] = True
    state["motors_on"] = True
    sender.reply(f"🚀 Navigating to {area_name} ({coords['x']}, {coords['y']})")

def savearea_cmd(sender: Sender, message: Message):
    args = message.text.strip().split(maxsplit=1)
    if len(args) < 2 or not args[1].strip():
        sender.reply("📍 Usage: `/savearea <name>`")
        return
        
    area_name = args[1].strip().lower()
    areas = _load_areas()
    pose = telemetry.get_pose()
    areas[area_name] = {"x": pose["x_cm"], "y": pose["y_cm"]}
    _save_areas(areas)
    sender.reply(f"✅ Area '{area_name}' saved at current position!")

def detect_objects(sender: Sender, message: Message, photo: bytes, filename: str, size: int):
    sender.reply("📷 Processing static image...")
    image = Image.open(BytesIO(photo))
    results = obj_detection.detect(image, confidence=0.1)
    img_with_boxes = obj_detection.draw_bounding_boxes(image, results)
    
    output = BytesIO()
    img_with_boxes.save(output, format="PNG")
    output.seek(0)
    
    caption = f"✅ Found {len(results['detection'])} object(s)!" if results else "No objects detected"
    sender.reply_photo(output.getvalue(), caption)

def sentiment(sender: Sender, message: Message):
    result = mood.get_sentiment(message.text)
    sender.reply(f"Text Sentiment: {result}")

# Register Telegram Commands
bot.add_command("hello", greet, "Get a personalized greeting")
bot.add_command("status", status_cmd, "Get robot telemetry and state")
bot.add_command("forward", forward_cmd, "Drive forward")
bot.add_command("stop", stop_cmd, "Emergency stop")
bot.add_command("goto", goto_cmd, "Navigate to a saved area")
bot.add_command("savearea", savearea_cmd, "Save current position")
bot.on_text(sentiment)
bot.on_photo(detect_objects)

# ── Wire up UI ─────────────────────────────────────────────────────────────────
ui.on_message('toggle_power', toggle_power)
ui.on_message('set_speed', set_speed)
ui.on_message('get_initial_state', on_get_initial_state)
ui.on_message('set_goal', set_goal)
ui.on_message('clear_goal', clear_goal)
ui.on_message('save_routine', save_routine)

# ── Navigation Loop ────────────────────────────────────────────────────────────
def navigation_loop():
    global state
    last_waypoints_count = -1
    while True:
        if state["navigating"] and state["motors_on"]:
            pose = telemetry.get_pose()
            if pose:
                x, y, theta = pose["x_cm"], pose["y_cm"], pose["theta_rad"]
                l_speed, r_speed, arrived = nav.step(x, y, theta)
                
                motor.send_motor_cmd(l_speed, r_speed)
                
                current_count = len(nav.waypoints)
                if current_count != last_waypoints_count:
                    path_to_send = []
                    if nav.goal:
                        path_to_send.append({"x": nav.goal[0], "y": nav.goal[1]})
                    for p in nav.waypoints:
                        path_to_send.append({"x": p[0], "y": p[1]})
                    ui.send_message('path_update', path_to_send)
                    last_waypoints_count = current_count
                
                if arrived:
                    state["navigating"] = False
                    state["motors_on"] = False
                    motor.send_motor_cmd(0, 0)
                    ui.send_message('state_update', get_state())
                    ui.send_message('path_update', []) 
        time.sleep(0.05)

# ── Start background threads ───────────────────────────────────────────────────
threading.Thread(target=telemetry.telemetry_loop, args=(ui,), daemon=True).start()
threading.Thread(target=navigation_loop, daemon=True).start()

# Start the Arduino App framework
App.run()