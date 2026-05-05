from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI
import threading
import logging

import serial_bridge
import motor
import telemetry
import navigator
import time
import json
import os

# ── Logging ────────────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

# ── Global state ───────────────────────────────────────────────────────────────
state = {
    "motors_on": False,
    "speed": 160,
    "navigating": False
}

nav = navigator.Navigator()

# ── Connect to Arduino via socat bridge ────────────────────────────────────────
serial_bridge.connect()

# ── UI event handlers ──────────────────────────────────────────────────────────
def get_state():
    return state


def toggle_power(client, data):
    global state
    state["motors_on"] = not state["motors_on"]
    
    # If turning off, also cancel any active navigation
    if not state["motors_on"]:
        state["navigating"] = False
        nav.clear_goal()
        motor.send_motor_cmd(0, 0)
    else:
        # If turning on but we were navigating, resume navigation or just start motors?
        # Actually, if not navigating, just run motors forward
        if not state["navigating"]:
            motor.send_motor_cmd(state["speed"], state["speed"])
        
    ui.send_message('state_update', get_state())
    log.info(f"Motors toggled: {state['motors_on']}")


def set_speed(client, data):
    global state
    state["speed"] = data.get("speed", 160)
    ui.send_message('state_update', get_state())
    log.info(f"Speed set to: {state['speed']}")
    if state["motors_on"]:
        motor.send_motor_cmd(state["speed"], state["speed"])


def on_get_initial_state(client, data):
    ui.send_message('state_update', get_state(), client)
    ui.send_message('routines_list', _load_routines(), client)


# ── Routines Management ────────────────────────────────────────────────────────
ROUTINES_FILE = "routines.json"

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
        log.info(f"Routine '{routine_name}' saved.")
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


def set_path(client, data):
    global state
    points = data.get("path", [])
    if not points:
        return
    path = [(p["x"], p["y"]) for p in points]
    nav.set_path(path, state["speed"])
    state["navigating"] = True
    state["motors_on"] = True
    ui.send_message('state_update', get_state())
    ui.send_message('path_update', [{"x": p[0], "y": p[1]} for p in path])
    log.info(f"Path set with {len(path)} waypoints")


def clean_zone(client, data):
    global state
    zone = data.get("zone")
    if not zone:
        return
    x_min, x_max = min(zone["x_min"], zone["x_max"]), max(zone["x_min"], zone["x_max"])
    y_min, y_max = min(zone["y_min"], zone["y_max"]), max(zone["y_min"], zone["y_max"])
    
    # 1. Perimeter Sweep
    path.append((x_min, y_min))
    path.append((x_max, y_min))
    path.append((x_max, y_max))
    path.append((x_min, y_max))
    path.append((x_min, y_min))
    
    # 2. Dense Lawnmower pattern
    lane_width = 20.0 # cm, ensuring 33% overlap for a 30cm wide robot
    
    # Sweep horizontally (X), shift vertically (Y)
    current_y = y_min + lane_width / 2.0
    going_right = True
    
    while current_y <= y_max:
        if going_right:
            path.append((x_min, current_y))
            path.append((x_max, current_y))
        else:
            path.append((x_max, current_y))
            path.append((x_min, current_y))
        
        current_y += lane_width
        going_right = not going_right
        
    if not path:
        return
        
    nav.set_path(path, state["speed"])
    state["navigating"] = True
    state["motors_on"] = True
    ui.send_message('state_update', get_state())
    ui.send_message('path_update', [{"x": p[0], "y": p[1]} for p in path])
    log.info(f"Zone cleaning started with {len(path)} waypoints")


def clear_goal(client, data):
    global state
    nav.clear_goal()
    state["navigating"] = False
    state["motors_on"] = False
    motor.send_motor_cmd(0, 0)
    ui.send_message('state_update', get_state())
    log.info("Goal cleared")


def navigation_loop():
    global state
    last_waypoints_count = -1
    while True:
        if state["navigating"] and state["motors_on"]:
            pose = telemetry.get_pose()
            if pose:
                x, y, theta = pose["x_cm"], pose["y_cm"], pose["theta_rad"]
                l_speed, r_speed, arrived = nav.step(x, y, theta)
                
                # Send the commands to the motor
                motor.send_motor_cmd(l_speed, r_speed)
                
                # Broadcast updated path if it changed
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
                    ui.send_message('path_update', []) # clear path
        time.sleep(0.05)  # 20 Hz


# ── Wire up UI ─────────────────────────────────────────────────────────────────
ui = WebUI()
ui.on_message('toggle_power', toggle_power)
ui.on_message('set_speed', set_speed)
ui.on_message('get_initial_state', on_get_initial_state)
ui.on_message('set_goal', set_goal)
ui.on_message('set_path', set_path)
ui.on_message('clean_zone', clean_zone)
ui.on_message('clear_goal', clear_goal)
ui.on_message('save_routine', save_routine)

# ── Start background threads ───────────────────────────────────────────────────
threading.Thread(target=telemetry.telemetry_loop, args=(ui,), daemon=True).start()
threading.Thread(target=navigation_loop, daemon=True).start()

App.run()