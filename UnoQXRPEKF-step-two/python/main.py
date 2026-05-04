from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI
import threading
import logging

import serial_bridge
import motor
import telemetry
import navigator
import time

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


def navigation_loop():
    global state
    while True:
        if state["navigating"] and state["motors_on"]:
            pose = telemetry.get_pose()
            if pose:
                x, y, theta = pose["x_cm"], pose["y_cm"], pose["theta_rad"]
                l_speed, r_speed, arrived = nav.step(x, y, theta)
                
                # Send the commands to the motor
                motor.send_motor_cmd(l_speed, r_speed)
                
                if arrived:
                    state["navigating"] = False
                    state["motors_on"] = False
                    motor.send_motor_cmd(0, 0)
                    ui.send_message('state_update', get_state())
        time.sleep(0.05)  # 20 Hz


# ── Wire up UI ─────────────────────────────────────────────────────────────────
ui = WebUI()
ui.on_message('toggle_power', toggle_power)
ui.on_message('set_speed', set_speed)
ui.on_message('get_initial_state', on_get_initial_state)
ui.on_message('set_goal', set_goal)
ui.on_message('clear_goal', clear_goal)

# ── Start background threads ───────────────────────────────────────────────────
threading.Thread(target=telemetry.telemetry_loop, args=(ui,), daemon=True).start()
threading.Thread(target=navigation_loop, daemon=True).start()

App.run()