from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI
import threading
import logging

import serial_bridge
import motor
import telemetry

# ── Logging ────────────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

# ── Global state ───────────────────────────────────────────────────────────────
state = {
    "motors_on": False,
    "speed": 160
}

# ── Connect to Arduino via socat bridge ────────────────────────────────────────
serial_bridge.connect()

# ── UI event handlers ──────────────────────────────────────────────────────────
def get_state():
    return state


def toggle_power(client, data):
    global state
    state["motors_on"] = not state["motors_on"]
    ui.send_message('state_update', get_state())
    log.info(f"Motors toggled: {state['motors_on']}")
    if state["motors_on"]:
        motor.send_motor_cmd(state["speed"], state["speed"])
    else:
        motor.send_motor_cmd(0, 0)


def set_speed(client, data):
    global state
    state["speed"] = data.get("speed", 160)
    ui.send_message('state_update', get_state())
    log.info(f"Speed set to: {state['speed']}")
    if state["motors_on"]:
        motor.send_motor_cmd(state["speed"], state["speed"])


def on_get_initial_state(client, data):
    ui.send_message('state_update', get_state(), client)


# ── Wire up UI ─────────────────────────────────────────────────────────────────
ui = WebUI()
ui.on_message('toggle_power', toggle_power)
ui.on_message('set_speed', set_speed)
ui.on_message('get_initial_state', on_get_initial_state)

# ── Start telemetry thread ─────────────────────────────────────────────────────
threading.Thread(target=telemetry.telemetry_loop, args=(ui,), daemon=True).start()

App.run()