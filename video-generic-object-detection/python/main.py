from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI
import threading
import time
import serial
import os
import logging

# Logger must be set up BEFORE init_serial() is called
logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

# Global state
state = {
    "motors_on": False,
    "speed": 160
}

telemetry = {
    "enc_l": 0, "enc_r": 0,
    "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
    "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0
}

# --- SERIAL SETUP ---
ser = None

def init_serial():
    global ser
    ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
    for p in ports:
        if os.path.exists(p):
            try:
                ser = serial.Serial(p, 115200, timeout=0.1)
                log.info(f"Connected to Arduino on {p}")
                return
            except Exception as e:
                log.error(f"Failed to open {p}: {e}")
    log.warning("No serial port found. Falling back to dummy data.")

init_serial()

def send_motor_cmd(left, right):
    if ser and ser.is_open:
        cmd = f"M,{left},{right}\n"
        try:
            ser.write(cmd.encode('utf-8'))
        except Exception as e:
            log.error(f"Failed to write to serial: {e}")

def get_state():
    return state

def toggle_power(client, data):
    global state
    state["motors_on"] = not state["motors_on"]
    ui.send_message('state_update', get_state())
    log.info(f"Motors toggled: {state['motors_on']}")

    if state["motors_on"]:
        send_motor_cmd(state["speed"], state["speed"])
    else:
        send_motor_cmd(0, 0)

def set_speed(client, data):
    global state
    state["speed"] = data.get("speed", 160)
    ui.send_message('state_update', get_state())
    log.info(f"Speed set to: {state['speed']}")

    if state["motors_on"]:
        send_motor_cmd(state["speed"], state["speed"])

def on_get_initial_state(client, data):
    ui.send_message('state_update', get_state(), client)

def telemetry_loop():
    while True:
        if ser and ser.is_open:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('T,'):
                    # Format: T,encL,encR,aX,aY,aZ,gX,gY,gZ
                    parts = line.split(',')
                    if len(parts) >= 10:
                        telemetry["enc_l"]   = int(parts[1])
                        telemetry["enc_r"]   = int(parts[2])
                        telemetry["accel_x"] = float(parts[3])
                        telemetry["accel_y"] = float(parts[4])
                        telemetry["accel_z"] = float(parts[5])
                        telemetry["gyro_x"]  = float(parts[6])
                        telemetry["gyro_y"]  = float(parts[7])
                        telemetry["gyro_z"]  = float(parts[8])
                        ui.send_message('telemetry_update', telemetry)
            except Exception:
                pass
        time.sleep(0.1)

ui = WebUI()
ui.on_message('toggle_power', toggle_power)
ui.on_message('set_speed', set_speed)
ui.on_message('get_initial_state', on_get_initial_state)

# Start telemetry reader thread
threading.Thread(target=telemetry_loop, daemon=True).start()

App.run()