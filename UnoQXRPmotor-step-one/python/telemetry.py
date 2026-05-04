import time
import logging
import serial_bridge

log = logging.getLogger(__name__)

telemetry = {
    "enc_l": 0, "enc_r": 0,
    "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
    "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0
}


def get_telemetry():
    return telemetry


def _parse_line(line: str) -> bool:
    """Parse a telemetry line. Returns True if valid data was parsed."""
    if not line.startswith('T,'):
        return False
    parts = line.split(',')
    if len(parts) < 10:
        return False
    try:
        telemetry["enc_l"]   = int(parts[1])
        telemetry["enc_r"]   = int(parts[2])
        telemetry["accel_x"] = float(parts[3])
        telemetry["accel_y"] = float(parts[4])
        telemetry["accel_z"] = float(parts[5])
        telemetry["gyro_x"]  = float(parts[6])
        telemetry["gyro_y"]  = float(parts[7])
        telemetry["gyro_z"]  = float(parts[8])
        return True
    except (ValueError, IndexError) as e:
        log.warning(f"Failed to parse telemetry line '{line}': {e}")
        return False


def telemetry_loop(ui):
    """Continuously read telemetry from bridge and push to UI."""
    while True:
        if serial_bridge.is_connected():
            # Drain the buffer completely
            while True:
                line = serial_bridge.readline()
                if not line:
                    break  # No more full lines available
                
                if _parse_line(line):
                    ui.send_message('telemetry_update', telemetry)
        time.sleep(0.05)