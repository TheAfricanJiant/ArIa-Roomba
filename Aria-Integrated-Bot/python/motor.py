import logging
import serial_bridge

log = logging.getLogger(__name__)


def send_motor_cmd(left: int, right: int):
    """Send motor speed command. Values typically 0-255."""
    if serial_bridge.is_connected():
        cmd = f"M,{left},{right}\n"
        serial_bridge.send(cmd)
        log.debug(f"Motor cmd sent: L={left} R={right}")
    else:
        log.debug(f"Dummy motor cmd: L={left} R={right}")