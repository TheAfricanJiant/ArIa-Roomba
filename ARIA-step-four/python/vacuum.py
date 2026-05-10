import logging
import serial_bridge

log = logging.getLogger(__name__)


def set_vacuum(pwm: int) -> int:
    """
    Set vacuum motor PWM (0-255).

    Protocol to Arduino (over socat bridge):
      V,<pwm>\n
    The Arduino sketch must implement this command and drive the L298N pins.
    """
    try:
        pwm_i = int(pwm)
    except Exception:
        pwm_i = 0
    pwm_i = max(0, min(255, pwm_i))

    if serial_bridge.is_connected():
        serial_bridge.send(f"V,{pwm_i}\n")
        log.debug("Vacuum cmd sent: pwm=%d", pwm_i)
    else:
        log.debug("Dummy vacuum cmd: pwm=%d", pwm_i)

    return pwm_i


def off() -> int:
    return set_vacuum(0)

