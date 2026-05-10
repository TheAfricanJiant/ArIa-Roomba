import logging
from arduino.app_utils import Bridge

log = logging.getLogger(__name__)


def set_vacuum(pwm: int) -> int:
    """
    Set vacuum motor PWM (0-255).

    This calls the UNO Q sketch via RouterBridge:
      Bridge.call("set_vacuum_pwm", pwm)
    """
    try:
        pwm_i = int(pwm)
    except Exception:
        pwm_i = 0
    pwm_i = max(0, min(255, pwm_i))

    try:
        Bridge.call("set_vacuum_pwm", pwm_i)
        log.debug("Vacuum RPC sent: pwm=%d", pwm_i)
    except Exception as e:
        # If Bridge isn't available (e.g. running without the UNO Q sketch),
        # we keep the app alive and just log.
        log.warning("Vacuum RPC failed: %s", e)

    return pwm_i


def off() -> int:
    return set_vacuum(0)

