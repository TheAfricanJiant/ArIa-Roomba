import logging
import threading
import time

log = logging.getLogger(__name__)

_lock = threading.Lock()
_latest_pwm = 0
_sent_pwm = None
_worker_started = False


def _vacuum_worker():
    """Send only the newest requested PWM so slider drags do not queue stale values."""
    global _sent_pwm
    # Import Bridge lazily — it may not be ready at module-load time
    try:
        from arduino.app_utils import Bridge
    except ImportError as e:
        log.error("Vacuum worker: arduino.app_utils.Bridge not available: %s", e)
        return

    while True:
        with _lock:
            pwm = _latest_pwm
        if pwm != _sent_pwm:
            try:
                Bridge.call("set_vacuum_pwm", pwm)
                _sent_pwm = pwm
                log.debug("Vacuum RPC sent: pwm=%d", pwm)
            except Exception as e:
                log.warning("Vacuum RPC failed: %s", e)
                time.sleep(0.2)
        time.sleep(0.02)


def _ensure_worker():
    global _worker_started
    if _worker_started:
        return
    with _lock:
        if _worker_started:
            return
        threading.Thread(target=_vacuum_worker, daemon=True).start()
        _worker_started = True


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

    _ensure_worker()
    with _lock:
        global _latest_pwm
        _latest_pwm = pwm_i

    return pwm_i


def off() -> int:
    return set_vacuum(0)
