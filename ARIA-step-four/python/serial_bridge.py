import socket
import logging
import threading
import time

log = logging.getLogger(__name__)

HOST = '192.168.1.205'
PORT = 5000

sock = None
_reconnect_lock = threading.Lock()
_reconnecting   = False   # True while a background reconnect thread is running


def connect():
    """Connect to the socat TCP-to-serial bridge (called once at startup)."""
    global sock
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.settimeout(0.1)
        sock = s
        log.info(f"Connected to Arduino via socat bridge at {HOST}:{PORT}")
    except Exception as e:
        log.warning(f"Could not connect to socat bridge: {e}. "
                    "Will retry in background.")
        sock = None
        _start_reconnect_thread()


# ── Fix #4 (2026-05): auto-reconnect with exponential backoff ─────────────────
def _start_reconnect_thread():
    """Spawn a daemon thread that retries the connection if not already running."""
    global _reconnecting
    with _reconnect_lock:
        if _reconnecting:
            return
        _reconnecting = True
    t = threading.Thread(target=_reconnect_loop, daemon=True)
    t.start()


def _reconnect_loop():
    """Background reconnect loop — doubles delay on each failure (max 30 s)."""
    global sock, _reconnecting
    delay = 2.0
    while True:
        time.sleep(delay)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            s.settimeout(0.1)
            with _reconnect_lock:
                sock = s
                _reconnecting = False
            log.info(f"Reconnected to Arduino bridge at {HOST}:{PORT}")
            return
        except Exception as e:
            delay = min(delay * 2, 30.0)
            log.warning(f"Reconnect failed ({e}), retrying in {delay:.0f} s")


def is_connected():
    return sock is not None


def send(data: str):
    """Send a string command over the bridge."""
    global sock
    if sock:
        try:
            sock.sendall(data.encode('utf-8'))
        except Exception as e:
            log.error(f"Send failed: {e}")
            sock = None  # Mark as disconnected
            _start_reconnect_thread()  # Fix #4: trigger reconnect on send failure


_buffer = ""

def readline() -> str:
    """Read a line from the bridge. Returns empty string if no full line available."""
    global sock, _buffer
    if not sock:
        return ''

    try:
        data = sock.recv(1024).decode('utf-8', errors='ignore')
        if not data:
            log.error("Socket connection closed by remote host")
            sock = None
            _start_reconnect_thread()  # Fix #4: trigger reconnect on EOF
            return ''
        _buffer += data
    except socket.timeout:
        pass
    except Exception as e:
        log.error(f"Read failed: {e}")
        sock = None
        _start_reconnect_thread()  # Fix #4: trigger reconnect on read error
        return ''

    if '\n' in _buffer:
        line, _buffer = _buffer.split('\n', 1)
        return line.strip()

    return ''


def close():
    global sock
    if sock:
        try:
            sock.close()
        except Exception:
            pass
        sock = None