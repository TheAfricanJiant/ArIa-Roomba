import socket
import logging

log = logging.getLogger(__name__)

HOST = '192.168.1.205'
PORT = 5000

sock = None


def connect():
    """Connect to the socat TCP-to-serial bridge."""
    global sock
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, PORT))
        sock.settimeout(0.1)
        log.info(f"Connected to Arduino via socat bridge at {HOST}:{PORT}")
    except Exception as e:
        log.warning(f"Could not connect to socat bridge: {e}. Falling back to dummy data.")
        sock = None


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


def readline() -> str:
    """Read a line from the bridge. Returns empty string on timeout or error."""
    global sock
    if sock:
        try:
            data = sock.recv(256).decode('utf-8', errors='ignore')
            return data.strip()
        except socket.timeout:
            return ''
        except Exception as e:
            log.error(f"Read failed: {e}")
            sock = None
    return ''


def close():
    global sock
    if sock:
        try:
            sock.close()
        except Exception:
            pass
        sock = None