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


_buffer = ""

def readline() -> str:
    """Read a line from the bridge. Returns empty string if no full line is available."""
    global sock, _buffer
    if not sock:
        return ''
        
    try:
        # Try to read more data if available
        data = sock.recv(1024).decode('utf-8', errors='ignore')
        if not data:
            log.error("Socket connection closed by remote host")
            sock = None
            return ''
        _buffer += data
    except socket.timeout:
        pass
    except Exception as e:
        log.error(f"Read failed: {e}")
        sock = None
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