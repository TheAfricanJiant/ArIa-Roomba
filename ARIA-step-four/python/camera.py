# SPDX-License-Identifier: MPL-2.0
# camera.py — Camera module for ARIA-step-four
#
# The live stream and real-time object detection are handled entirely by the
# arduino:video_object_detection brick (served at http://board:4912/embed).
# This module just stores the latest detections and provides a snapshot helper
# for Telegram /photo and /detect commands.

import io
import logging
import threading

log = logging.getLogger(__name__)

# ── Shared state (populated by main.py via register_stream) ──────────────────
_detection_stream = None    # VideoObjectDetection instance
_latest_detections: dict = {}
_lock = threading.Lock()

STREAM_PORT = 4912
STREAM_PATH = "/embed"


def register_stream(stream):
    """Called from main.py after VideoObjectDetection is created."""
    global _detection_stream
    _detection_stream = stream


def on_detections(detections: dict):
    """Callback registered with detection_stream.on_detect_all()."""
    global _latest_detections
    with _lock:
        _latest_detections = dict(detections)


def get_latest_detections() -> dict:
    with _lock:
        return dict(_latest_detections)


def get_snapshot_jpeg() -> bytes | None:
    """
    Fetch one JPEG frame from the MJPEG stream served by the brick.
    Returns raw JPEG bytes, or None if unavailable.
    """
    try:
        import urllib.request
        # The brick serves an MJPEG stream at /embed; /snapshot returns a single JPEG
        url = f"http://localhost:{STREAM_PORT}/snapshot"
        with urllib.request.urlopen(url, timeout=3) as resp:
            return resp.read()
    except Exception as e:
        log.warning(f"Snapshot fetch failed ({e}). Trying MJPEG parse...")

    # Fallback: parse first JPEG from the MJPEG stream
    try:
        import urllib.request
        url = f"http://localhost:{STREAM_PORT}{STREAM_PATH}"
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=4) as resp:
            buf = b""
            while True:
                chunk = resp.read(4096)
                if not chunk:
                    break
                buf += chunk
                # JPEG starts with FF D8 and ends with FF D9
                start = buf.find(b'\xff\xd8')
                end   = buf.find(b'\xff\xd9', start)
                if start != -1 and end != -1:
                    return buf[start:end + 2]
    except Exception as e2:
        log.warning(f"MJPEG frame parse also failed: {e2}")

    return None
