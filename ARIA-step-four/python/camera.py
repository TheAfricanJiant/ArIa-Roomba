# camera.py for ARIA-step-four
#
# The arduino:video_object_detection brick serves frames embedded as
# data:image/jpeg;base64,... inside an <img> tag in the HTML at
# http://localhost:4912/embed
#
# We poll that page, extract the base64, and keep the latest JPEG in memory.

import io
import base64
import logging
import threading
import time
import urllib.request
import re

log = logging.getLogger(__name__)

STREAM_URL   = "http://localhost:4912/embed"
POLL_INTERVAL = 0.25   # seconds between frame fetches (~4 fps)

_latest_frame_jpeg = None
_latest_detections = {}
_frame_lock = threading.Lock()
_det_lock   = threading.Lock()
_running    = False


# ── Detection store ──────────────────────────────────────────────────────────
def on_detections(detections):
    global _latest_detections
    with _det_lock:
        _latest_detections = dict(detections)


def get_latest_detections():
    with _det_lock:
        return dict(_latest_detections)


# ── Extract JPEG from /embed HTML ────────────────────────────────────────────
_IMG_RE = re.compile(
    rb'<img[^>]+src=["\']data:image/jpeg;base64,([A-Za-z0-9+/=]+)["\']',
    re.IGNORECASE
)

def _fetch_frame():
    """Fetch /embed and extract the base64 JPEG from the <img> tag."""
    try:
        with urllib.request.urlopen(STREAM_URL, timeout=5) as resp:
            html = resp.read()
        m = _IMG_RE.search(html)
        if m:
            return base64.b64decode(m.group(1))
    except Exception as e:
        log.debug(f"Camera fetch error: {e}")
    return None


# ── Polling loop ─────────────────────────────────────────────────────────────
def _poll_loop():
    global _latest_frame_jpeg, _running
    _running = True
    log.info("Camera poller started — fetching from %s", STREAM_URL)
    consecutive_fails = 0
    while _running:
        frame = _fetch_frame()
        if frame:
            with _frame_lock:
                _latest_frame_jpeg = frame
            consecutive_fails = 0
        else:
            consecutive_fails += 1
            if consecutive_fails == 1:
                log.warning("Camera: no frame yet — waiting for brick to start...")
            elif consecutive_fails % 20 == 0:
                log.warning("Camera: still no frame after %d attempts", consecutive_fails)
        time.sleep(POLL_INTERVAL)


def start_frame_grabber():
    """Start the background polling thread."""
    t = threading.Thread(target=_poll_loop, daemon=True)
    t.start()


def get_snapshot_jpeg():
    """Return the latest captured JPEG bytes, or None if not ready."""
    with _frame_lock:
        return _latest_frame_jpeg


# ── GIF recording ────────────────────────────────────────────────────────────
def record_gif(duration_sec=5, fps=4):
    """Capture frames for duration_sec seconds and return animated GIF bytes."""
    from PIL import Image as _Img
    frames   = []
    interval = 1.0 / fps
    total    = duration_sec * fps
    log.info("Recording GIF: %ds @ %dfps (%d frames)", duration_sec, fps, total)
    for _ in range(total):
        frame = get_snapshot_jpeg()
        if frame:
            try:
                img = _Img.open(io.BytesIO(frame)).convert("P", palette=_Img.ADAPTIVE)
                frames.append(img)
            except Exception as e:
                log.warning("record_gif frame error: %s", e)
        time.sleep(interval)
    if not frames:
        log.warning("record_gif: no frames captured")
        return None
    buf = io.BytesIO()
    frames[0].save(
        buf, format="GIF", save_all=True,
        append_images=frames[1:], duration=int(1000 / fps), loop=0
    )
    buf.seek(0)
    log.info("record_gif: encoded %d frames, %d bytes", len(frames), buf.getbuffer().nbytes)
    return buf.getvalue()


# ── Compat stubs ─────────────────────────────────────────────────────────────
def register_stream(stream):
    pass   # no-op — kept for API compatibility with main.py
