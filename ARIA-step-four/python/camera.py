# SPDX-License-Identifier: MPL-2.0
# camera.py — Camera helper for ARIA-step-four
#
# The live stream is served by the arduino:video_object_detection brick at
# http://localhost:4912/embed (HTML wrapper) or /stream (raw MJPEG).
#
# This module runs a background thread that continuously reads JPEG frames
# from the raw stream so that /photo and /record always have a frame ready.

import io
import logging
import threading
import time
import urllib.request

log = logging.getLogger(__name__)

# ── Config ───────────────────────────────────────────────────────────────────
STREAM_PORT  = 4912
# Candidate paths for the raw MJPEG byte stream (tried in order)
MJPEG_PATHS  = ["/stream", "/", "/video", "/mjpeg", "/embed"]

# ── Shared state ─────────────────────────────────────────────────────────────
_latest_frame_jpeg: bytes | None = None   # always holds the most recent JPEG
_latest_detections: dict         = {}
_frame_lock   = threading.Lock()
_det_lock     = threading.Lock()
_grabber_running = False


# ── Detection store (called from main.py on_detect_all callback) ─────────────
def on_detections(detections: dict):
    global _latest_detections
    with _det_lock:
        _latest_detections = dict(detections)


def get_latest_detections() -> dict:
    with _det_lock:
        return dict(_latest_detections)


# ── Frame grabber ─────────────────────────────────────────────────────────────
def _parse_mjpeg_frame(stream) -> bytes | None:
    """Read bytes from an open MJPEG stream until we get one complete JPEG."""
    buf = b""
    while True:
        chunk = stream.read(4096)
        if not chunk:
            return None
        buf += chunk
        start = buf.find(b'\xff\xd8')      # JPEG SOI marker
        end   = buf.find(b'\xff\xd9', start + 2) if start != -1 else -1
        if start != -1 and end != -1:
            return buf[start: end + 2]
        # Keep buffer small — drop everything before a potential SOI
        if len(buf) > 1_000_000:
            buf = buf[-4096:]


def _try_open_stream():
    """Try each MJPEG path and return an open HTTP response, or None."""
    for path in MJPEG_PATHS:
        url = f"http://localhost:{STREAM_PORT}{path}"
        try:
            resp = urllib.request.urlopen(url, timeout=3)
            ct = resp.headers.get("Content-Type", "")
            # Accept both raw MJPEG and anything we can try to parse
            log.info(f"Camera: connected to {url}  Content-Type: {ct}")
            return resp
        except Exception as e:
            log.debug(f"Camera: {url} failed: {e}")
    return None


def _frame_grabber_loop():
    """Background daemon that continuously refreshes _latest_frame_jpeg."""
    global _latest_frame_jpeg, _grabber_running
    _grabber_running = True
    log.info("Camera frame grabber started.")

    while _grabber_running:
        stream = _try_open_stream()
        if stream is None:
            log.warning("Camera: MJPEG stream not available yet — retrying in 3s")
            time.sleep(3)
            continue
        try:
            while _grabber_running:
                frame = _parse_mjpeg_frame(stream)
                if frame is None:
                    break           # stream closed, reconnect
                with _frame_lock:
                    _latest_frame_jpeg = frame
        except Exception as e:
            log.warning(f"Camera: stream error ({e}), reconnecting...")
        finally:
            try: stream.close()
            except: pass
        time.sleep(1)   # brief pause before reconnect


def start_frame_grabber():
    """Start the background MJPEG frame-grabber thread (call once from main.py)."""
    t = threading.Thread(target=_frame_grabber_loop, daemon=True)
    t.start()


def get_snapshot_jpeg() -> bytes | None:
    """Return the most recently captured JPEG frame, or None if not available."""
    with _frame_lock:
        return _latest_frame_jpeg


# ── Record (animated GIF via Pillow) ─────────────────────────────────────────
def record_gif(duration_sec: int = 5, fps: int = 4) -> bytes | None:
    """
    Capture `duration_sec` seconds of frames at `fps` and encode as animated GIF.
    Returns raw GIF bytes, or None on failure.
    """
    interval = 1.0 / fps
    total    = duration_sec * fps
    frames   = []

    log.info(f"Recording GIF: {duration_sec}s @ {fps}fps ({total} frames)")
    for _ in range(total):
        frame = get_snapshot_jpeg()
        if frame:
            try:
                from PIL import Image as _Img
                img = _Img.open(io.BytesIO(frame)).convert("P", palette=_Img.ADAPTIVE)
                frames.append(img)
            except Exception as e:
                log.warning(f"record_gif frame error: {e}")
        time.sleep(interval)

    if not frames:
        log.warning("record_gif: no frames captured")
        return None

    buf = io.BytesIO()
    frames[0].save(
        buf, format="GIF",
        save_all=True,
        append_images=frames[1:],
        duration=int(1000 / fps),
        loop=0
    )
    buf.seek(0)
    log.info(f"record_gif: encoded {len(frames)} frames")
    return buf.getvalue()


# ── Register stream (kept for compatibility with main.py) ────────────────────
def register_stream(stream):
    """Called from main.py after VideoObjectDetection is created (no-op here)."""
    pass
