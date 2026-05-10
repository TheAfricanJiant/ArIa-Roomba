# SPDX-License-Identifier: MPL-2.0
# camera.py — Live camera module for ARIA-step-four
# Uses OpenCV to stream frames, take snapshots, and run object detection.

import threading
import logging
import base64
import io
import time

log = logging.getLogger(__name__)

# ── Try to import OpenCV ──────────────────────────────────────────────────────
try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False
    log.warning("OpenCV (cv2) not installed. Camera disabled. Run: pip install opencv-python-headless")

_cap = None          # cv2.VideoCapture instance
_lock = threading.Lock()
_streaming = False
_latest_frame_b64 = None   # always-available last frame as base64 PNG

CAM_INDEX = 0        # Change if your USB camera is on a different index
STREAM_FPS = 8       # frames per second pushed over Socket.IO


def _open_camera():
    global _cap
    if not _CV2_AVAILABLE:
        return False
    with _lock:
        if _cap and _cap.isOpened():
            return True
        _cap = cv2.VideoCapture(CAM_INDEX)
        if not _cap.isOpened():
            log.warning(f"Camera index {CAM_INDEX} not found.")
            _cap = None
            return False
        _cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        _cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        log.info("Camera opened.")
        return True


def _frame_to_b64(frame) -> str:
    """Encode a cv2 BGR frame to base64 JPEG string."""
    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return base64.b64encode(buf).decode("utf-8")


def is_available() -> bool:
    return _CV2_AVAILABLE and _open_camera()


def capture_snapshot() -> bytes | None:
    """Grab one frame and return raw PNG bytes, or None on failure."""
    if not is_available():
        return None
    with _lock:
        ret, frame = _cap.read()
    if not ret:
        return None
    _, buf = cv2.imencode(".png", frame)
    return buf.tobytes()


def capture_snapshot_b64() -> str | None:
    """Grab one frame and return base64-encoded JPEG string."""
    if not is_available():
        return None
    with _lock:
        ret, frame = _cap.read()
    if not ret:
        return None
    return _frame_to_b64(frame)


def start_stream(ui, obj_detection=None):
    """Background thread: push JPEG frames over Socket.IO as camera_frame events."""
    global _streaming, _latest_frame_b64
    _streaming = True

    def _loop():
        global _streaming, _latest_frame_b64
        interval = 1.0 / STREAM_FPS
        while _streaming:
            if is_available():
                with _lock:
                    ret, frame = _cap.read()
                if ret:
                    b64 = _frame_to_b64(frame)
                    _latest_frame_b64 = b64
                    ui.send_message("camera_frame", {"frame": b64})
            time.sleep(interval)

    t = threading.Thread(target=_loop, daemon=True)
    t.start()
    log.info("Camera stream started.")


def stop_stream():
    global _streaming
    _streaming = False
    log.info("Camera stream stopped.")


def detect_on_snapshot(obj_detection) -> dict:
    """Capture a frame, run object detection, return result dict."""
    raw = capture_snapshot()
    if raw is None:
        return {"error": "Camera not available"}
    try:
        from PIL import Image
        pil_img = Image.open(io.BytesIO(raw))
        results = obj_detection.detect(pil_img, confidence=0.1)
        annotated = obj_detection.draw_bounding_boxes(pil_img, results)
        buf = io.BytesIO()
        annotated.save(buf, format="PNG")
        buf.seek(0)
        b64 = base64.b64encode(buf.getvalue()).decode("utf-8")
        count = len(results.get("detection", [])) if results else 0
        return {"success": True, "result_image": b64, "detection_count": count}
    except Exception as e:
        log.error(f"detect_on_snapshot error: {e}")
        return {"error": str(e)}
