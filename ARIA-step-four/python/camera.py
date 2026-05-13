# camera.py for ARIA-step-four
#
# Env: ARIA_VIDEO_HTTP_HOST — comma-separated hosts to try first (e.g. 192.168.1.205).
#      If unset, guesses LAN IPv4 via default route, then 127.0.0.1 / localhost.
#
# Strategy (all stdlib, no external deps except optional OpenCV in grab_jpeg_v4l2):
#   1. HTTP probe: try every common path at port for raw JPEG/MJPEG
#   2. HTML parse: if /embed is HTML, extract <img src>, fetch/WebSocket URLs
#   3. WebSocket probe: try ws:// paths using raw socket + HTTP Upgrade
#   4. Once frames are found via any method, store in _latest_frame_jpeg
#   5. Expose get_snapshot_jpeg() / record_gif() for Telegram + Web UI

import glob, io, base64, hashlib, logging, os, re, socket, struct, threading, time, urllib.request

log = logging.getLogger(__name__)

# VideoObjectDetection embed/MJPEG HTTP (Arduino App Lab default is 4912).
PORT = int(os.environ.get("ARIA_VIDEO_HTTP_PORT", "4912"))
# Seconds to wait before first probe: grabber thread starts before App.run() finishes
# starting V4LCamera + the embed server; probing too early yields connection refused.
HTTP_WARMUP_SEC = float(os.environ.get("ARIA_VIDEO_HTTP_WARMUP_SEC", "14"))
RETRY_SEC = float(os.environ.get("ARIA_CAMERA_RETRY_SEC", "2.5"))
# Direct snapshot URL override — if set, skips all discovery and polls this URL directly.
SNAPSHOT_URL = (os.environ.get("ARIA_CAMERA_SNAPSHOT_URL") or "").strip()
HTTP_PATHS = ["/stream", "/embed", "/", "/snapshot", "/video", "/mjpeg", "/frame", "/cam"]
WS_PATHS   = ["/ws", "/stream", "/", "/video", "/cam"]
# Max full discovery cycles before switching to slow (60 s) retry.
MAX_RETRIES = int(os.environ.get("ARIA_CAMERA_MAX_RETRIES", "4"))

_latest_frame_jpeg = None
_latest_detections = {}
_frame_lock = threading.Lock()
_det_lock   = threading.Lock()
_active_url = None   # the URL that's actually giving us frames
_ws_socket  = None   # active WebSocket socket, if any
_retry_count = 0      # number of full discovery cycles attempted


def _outbound_ipv4():
    """Default-route IPv4 (often 192.168.x.x) without sending packets. Fails offline."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.settimeout(0.35)
        s.connect(("198.51.100.1", 80))  # TEST-NET-2; resolves local bind only
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def video_http_hosts():
    """Hosts to try for VideoObjectDetection HTTP/WS (:PORT).

    Some App Lab / UNO Q setups bind MJPEG/embed on the LAN address only loopback misses.
    Override with comma-separated ``ARIA_VIDEO_HTTP_HOST`` (e.g. ``192.168.1.205``).
    """
    seen = set()
    out = []
    raw = (os.environ.get("ARIA_VIDEO_HTTP_HOST") or "").strip()
    if raw:
        for part in raw.split(","):
            h = part.strip()
            if h and h not in seen:
                seen.add(h)
                out.append(h)
    lan = _outbound_ipv4()
    if lan and lan not in seen:
        seen.add(lan)
        out.append(lan)
        # Infer Docker host gateway from container's LAN IP (replace last octet with .1)
        parts = lan.rsplit(".", 1)
        if len(parts) == 2:
            gw = parts[0] + ".1"
            if gw not in seen:
                seen.add(gw)
                out.append(gw)
    for h in ("host.docker.internal", "127.0.0.1", "localhost"):
        if h not in seen:
            seen.add(h)
            out.append(h)
    return out


# ── Detection store ───────────────────────────────────────────────────────────
def on_detections(detections):
    global _latest_detections
    with _det_lock:
        _latest_detections = dict(detections)

def get_latest_detections():
    with _det_lock:
        return dict(_latest_detections)


# ── Store a raw JPEG frame ────────────────────────────────────────────────────
def _store_frame(data: bytes):
    global _latest_frame_jpeg
    soi = data.find(b"\xff\xd8")
    eoi = data.rfind(b"\xff\xd9")
    if soi >= 0 and eoi > soi:
        with _frame_lock:
            _latest_frame_jpeg = data[soi:eoi+2]
        return True
    return False


# ── HTTP probe: check if a URL returns raw JPEG / MJPEG ──────────────────────
def _http_probe(path):
    for host in video_http_hosts():
        url = f"http://{host}:{PORT}{path}"
        try:
            resp = urllib.request.urlopen(url, timeout=4)
            ct   = resp.headers.get("Content-Type", "")
            first = resp.read(65536)   # read up to 64 KB
            log.info(f"HTTP {url}: ct={ct!r} len={len(first)}")

            # Case 1: raw JPEG
            if b"\xff\xd8" in first:
                if b"<html" not in first[:200].lower():
                    log.info(f"HTTP: Found JPEG bytes at {url}!")
                    _store_frame(first)
                    return ("jpeg", url, resp, first)

            # Case 2: MJPEG multipart stream
            if "multipart" in ct.lower():
                log.info(f"HTTP: MJPEG multipart stream at {url}!")
                return ("mjpeg", url, resp, first)

            # Case 3: HTML — extract URLs for WebSocket/fetch probing
            if b"<html" in first[:200].lower() or "html" in ct.lower():
                ws_urls    = re.findall(rb'["\']?(ws[s]?://[^\'">\s]+)', first)
                fetch_urls = re.findall(rb"fetch\(['\"]([^'\"]+)['\"]", first)
                img_srcs   = re.findall(rb'src=["\']([^"\']{2,})["\']', first)
                log.info(f"HTML {url}: ws={ws_urls} fetch={fetch_urls} srcs={img_srcs[:5]}")
                resp.close()

                # Case 4: Relative <img src="..."> on embed page (Brick often serves stream here)
                for m in img_srcs[:12]:
                    sub = m.decode(errors="replace").strip()
                    if not sub or sub.startswith("data:") or sub.startswith("blob:"):
                        continue
                    tail = sub if sub.startswith("/") else "/" + sub
                    suburl = f"http://{host}:{PORT}{tail}"
                    try:
                        sr = urllib.request.urlopen(suburl, timeout=4)
                        ct2 = (sr.headers.get("Content-Type") or "").lower()
                        chunk = sr.read(524288)
                        sr.close()
                        if "multipart" in ct2:
                            rr = urllib.request.urlopen(suburl, timeout=4)
                            return ("mjpeg", suburl, rr, chunk)
                        if b"\xff\xd8" in chunk and b"<html" not in chunk[:200].lower():
                            log.info(f"Camera: JPEG from embedded img src {suburl}")
                            _store_frame(chunk)
                            return ("jpeg", suburl, None, chunk)
                    except Exception as e2:
                        log.debug("embedded img probe %s: %s", suburl, e2)
                return ("html", url, None, {"ws": ws_urls, "fetch": fetch_urls, "srcs": img_srcs})

            resp.close()
        except Exception as e:
            log.debug(f"HTTP {url}: {type(e).__name__}: {e}")
    return None


# ── WebSocket probe: try ws:// using raw socket + HTTP Upgrade ────────────────
def _ws_probe(path):
    raw_key = base64.b64encode(b"ARIACameraProbe1").decode()
    for host in video_http_hosts():
        url = f"ws://{host}:{PORT}{path}"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(4)
            s.connect((host, PORT))
            handshake = (
                f"GET {path} HTTP/1.1\r\n"
                f"Host: {host}:{PORT}\r\n"
                f"Upgrade: websocket\r\n"
                f"Connection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {raw_key}\r\n"
                f"Sec-WebSocket-Version: 13\r\n"
                f"\r\n"
            )
            s.sendall(handshake.encode())
            response = s.recv(4096).decode(errors="replace")
            if "101" in response:
                log.info("WS: Connected at %s", url)
                return s
            log.debug("WS %s: no 101, got: %s", url, response[:200])
            s.close()
        except Exception as e:
            log.debug("WS %s: %s", url, e)
    return None


# ── Read one WebSocket frame (text or binary) ─────────────────────────────────
def _ws_read_frame(s):
    """Read one WebSocket data frame. Returns (opcode, payload_bytes)."""
    try:
        header = _ws_recv_exact(s, 2)
        if not header: return None, None
        opcode  = header[0] & 0x0F
        masked  = (header[1] & 0x80) != 0
        length  = header[1] & 0x7F
        if length == 126:
            ext = _ws_recv_exact(s, 2)
            length = struct.unpack(">H", ext)[0]
        elif length == 127:
            ext = _ws_recv_exact(s, 8)
            length = struct.unpack(">Q", ext)[0]
        mask = _ws_recv_exact(s, 4) if masked else b"\x00\x00\x00\x00"
        payload = _ws_recv_exact(s, length)
        if masked:
            payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        return opcode, payload
    except Exception as e:
        log.debug(f"WS read error: {e}")
        return None, None

def _ws_recv_exact(s, n):
    buf = b""
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk: return None
        buf += chunk
    return buf


# ── MJPEG stream reader ───────────────────────────────────────────────────────
def _read_mjpeg(resp, seed_data=b""):
    """Read JPEG frames from an open MJPEG response. Yields jpeg bytes."""
    buf = seed_data
    while True:
        try:
            chunk = resp.read(8192)
            if not chunk: break
            buf += chunk
        except Exception:
            break
        while True:
            soi = buf.find(b"\xff\xd8")
            if soi < 0:
                buf = buf[-1024:] if len(buf) > 1024 else b""
                break
            eoi = buf.find(b"\xff\xd9", soi + 2)
            if eoi < 0:
                if len(buf) > 2 * 1024 * 1024:  # 2 MB safety cap
                    buf = buf[soi:]
                break
            yield buf[soi:eoi+2]
            buf = buf[eoi+2:]


# ── Main discovery + streaming loop ──────────────────────────────────────────
def _main_loop():
    global _active_url, _ws_socket, _retry_count
    if HTTP_WARMUP_SEC > 0:
        log.info(
            "Camera: waiting %.1fs for VideoObjectDetection HTTP on :%d (Brick starts after App.run).",
            HTTP_WARMUP_SEC,
            PORT,
        )
        time.sleep(HTTP_WARMUP_SEC)

    # ── Direct snapshot URL override ─────────────────────────────────────────
    if SNAPSHOT_URL:
        log.info("Camera: using direct snapshot URL: %s", SNAPSHOT_URL)
        try:
            with urllib.request.urlopen(SNAPSHOT_URL, timeout=5) as r:
                data = r.read()
            if b"\xff\xd8" in data:
                _store_frame(data)
                log.info("Camera: got JPEG from snapshot URL")
            else:
                log.warning("Camera: snapshot URL did not return JPEG (%d bytes)", len(data))
        except Exception as e:
            log.warning("Camera: snapshot URL failed: %s", e)
        # Poll it at 2 fps regardless
        _http_poll_loop(SNAPSHOT_URL)
        return

    # ── Give up after MAX_RETRIES and switch to slow poll ────────────────────
    _retry_count += 1
    if _retry_count > MAX_RETRIES:
        log.warning(
            "Camera: no frame source after %d attempts. "
            "Set ARIA_VIDEO_HTTP_HOST=<LAN_IP> (your host IP, e.g. 192.168.1.205) "
            "or ARIA_CAMERA_SNAPSHOT_URL for a direct JPEG URL. "
            "Checking again in 60s.",
            MAX_RETRIES,
        )
        time.sleep(60)
        _main_loop()
        return

    hosts = video_http_hosts()
    log.info("Camera: starting discovery on port %d (HTTP hosts: %s)", PORT, ", ".join(hosts))
    found_html_ws = []

    # Phase 1: HTTP probe all paths
    for path in HTTP_PATHS:
        kind, url, resp, extra = _http_probe(path) or (None, None, None, None)
        if kind == "jpeg":
            # Single JPEG snapshot — poll this path
            _active_url = url
            log.info(f"Camera: polling JPEG snapshots from {url}")
            _http_poll_loop(url)
            return
        elif kind == "mjpeg":
            _active_url = url
            log.info(f"Camera: reading MJPEG stream from {url}")
            for frame in _read_mjpeg(resp, extra):
                _store_frame(frame)
            return   # stream ended
        elif kind == "html" and extra:
            for ws_url in extra.get("ws", []):
                found_html_ws.append(ws_url.decode(errors="replace"))
            for src in extra.get("srcs", []):
                src_s = src.decode(errors="replace")
                if src_s.startswith("/") or src_s.startswith("http"):
                    found_html_ws.append(src_s)

    # Phase 2: WebSocket probe
    ws_candidates = WS_PATHS[:]
    for ws_url in found_html_ws:
        path_part = ws_url.split(":")[-1] if ":" in ws_url else ws_url
        path_part = "/" + path_part.lstrip("/")
        if path_part not in ws_candidates:
            ws_candidates.insert(0, path_part)

    for path in ws_candidates:
        s = _ws_probe(path)
        if s:
            try:
                ph = s.getpeername()[0]
            except Exception:
                ph = ""
            _active_url = f"ws://{ph}:{PORT}{path}" if ph else f"ws://*:{PORT}{path}"
            _ws_socket = s
            log.info(f"Camera: reading WebSocket frames from {_active_url}")
            _ws_loop(s)
            return

    log.warning(
        "Camera: no frame source on :%d (connection refused is normal until embed is up). Retrying in %.1fs.",
        PORT,
        RETRY_SEC,
    )
    time.sleep(RETRY_SEC)
    _main_loop()   # retry


def _http_poll_loop(url):
    """Poll a single JPEG URL at 4fps."""
    while True:
        try:
            with urllib.request.urlopen(url, timeout=4) as resp:
                data = resp.read()
            _store_frame(data)
        except Exception as e:
            log.debug(f"Poll {url}: {e}")
        time.sleep(0.25)


def _ws_loop(s):
    """Receive WebSocket frames and store JPEG data."""
    while True:
        opcode, payload = _ws_read_frame(s)
        if opcode is None:
            log.warning("Camera: WebSocket closed, reconnecting...")
            s.close()
            time.sleep(2)
            _main_loop()
            return
        if opcode == 0x2:  # binary
            if payload and b"\xff\xd8" in payload:
                _store_frame(payload)
        elif opcode == 0x1:  # text — might be base64
            if payload:
                try:
                    data = base64.b64decode(payload)
                    _store_frame(data)
                except Exception:
                    pass
        elif opcode == 0x8:  # close
            log.warning("Camera: WS close frame received")
            s.close()
            time.sleep(2)
            _main_loop()
            return


# ── Public API ────────────────────────────────────────────────────────────────
def start_frame_grabber():
    t = threading.Thread(target=_main_loop, daemon=True)
    t.start()

def _v4l_candidates():
    """Paths and indices to try for a USB UVC device on Linux."""
    out = []
    envp = os.environ.get("ARIA_CAMERA_DEVICE", "").strip()
    if envp:
        out.append(envp)
    try:
        devs = sorted(glob.glob("/dev/video*"))
        out.extend(d for d in devs if os.path.exists(d))
    except Exception:
        pass
    out.extend(range(0, 8))
    seen, uniq = set(), []
    for d in out:
        key = d
        if key in seen:
            continue
        seen.add(key)
        uniq.append(d)
    return uniq


def grab_jpeg_v4l2():
    """Direct USB webcam grab when VideoObjectDetection Brick HTTP (:4912) is unavailable.
    Tries env ARIA_CAMERA_DEVICE, each /dev/video*, then indices 0..7 with V4L2 then CAP_ANY."""
    os.environ.setdefault("OPENCV_LOG_LEVEL", "SILENT")
    try:
        import cv2  # noqa: PLC0415
        try:
            cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
        except Exception:
            pass
    except ImportError:
        log.debug("OpenCV not installed — no V4L2 fallback.")
        return None

    apis = (getattr(cv2, "CAP_V4L2", 200), getattr(cv2, "CAP_ANY", 0))

    for spec in _v4l_candidates():
        cap = None
        try:
            for api in apis:
                cap = cv2.VideoCapture(spec, api)
                if cap.isOpened():
                    break
                cap.release()
                cap = None
            if cap is None or not cap.isOpened():
                continue
            ok, frm = cap.read()
            if ok and frm is not None and getattr(frm, "size", 0) > 0:
                log.info("OpenCV grabbed JPEG via %s", spec)
                enc_ok, enc = cv2.imencode(".jpg", frm, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
                if enc_ok:
                    return bytes(enc)
        except Exception as e:
            log.debug("OpenCV try %s: %s", spec, e)
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

    devs = sorted(glob.glob("/dev/video*"))
    log.warning(
        "OpenCV/V4L2: cannot read USB/UVC camera. Found: %s. "
        ":4912 must be reachable for the Brick, or unset OpenCV grab and fix hub/Network Mode. "
        "Try ARIA_CAMERA_DEVICE=/dev/video1 if multiple nodes exist.",
        ", ".join(devs) if devs else "(no /dev/video* — wiring, driver, or exclusive Brick lock)",
    )
    return None

def get_snapshot_jpeg():
    with _frame_lock:
        cached = _latest_frame_jpeg
    if cached:
        return cached
    v = grab_jpeg_v4l2()
    return v

def record_gif(duration_sec=5, fps=4):
    from PIL import Image as _Img
    frames, interval = [], 1.0 / fps
    for _ in range(duration_sec * fps):
        f = get_snapshot_jpeg()
        if f:
            try:
                frames.append(_Img.open(io.BytesIO(f)).convert("P", palette=_Img.ADAPTIVE))
            except Exception:
                pass
        time.sleep(interval)
    if not frames:
        return None
    buf = io.BytesIO()
    frames[0].save(buf, format="GIF", save_all=True,
                   append_images=frames[1:], duration=int(1000/fps), loop=0)
    buf.seek(0)
    return buf.getvalue()

def register_stream(stream):
    pass  # compat stub
