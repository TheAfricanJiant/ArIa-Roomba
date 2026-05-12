// ── Element references ──────────────────────────────────────────────────────
const powerBtn    = document.getElementById('motor-power-btn');
const powerText   = document.getElementById('power-text');
const speedSlider = document.getElementById('speed-slider');
const speedVal    = document.getElementById('speed-val');

const vacuumToggleBtn = document.getElementById('vacuum-toggle-btn');
const vacuumSlider    = document.getElementById('vacuum-slider');
const vacuumVal       = document.getElementById('vacuum-val');

const brushToggleBtn  = document.getElementById('brush-toggle-btn');
const brushSlider     = document.getElementById('brush-slider');
const brushVal        = document.getElementById('brush-val');

const driveForwardBtn  = document.getElementById('drive-forward-btn');
const driveBackwardBtn = document.getElementById('drive-backward-btn');
const driveLeftBtn     = document.getElementById('drive-left-btn');
const driveRightBtn    = document.getElementById('drive-right-btn');
const driveStopBtn     = document.getElementById('drive-stop-btn');
const homeBtn          = document.getElementById('home-btn');
const autoCleanBtn     = document.getElementById('auto-clean-btn');

const encL     = document.getElementById('enc-l');
const encR     = document.getElementById('enc-r');
const imuAccel = document.getElementById('imu-accel');
const imuGyro  = document.getElementById('imu-gyro');
const usLeft   = document.getElementById('us-left');
const usFront  = document.getElementById('us-front');
const usRight  = document.getElementById('us-right');

const ekfX     = document.getElementById('ekf-x');
const ekfY     = document.getElementById('ekf-y');
const ekfTheta = document.getElementById('ekf-theta');
const ekfDist  = document.getElementById('ekf-dist');

const sysCpu   = document.getElementById('sys-cpu');
const sysMem   = document.getElementById('sys-mem');

const mapCanvas      = document.getElementById('map-canvas');
const mapCtx         = mapCanvas.getContext('2d');
const coveragePct    = document.getElementById('coverage-pct');
const setGoalBtn     = document.getElementById('set-goal-btn');
const drawZoneBtn    = document.getElementById('draw-zone-btn');
const startNavBtn    = document.getElementById('start-nav-btn');
const saveRoutineBtn = document.getElementById('save-routine-btn');
const clearGoalBtn   = document.getElementById('clear-goal-btn');
const routinesSelect = document.getElementById('routines-select');

const navCanvas = document.getElementById('nav-canvas');
const navCtx    = navCanvas ? navCanvas.getContext('2d') : null;

let _settingGoal = false;
let _drawingZone = false;
let _waypoints = []; // array of {x, y}
let _zoneStart = null; // {x, y}
let _zone = null; // {x_min, y_min, x_max, y_max}
let _routinesData = [];

// â”€â”€ Cell state constants (match Python OccupancyGrid uint8 values) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
const CELL_UNKNOWN  = 0;
const CELL_FREE     = 1;
const CELL_CLEANED  = 2;
const CELL_WALL     = 3;
const CELL_OBSTACLE = 4;

// â”€â”€ Socket â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
const socket = io(`http://${window.location.host}`);

/** Binary to base64 in chunks (avoids stack limits on huge frames). */
function u8ToBase64Chunked(u8) {
    const cs = 0x8000;
    let bin = '';
    for (let i = 0; i < u8.length; i += cs) {
        bin += String.fromCharCode.apply(null, u8.subarray(i, Math.min(i + cs, u8.length)));
    }
    return btoa(bin);
}

/** Find first JPEG SOI-EOI in buffer (MJPEG multipart first frame). */
function extractFirstJpegBytes(arrayBuffer, maxScan) {
    const cap = typeof maxScan === 'number' ? maxScan : 2 * 1024 * 1024;
    const u = new Uint8Array(arrayBuffer.byteLength > cap ? arrayBuffer.slice(0, cap) : arrayBuffer);
    let soi = -1;
    for (let i = 0; i < u.length - 1; i++) {
        if (u[i] === 0xff && u[i + 1] === 0xd8) { soi = i; break; }
    }
    if (soi < 0) return null;
    for (let j = soi + 2; j < u.length - 1; j++) {
        if (u[j] === 0xff && u[j + 1] === 0xd9) return u.subarray(soi, j + 2);
    }
    return null;
}

/** Embed/MJPEG HTTP (App Lab default 4912; keep in sync with ARIA_VIDEO_HTTP_PORT in Python env). */
const VIDEO_HTTP_PORT = (typeof window.ARIA_VIDEO_HTTP_PORT !== 'undefined' && window.ARIA_VIDEO_HTTP_PORT) || 4912;

/** Grab one JPEG from VideoObjectDetection HTTP endpoint. */
async function fetchOneJpegFromBrick4912(host) {
    const paths = ['/snapshot', '/stream', '/', '/video', '/frame', '/mjpeg', '/cam', '/embed'];
    for (let i = 0; i < paths.length; i++) {
        const controller = new AbortController();
        const tid = setTimeout(() => controller.abort(), 4000);
        try {
            const r = await fetch(`http://${host}:${VIDEO_HTTP_PORT}${paths[i]}`, {
                signal: controller.signal,
                mode: 'cors',
                cache: 'no-store',
            });
            const jpeg = extractFirstJpegBytes(await r.arrayBuffer());
            if (jpeg && jpeg.byteLength > 500) return u8ToBase64Chunked(jpeg);
        } catch (_) { /* try next */ } finally {
            clearTimeout(tid);
        }
    }
    return null;
}

socket.on('request_frame', async () => {
    const host = window.location.hostname;
    let b64 = null;
    try {
        b64 = await fetchOneJpegFromBrick4912(host);
    } catch (e) {
        console.warn('request_frame', e);
    }
    socket.emit('frame_from_browser', { image: b64 || null });
});

// â”€â”€ Tab switching â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
function switchTab(name) {
    document.querySelectorAll('.tab-panel').forEach(p => p.classList.add('hidden'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.getElementById(`tab-${name}`).classList.remove('hidden');
    document.getElementById(`tab-${name}-btn`).classList.add('active');

    const host = window.location.hostname;
    const camUrl = `http://${host}:${VIDEO_HTTP_PORT}/embed`;
    
    if (name === 'camera') {
        const camIframe = document.getElementById('cam-iframe');
        if (camIframe && !camIframe.src) {
            camIframe.src = camUrl;
            camIframe.style.display = 'block';
            const camPlaceholder = document.getElementById('cam-placeholder');
            if (camPlaceholder) camPlaceholder.style.display = 'none';
        }
    } else if (name === 'nav') {
        const navCamIframe = document.getElementById('nav-cam-iframe');
        if (navCamIframe && !navCamIframe.src) {
            navCamIframe.src = camUrl;
            navCamIframe.style.display = 'block';
            const navPlaceholder = document.getElementById('nav-cam-placeholder');
            if (navPlaceholder) navPlaceholder.style.display = 'none';
        }
    }
}

// â”€â”€ Socket event wiring â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
document.addEventListener('DOMContentLoaded', () => {
    socket.on('connect', () => { socket.emit('get_initial_state', {}); });
    socket.on('state_update',     (s) => updateUI(s));
    socket.on('telemetry_update', (d) => updateTelemetry(d));
    socket.on('ekf_update',       (e) => updateEKF(e));
    socket.on('map_update',       (m) => renderMap(m));
    socket.on('obstacle_map_update', (m) => renderObstacleMap(m));
    socket.on('us_update',        (u) => updateUltrasonics(u));
    socket.on('cpu_usage',        (d) => { if (sysCpu) sysCpu.textContent = d.value.toFixed(1) + '%'; });
    socket.on('memory_usage',     (d) => { if (sysMem) sysMem.textContent = d.value.toFixed(1) + '%'; });
    socket.on('clean_state',      (s) => {
        const badge = document.getElementById('clean-state-badge');
        if (badge) badge.textContent = `State: ${s.state}`;
    });
    
    socket.on('path_update', (path) => {
        _waypoints = path;
        if (_lastMapData) _drawMap(_lastMapData);
    });
    
    socket.on('routines_list', (list) => {
        _routinesData = list;
        routinesSelect.innerHTML = '<option value="">-- Load Saved Routine --</option>';
        list.forEach((r, i) => {
            const opt = document.createElement('option');
            opt.value = i;
            opt.textContent = `${r.name} [${r.type === 'zone' ? 'Zone' : 'Path'}]`;
            routinesSelect.appendChild(opt);
        });
    });

    socket.on('disconnect', () => {
        const err = document.getElementById('error-container');
        if (err) { err.textContent = 'Connection lost.'; err.style.display = 'block'; }
    });

    powerBtn.addEventListener('click', () => socket.emit('toggle_power', {}));
    speedSlider.addEventListener('input',  (e) => { speedVal.textContent = e.target.value; });
    speedSlider.addEventListener('change', (e) => socket.emit('set_speed', { speed: parseInt(e.target.value) }));

    function makeThrottledEmitter(eventName, key, delayMs = 50) {
        let timer = null;
        let latest = null;
        return (value) => {
            latest = value;
            if (timer) return;
            socket.emit(eventName, { [key]: latest });
            timer = setTimeout(() => {
                timer = null;
                if (latest !== value) {
                    socket.emit(eventName, { [key]: latest });
                }
            }, delayMs);
        };
    }

    const emitVacuum = makeThrottledEmitter('set_vacuum', 'pwm');
    const emitBrush = makeThrottledEmitter('set_brush', 'speed');

    if (vacuumSlider) {
        vacuumSlider.addEventListener('input', (e) => {
            const pwm = parseInt(e.target.value) || 0;
            vacuumVal.textContent = pwm.toString();
            emitVacuum(pwm);
        });
        vacuumSlider.addEventListener('change', (e) => emitVacuum(parseInt(e.target.value) || 0));
    }
    if (vacuumToggleBtn) {
        vacuumToggleBtn.addEventListener('click', () => {
            const cur = parseInt(vacuumSlider ? vacuumSlider.value : '0') || 0;
            const next = cur > 0 ? 0 : 255;
            if (vacuumSlider) { vacuumSlider.value = next; }
            if (vacuumVal) { vacuumVal.textContent = next.toString(); }
            emitVacuum(next);
        });
    }

    // ── Brush slider & toggle ──
    if (brushSlider) {
        brushSlider.addEventListener('input', (e) => {
            const speed = parseInt(e.target.value) || 0;
            if (brushVal) brushVal.textContent = speed.toString();
            emitBrush(speed);
        });
        brushSlider.addEventListener('change', (e) => emitBrush(parseInt(e.target.value) || 0));
    }
    if (brushToggleBtn) {
        brushToggleBtn.addEventListener('click', () => {
            const cur = parseInt(brushSlider ? brushSlider.value : '0') || 0;
            const next = cur !== 0 ? 0 : 80;
            if (brushSlider) brushSlider.value = next;
            if (brushVal) brushVal.textContent = next;
            emitBrush(next);
            brushToggleBtn.textContent = next !== 0 ? '🪥 Brush ON' : '🪥 Brush OFF';
            brushToggleBtn.style.background = next !== 0 ? '#4CAF50' : '#5d4037';
        });
    }

    // ── Momentary drive buttons (hold to move, release to stop) ──
    function sendDrive(action) {
        const spd = parseInt(speedSlider ? speedSlider.value : '160') || 160;
        socket.emit('manual_drive', { action, speed: spd });
    }
    function addMomentary(btn, dir) {
        if (!btn) return;
        let activePointer = null;
        const stop = () => {
            if (activePointer === null) return;
            activePointer = null;
            sendDrive('stop');
        };
        btn.addEventListener('pointerdown', (e) => {
            e.preventDefault();
            activePointer = e.pointerId;
            btn.setPointerCapture(e.pointerId);
            sendDrive(dir);
        });
        btn.addEventListener('pointerup', stop);
        btn.addEventListener('pointercancel', stop);
        btn.addEventListener('lostpointercapture', stop);
    }
    addMomentary(driveForwardBtn,  'forward');
    addMomentary(driveBackwardBtn, 'backward');
    addMomentary(driveLeftBtn,     'left');
    addMomentary(driveRightBtn,    'right');
    if (driveStopBtn) driveStopBtn.addEventListener('click', () => sendDrive('stop'));

    // ── Home / Reset Origin button ──
    if (homeBtn) {
        homeBtn.addEventListener('click', () => {
            if (confirm('🏠 Set current position as Home (resets encoders and map)?')) {
                socket.emit('reset_encoders', {});
                _trajectory = [];
                _lastNavData = null;
                if (navCtx) { navCtx.clearRect(0, 0, navCanvas.width, navCanvas.height); }
            }
        });
    }

    // ── Auto-clean toggle ──
    if (autoCleanBtn) {
        autoCleanBtn.addEventListener('click', () => {
            socket.emit('toggle_auto_clean', {});
        });
    }

    // -- Trajectory-Only toggle (Map tab) --
    const trajOnlyBtn = document.getElementById('traj-only-btn');
    if (trajOnlyBtn) {
        trajOnlyBtn.addEventListener('click', () => {
            _trajectoryOnly = !_trajectoryOnly;
            trajOnlyBtn.style.background = _trajectoryOnly ? '#008184' : '#2C353A';
            trajOnlyBtn.style.color      = _trajectoryOnly ? '#ffffff' : '#90CAF9';
            trajOnlyBtn.textContent      = _trajectoryOnly ? '\u2705 Trajectory Only' : '\uD83D\uDDFA\uFE0F Trajectory Only';
            if (_lastMapData) _drawMap(_lastMapData);
        });
    }

    setGoalBtn.addEventListener('click', () => {
        _settingGoal = true;
        _drawingZone = false;
        mapCanvas.style.cursor = 'crosshair';
        setGoalBtn.style.background = '#005e60';
        drawZoneBtn.style.background = '#008184';
        setGoalBtn.textContent = 'Click to add points...';
        drawZoneBtn.textContent = 'ðŸ”² Draw Zone';
        _zoneStart = null;
    });

    drawZoneBtn.addEventListener('click', () => {
        _drawingZone = true;
        _settingGoal = false;
        mapCanvas.style.cursor = 'crosshair';
        drawZoneBtn.style.background = '#005e60';
        setGoalBtn.style.background = '#008184';
        drawZoneBtn.textContent = 'Click 2 corners...';
        setGoalBtn.textContent = 'ðŸ“ Add Waypoints';
        _zoneStart = null;
    });

    startNavBtn.addEventListener('click', () => {
        if (_waypoints.length > 0) {
            socket.emit('set_path', { path: _waypoints });
        } else if (_zone) {
            socket.emit('clean_zone', { zone: _zone });
        }
        _resetUI();
        startNavBtn.style.display = 'none';
        saveRoutineBtn.style.display = 'none';
    });

    saveRoutineBtn.addEventListener('click', () => {
        const name = prompt("Enter a name for this routine (e.g. 'Living Room'):");
        if (name) {
            if (_zone) {
                socket.emit('save_routine', { name, type: 'zone', data: _zone });
            } else if (_waypoints.length > 0) {
                socket.emit('save_routine', { name, type: 'waypoints', data: _waypoints });
            }
        }
    });

    clearGoalBtn.addEventListener('click', () => {
        socket.emit('clear_goal', {});
        _waypoints = [];
        _zone = null;
        _zoneStart = null;
        _resetUI();
        clearGoalBtn.style.display = 'none';
        startNavBtn.style.display = 'none';
        saveRoutineBtn.style.display = 'none';
        if (_lastMapData) _drawMap(_lastMapData);
    });

    routinesSelect.addEventListener('change', (e) => {
        const idx = e.target.value;
        if (idx === "") return;
        const routine = _routinesData[idx];
        if (routine.type === 'zone') {
            _zone = routine.data;
            _waypoints = [];
        } else {
            _waypoints = routine.data;
            _zone = null;
        }
        startNavBtn.style.display = 'inline-block';
        clearGoalBtn.style.display = 'inline-block';
        saveRoutineBtn.style.display = 'none'; // already saved
        _resetUI();
        if (_lastMapData) _drawMap(_lastMapData);
        routinesSelect.value = ""; // reset dropdown
    });

    function _resetUI() {
        _settingGoal = false;
        _drawingZone = false;
        mapCanvas.style.cursor = 'default';
        setGoalBtn.style.background = '#008184';
        drawZoneBtn.style.background = '#008184';
        setGoalBtn.textContent = 'ðŸ“ Add Waypoints';
        drawZoneBtn.textContent = 'ðŸ”² Draw Zone';
    }

    // Map click handler
    mapCanvas.addEventListener('click', (e) => {
        if (!_settingGoal && !_drawingZone) return;
        
        const rect = mapCanvas.getBoundingClientRect();
        const scaleX = mapCanvas.width / rect.width;
        const scaleY = mapCanvas.height / rect.height;
        const clickX = (e.clientX - rect.left) * scaleX;
        const clickY = (e.clientY - rect.top) * scaleY;

        const cx = _latestPose ? _latestPose.x_cm : 0;
        const cy = _latestPose ? _latestPose.y_cm : 0;
        const worldX = cx + (clickX - MAP_PX / 2) / PX_PER_CM;
        const worldY = cy - (clickY - MAP_PX / 2) / PX_PER_CM;

        if (_settingGoal) {
            _waypoints.push({ x: worldX, y: worldY });
            startNavBtn.style.display = 'inline-block';
            saveRoutineBtn.style.display = 'inline-block';
            clearGoalBtn.style.display = 'inline-block';
            _zone = null; // clear any zone
        } else if (_drawingZone) {
            if (!_zoneStart) {
                _zoneStart = { x: worldX, y: worldY };
                _zone = null;
            } else {
                _zone = {
                    x_min: Math.min(_zoneStart.x, worldX),
                    y_min: Math.min(_zoneStart.y, worldY),
                    x_max: Math.max(_zoneStart.x, worldX),
                    y_max: Math.max(_zoneStart.y, worldY)
                };
                _zoneStart = null;
                _waypoints = []; // clear any waypoints
                startNavBtn.style.display = 'inline-block';
                saveRoutineBtn.style.display = 'inline-block';
                clearGoalBtn.style.display = 'inline-block';
                _resetUI(); // Auto-exit draw mode when complete
            }
        }
        
        if (_lastMapData) _drawMap(_lastMapData);
    });
});

// â”€â”€ UI helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
function updateUI(state) {
    const isOn = state.motors_on;
    powerBtn.className    = isOn ? 'led-on' : 'led-off';
    powerText.textContent = isOn ? 'MOTORS ON' : 'MOTORS OFF';
    speedSlider.value     = state.speed;
    speedVal.textContent  = state.speed;

    // Vacuum
    if (typeof state.vacuum !== 'undefined' && vacuumSlider && vacuumVal) {
        const v = parseInt(state.vacuum) || 0;
        vacuumSlider.value = v;
        vacuumVal.textContent = v.toString();
        if (vacuumToggleBtn) {
            vacuumToggleBtn.textContent = v > 0 ? '🌀 Vacuum ON' : '🌀 Vacuum OFF';
            vacuumToggleBtn.style.background = v > 0 ? '#4CAF50' : '#008184';
        }
    }

    // Brush
    if (typeof state.brush !== 'undefined' && brushSlider) {
        const b = parseInt(state.brush) || 0;
        brushSlider.value = b;
        if (brushVal) brushVal.textContent = b.toString();
        if (brushToggleBtn) {
            brushToggleBtn.textContent = b !== 0 ? '🪥 Brush ON' : '🪥 Brush OFF';
            brushToggleBtn.style.background = b !== 0 ? '#4CAF50' : '#5d4037';
        }
    }

    // Auto-clean button label
    if (autoCleanBtn) {
        autoCleanBtn.textContent = state.auto_clean ? '🛑 Stop Auto-Clean' : '🧹 Start Auto-Clean';
        autoCleanBtn.style.background = state.auto_clean ? '#e53935' : '#6a1b9a';
    }

    // Auto-clear UI if robot stops navigating
    if (!state.navigating && (_waypoints.length > 0 || _zone)) {
        _waypoints = [];
        _zone = null;
        clearGoalBtn.style.display = 'none';
        startNavBtn.style.display = 'none';
        saveRoutineBtn.style.display = 'none';
        if (_lastMapData) _drawMap(_lastMapData);
    }
}

function updateTelemetry(data) {
    if (encL) encL.textContent     = data.enc_l;
    if (encR) encR.textContent     = data.enc_r;
    if (imuAccel) imuAccel.textContent = `X: ${data.accel_x.toFixed(2)} | Y: ${data.accel_y.toFixed(2)} | Z: ${data.accel_z.toFixed(2)}`;
    if (imuGyro)  imuGyro.textContent  = `X: ${data.gyro_x.toFixed(2)} | Y: ${data.gyro_y.toFixed(2)} | Z: ${data.gyro_z.toFixed(2)}`;
}

function updateUltrasonics(u) {
    const fmt = (v) => (v >= 380 || v <= 0) ? '∞' : Math.round(v).toString();
    if (usFront) usFront.textContent = fmt(u.front);
    if (usRight) usRight.textContent = fmt(u.right);
    if (usLeft)  usLeft.textContent  = fmt(u.left);
    // Mirror to nav tab cam placeholder if needed
    const frontWarn = u.front < 20;
    if (usFront) usFront.style.color = frontWarn ? '#ff5252' : '#69f0ae';
}

function updateEKF(e) {
    ekfX.textContent     = e.x_cm.toFixed(2);
    ekfY.textContent     = e.y_cm.toFixed(2);
    ekfTheta.textContent = `${(e.theta_rad * 180 / Math.PI).toFixed(1)}Â°`;
    ekfDist.textContent  = Math.sqrt(e.x_cm ** 2 + e.y_cm ** 2).toFixed(2);

    // Update robot position and redraw map immediately (smooth motion)
    _latestPose = e;
    if (_trajectory.length === 0) {
        _trajectory.push({ x: e.x_cm, y: e.y_cm });
    } else {
        const last = _trajectory[_trajectory.length - 1];
        if (Math.hypot(e.x_cm - last.x, e.y_cm - last.y) > 2.0) {
            _trajectory.push({ x: e.x_cm, y: e.y_cm });
        }
    }
    _drawMap(_lastMapData); // always redraw â€” works even without map data
}

// â”€â”€ Map state â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
let _latestPose      = null;
let _trajectory      = [];
let _lastMapData     = null;
let _trajectoryOnly  = false;   // true = suppress occupancy-cell coloring on Map tab

// Viewport: always show robot at centre, Â±300 cm visible
const HALF_VIEW = 300;  // cm
const MAP_PX    = 600;  // canvas px
const PX_PER_CM = MAP_PX / (HALF_VIEW * 2);

/** World cm â†’ canvas px, centred on robot. */
function w2p(wx, wy) {
    const cx = _latestPose ? _latestPose.x_cm : 0;
    const cy = _latestPose ? _latestPose.y_cm : 0;
    return {
        x: MAP_PX / 2 + (wx - cx) * PX_PER_CM,
        y: MAP_PX / 2 - (wy - cy) * PX_PER_CM   // canvas Y is flipped
    };
}

function renderMap(m) {
    _lastMapData = m;
    _drawMap(m);
    coveragePct.textContent = `${m.coverage}%`;
}

function _drawMap(m) {
    // m can be null on first render â€” draw background/trajectory/robot without grid
    const cols       = m ? m.cols       : 33;
    const rows       = m ? m.rows       : 33;
    const data       = m ? m.data       : null;
    const origin_col = m ? m.origin_col : 16;
    const origin_row = m ? m.origin_row : 16;
    const cell_cm    = m ? m.cell_cm    : 30;

    mapCanvas.width  = MAP_PX;
    mapCanvas.height = MAP_PX;

    // 1 â”€â”€ Background
    mapCtx.fillStyle = '#E8EEEE';
    mapCtx.fillRect(0, 0, MAP_PX, MAP_PX);

    // 2 â”€â”€ Occupancy grid cells (only non-UNKNOWN, skip if no data)
    const cellPx = cell_cm * PX_PER_CM;
    if (data && !_trajectoryOnly) {
        for (let r = 0; r < rows; r++) {
            for (let c = 0; c < cols; c++) {
                const val = data[r][c];
                if (val === CELL_UNKNOWN) continue;
                const wx = (c - origin_col) * cell_cm + cell_cm / 2;
                const wy = (origin_row - r) * cell_cm - cell_cm / 2;
                const p  = w2p(wx, wy);
                mapCtx.fillStyle = cellColor(val);
                mapCtx.fillRect(p.x - cellPx / 2, p.y - cellPx / 2, cellPx, cellPx);
            }
        }
    }

    // 3 â”€â”€ Grid lines
    const robotX = _latestPose ? _latestPose.x_cm : 0;
    const robotY = _latestPose ? _latestPose.y_cm : 0;
    mapCtx.strokeStyle = 'rgba(0,0,0,0.09)';
    mapCtx.lineWidth   = 0.5;
    const x0 = Math.floor((robotX - HALF_VIEW) / cell_cm) * cell_cm;
    const y0 = Math.floor((robotY - HALF_VIEW) / cell_cm) * cell_cm;
    for (let wx = x0; wx <= robotX + HALF_VIEW; wx += cell_cm) {
        const p = w2p(wx, 0);
        mapCtx.beginPath(); mapCtx.moveTo(p.x, 0); mapCtx.lineTo(p.x, MAP_PX); mapCtx.stroke();
    }
    for (let wy = y0; wy <= robotY + HALF_VIEW; wy += cell_cm) {
        const p = w2p(0, wy);
        mapCtx.beginPath(); mapCtx.moveTo(0, p.y); mapCtx.lineTo(MAP_PX, p.y); mapCtx.stroke();
    }

    // 4 â”€â”€ Origin crosshair (blue dashes)
    const op = w2p(0, 0);
    mapCtx.strokeStyle = 'rgba(33,150,243,0.6)';
    mapCtx.lineWidth   = 1.5;
    mapCtx.setLineDash([5, 4]);
    mapCtx.beginPath(); mapCtx.moveTo(op.x - 14, op.y); mapCtx.lineTo(op.x + 14, op.y); mapCtx.stroke();
    mapCtx.beginPath(); mapCtx.moveTo(op.x, op.y - 14); mapCtx.lineTo(op.x, op.y + 14); mapCtx.stroke();
    mapCtx.setLineDash([]);

    // 5 â”€â”€ Trajectory path (red line)
    if (_trajectory.length > 1) {
        mapCtx.beginPath();
        mapCtx.strokeStyle = 'rgba(229, 57, 53, 0.95)';
        mapCtx.lineWidth   = 2.5;
        mapCtx.lineJoin    = 'round';
        _trajectory.forEach((pt, i) => {
            const p = w2p(pt.x, pt.y);
            i === 0 ? mapCtx.moveTo(p.x, p.y) : mapCtx.lineTo(p.x, p.y);
        });
        mapCtx.stroke();
        // Start dot
        const sp = w2p(_trajectory[0].x, _trajectory[0].y);
        mapCtx.beginPath();
        mapCtx.arc(sp.x, sp.y, 5, 0, Math.PI * 2);
        mapCtx.fillStyle = 'rgba(229,57,53,0.8)';
        mapCtx.fill();
    }

    // 6 â”€â”€ Robot marker (always at canvas centre)
    const theta = _latestPose ? _latestPose.theta_rad : 0;
    const R = 12;
    mapCtx.save();
    mapCtx.translate(MAP_PX / 2, MAP_PX / 2);
    mapCtx.rotate(-theta);
    // Body
    mapCtx.beginPath();
    mapCtx.arc(0, 0, R, 0, Math.PI * 2);
    mapCtx.fillStyle   = '#00897B';
    mapCtx.fill();
    mapCtx.strokeStyle = '#ffffff';
    mapCtx.lineWidth   = 2;
    mapCtx.stroke();
    // Heading arrow
    mapCtx.beginPath();
    mapCtx.moveTo(0, 0);
    mapCtx.lineTo(R + 9, 0);
    mapCtx.strokeStyle = '#ffffff';
    mapCtx.lineWidth   = 3;
    mapCtx.stroke();
    mapCtx.restore();

    // 7 â”€â”€ Zone and Waypoints
    if (_zone) {
        const p1 = w2p(_zone.x_min, _zone.y_max);
        const p2 = w2p(_zone.x_max, _zone.y_min);
        mapCtx.fillStyle = 'rgba(76, 175, 80, 0.2)';
        mapCtx.strokeStyle = '#4CAF50';
        mapCtx.lineWidth = 2;
        mapCtx.fillRect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y);
        mapCtx.strokeRect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y);
    } else if (_zoneStart) {
        const p = w2p(_zoneStart.x, _zoneStart.y);
        mapCtx.beginPath(); mapCtx.arc(p.x, p.y, 4, 0, Math.PI*2);
        mapCtx.fillStyle = '#4CAF50'; mapCtx.fill();
    }

    if (_waypoints.length > 0) {
        // Draw lines
        mapCtx.beginPath();
        let lastPt = {x: MAP_PX / 2, y: MAP_PX / 2}; // Robot center
        mapCtx.moveTo(lastPt.x, lastPt.y);
        _waypoints.forEach(pt => {
            const p = w2p(pt.x, pt.y);
            mapCtx.lineTo(p.x, p.y);
        });
        mapCtx.strokeStyle = 'rgba(76, 175, 80, 0.7)';
        mapCtx.lineWidth = 2;
        mapCtx.setLineDash([5, 5]);
        mapCtx.stroke();
        mapCtx.setLineDash([]);
        
        // Draw directional arrows
        lastPt = {x: MAP_PX / 2, y: MAP_PX / 2};
        mapCtx.beginPath();
        _waypoints.forEach(pt => {
            const p = w2p(pt.x, pt.y);
            const dx = p.x - lastPt.x;
            const dy = p.y - lastPt.y;
            const angle = Math.atan2(dy, dx);
            // Draw arrow head near the destination point (or midpoint)
            const mx = lastPt.x + dx * 0.6;
            const my = lastPt.y + dy * 0.6;
            
            mapCtx.save();
            mapCtx.translate(mx, my);
            mapCtx.rotate(angle);
            mapCtx.moveTo(0, 0);
            mapCtx.lineTo(-8, -6);
            mapCtx.moveTo(0, 0);
            mapCtx.lineTo(-8, 6);
            mapCtx.restore();
            
            lastPt = p;
        });
        mapCtx.strokeStyle = 'rgba(76, 175, 80, 0.9)';
        mapCtx.lineWidth = 2;
        mapCtx.stroke();

        // Draw points
        _waypoints.forEach((pt, i) => {
            const p = w2p(pt.x, pt.y);
            mapCtx.beginPath();
            mapCtx.arc(p.x, p.y, 8, 0, Math.PI * 2);
            mapCtx.fillStyle = '#4CAF50';
            mapCtx.fill();
            mapCtx.strokeStyle = '#FFFFFF';
            mapCtx.lineWidth = 2;
            mapCtx.stroke();
            // Number
            mapCtx.fillStyle = '#FFFFFF';
            mapCtx.font = '10px sans-serif';
            mapCtx.textAlign = 'center';
            mapCtx.textBaseline = 'middle';
            mapCtx.fillText((i+1).toString(), p.x, p.y);
        });
    }

    // 8 â”€â”€ Coordinate label (bottom-left)
    mapCtx.fillStyle = 'rgba(0,0,0,0.5)';
    mapCtx.font      = '11px monospace';
    const lbl = _latestPose
        ? `X: ${robotX.toFixed(1)} cm   Y: ${robotY.toFixed(1)} cm`
        : 'Waiting for pose...';
    mapCtx.fillText(lbl, 8, MAP_PX - 8);
}

function cellColor(val) {
    if (val === CELL_WALL)     return '#263238';  // dark charcoal
    if (val === CELL_OBSTACLE) return '#b71c1c';  // deep red
    if (val === CELL_CLEANED)  return '#26a69a';  // teal green
    if (val === CELL_FREE)     return '#b2dfdb';  // light teal
    return '#E8EEEE';                             // unknown â€” same as background
}
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
// =============================================================================
// NAV TAB - Obstacle Map canvas rendering
// Driven by the 'obstacle_map_update' socket event emitted by Python when
// ultrasonic readings update the nav-grid.
// Grid payload: { cols, rows, data[][], origin_col, origin_row, cell_cm,
//                robot: {x_cm, y_cm, theta_rad}, goal: {x_cm, y_cm} | null }
// Each data[r][c] value: 0 = free, 1-255 = obstacle confidence.
// =============================================================================

let _lastNavData = null;

/** Convert nav-grid world cm -> nav-canvas pixels, centred on robot. */
function navW2P(wx, wy, canvasW, canvasH, pose, navPxPerCm) {
    const cx = pose ? pose.x_cm : 0;
    const cy = pose ? pose.y_cm : 0;
    return {
        x: canvasW / 2 + (wx - cx) * navPxPerCm,
        y: canvasH / 2 - (wy - cy) * navPxPerCm,
    };
}

function renderObstacleMap(m) {
    _lastNavData = m;
    if (!navCtx || !navCanvas) return;

    const cols       = m.cols       || 21;
    const rows       = m.rows       || 21;
    const data       = m.data       || null;
    const origin_col = m.origin_col != null ? m.origin_col : Math.floor(cols / 2);
    const origin_row = m.origin_row != null ? m.origin_row : Math.floor(rows / 2);
    const cell_cm    = m.cell_cm    || 20;
    const pose       = m.robot      || _latestPose || null;
    const goal       = m.goal       || null;

    // Size the canvas to fill its CSS container
    const cssW = navCanvas.clientWidth  || 400;
    const cssH = navCanvas.clientHeight || 400;
    navCanvas.width  = cssW;
    navCanvas.height = cssH;

    const navPxPerCm = Math.min(cssW, cssH) / (Math.max(cols, rows) * cell_cm);
    const cellPx     = cell_cm * navPxPerCm;

    // 1 -- Dark background
    navCtx.fillStyle = '#111827';
    navCtx.fillRect(0, 0, cssW, cssH);

    // 2 -- Obstacle cells shaded by confidence (0=free, 255=solid obstacle)
    if (data) {
        for (let r = 0; r < rows; r++) {
            for (let c = 0; c < cols; c++) {
                const conf = data[r][c] || 0;
                if (conf === 0) continue;
                const wx = (c - origin_col) * cell_cm + cell_cm / 2;
                const wy = (origin_row - r) * cell_cm - cell_cm / 2;
                const p  = navW2P(wx, wy, cssW, cssH, pose, navPxPerCm);
                const alpha = Math.min(1, conf / 255);
                navCtx.fillStyle = 'rgba(255,87,34,' + (0.25 + 0.75 * alpha).toFixed(2) + ')';
                navCtx.fillRect(p.x - cellPx / 2, p.y - cellPx / 2, cellPx, cellPx);
            }
        }
    }

    // 3 -- Faint grid lines (centred on robot)
    navCtx.strokeStyle = 'rgba(255,255,255,0.07)';
    navCtx.lineWidth   = 0.5;
    const pCx = pose ? pose.x_cm : 0;
    const pCy = pose ? pose.y_cm : 0;
    const halfSpanCm = Math.max(cols, rows) * cell_cm / 2;
    for (let wx = pCx - halfSpanCm; wx <= pCx + halfSpanCm; wx += cell_cm) {
        const p = navW2P(wx, pCy, cssW, cssH, pose, navPxPerCm);
        navCtx.beginPath(); navCtx.moveTo(p.x, 0); navCtx.lineTo(p.x, cssH); navCtx.stroke();
    }
    for (let wy = pCy - halfSpanCm; wy <= pCy + halfSpanCm; wy += cell_cm) {
        const p = navW2P(pCx, wy, cssW, cssH, pose, navPxPerCm);
        navCtx.beginPath(); navCtx.moveTo(0, p.y); navCtx.lineTo(cssW, p.y); navCtx.stroke();
    }

    // 4 -- Trajectory path overlay
    if (_trajectory.length > 1) {
        navCtx.beginPath();
        navCtx.strokeStyle = 'rgba(105,240,174,0.75)';
        navCtx.lineWidth   = 2;
        navCtx.lineJoin    = 'round';
        _trajectory.forEach(function(pt, i) {
            const p = navW2P(pt.x, pt.y, cssW, cssH, pose, navPxPerCm);
            i === 0 ? navCtx.moveTo(p.x, p.y) : navCtx.lineTo(p.x, p.y);
        });
        navCtx.stroke();
    }

    // 5 -- Goal star
    if (goal) {
        const gp = navW2P(goal.x_cm, goal.y_cm, cssW, cssH, pose, navPxPerCm);
        navCtx.font         = '18px serif';
        navCtx.textAlign    = 'center';
        navCtx.textBaseline = 'middle';
        navCtx.fillText('\u2B50', gp.x, gp.y);
    }

    // 6 -- Robot dot (always at canvas centre because we track robot position)
    const theta = pose ? pose.theta_rad : 0;
    const RR    = 10;
    navCtx.save();
    navCtx.translate(cssW / 2, cssH / 2);
    navCtx.rotate(-theta);
    navCtx.beginPath();
    navCtx.arc(0, 0, RR, 0, Math.PI * 2);
    navCtx.fillStyle   = '#1565C0';
    navCtx.fill();
    navCtx.strokeStyle = '#90CAF9';
    navCtx.lineWidth   = 2;
    navCtx.stroke();
    // Heading arrow
    navCtx.beginPath();
    navCtx.moveTo(0, 0);
    navCtx.lineTo(RR + 8, 0);
    navCtx.strokeStyle = '#90CAF9';
    navCtx.lineWidth   = 2.5;
    navCtx.stroke();
    navCtx.restore();

    // 7 -- Coordinate label (bottom-left)
    const lbl = pose
        ? 'X: ' + pose.x_cm.toFixed(1) + ' cm  Y: ' + pose.y_cm.toFixed(1) + ' cm'
        : 'Waiting for pose...';
    navCtx.fillStyle    = 'rgba(255,255,255,0.4)';
    navCtx.font         = '11px monospace';
    navCtx.textAlign    = 'left';
    navCtx.textBaseline = 'bottom';
    navCtx.fillText(lbl, 8, cssH - 6);
}

// CAMERA TAB â€” iframe live stream + snapshot + upload+detect + detections log
// Stream is served automatically by the arduino:video_object_detection brick
// at http://BOARD:4912/embed â€” no manual start/stop needed.
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

(function () {
    const camIframe      = document.getElementById('cam-iframe');
    const camPlaceholder = document.getElementById('cam-placeholder');
    const camStatus      = document.getElementById('camera-status');
    const camImg         = document.getElementById('camera-img');
    const snapshotBtn    = document.getElementById('snapshot-btn');
    const uploadBtn      = document.getElementById('camera-upload-btn');
    const detectUploadBtn= document.getElementById('camera-detect-upload-btn');
    const downloadBtn    = document.getElementById('camera-download-btn');
    const fileInput      = document.getElementById('camera-file-input');
    const confidenceSlider = document.getElementById('cam-confidence');
    const confidenceVal  = document.getElementById('cam-confidence-val');
    const detectionsList = document.getElementById('cam-detections-list');

    const STREAM_PORT = VIDEO_HTTP_PORT;
    const STREAM_PATH = '/embed';
    let _streamIntervalId = null;
    let _lastResultB64    = null;
    let _uploadedB64      = null;
    const MAX_DETECTIONS  = 6;
    let _detections       = [];

    // â”€â”€ iframe auto-retry (same pattern as video-generic-object-detection) â”€â”€â”€â”€
    function startIframeStream() {
        if (_streamIntervalId) return;
        const url = `http://${window.location.hostname}:${STREAM_PORT}${STREAM_PATH}`;
        camIframe.onload = () => {
            if (_streamIntervalId) { clearInterval(_streamIntervalId); _streamIntervalId = null; }
            camPlaceholder.style.display = 'none';
            camIframe.style.display = 'block';
            setStatus('Live stream active âœ…', '#4CAF50');
        };
        camIframe.onerror = () => setStatus('Camera not detected', '#ef5350');
        _streamIntervalId = setInterval(() => { camIframe.src = url; }, 1500);
        camIframe.src = url;
    }

    function stopIframeStream() {
        if (_streamIntervalId) { clearInterval(_streamIntervalId); _streamIntervalId = null; }
        camIframe.src = '';
        camIframe.style.display = 'none';
        camPlaceholder.style.display = 'block';
        setStatus('', '');
    }

    // â”€â”€ Auto-start/stop stream when Camera tab is active â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    const origSwitchTab = window.switchTab;
    window.switchTab = function (name) {
        origSwitchTab(name);
        if (name === 'camera') startIframeStream();
        else stopIframeStream();
    };

    // â”€â”€ helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    function setStatus(msg, color) {
        if (camStatus) { camStatus.textContent = msg; camStatus.style.color = color || '#aaa'; }
    }

    function showSnapshotImage(b64) {
        camImg.src = `data:image/jpeg;base64,${b64}`;
        camImg.style.display = 'block';
        downloadBtn.style.display = 'inline-block';
        _lastResultB64 = b64;
    }

    // â”€â”€ confidence slider â†’ override_th â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    confidenceSlider.addEventListener('input', () => {
        const v = parseInt(confidenceSlider.value) / 100;
        confidenceVal.textContent = confidenceSlider.value;
        socket.emit('override_th', v);
    });

    // â”€â”€ snapshot button â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    snapshotBtn.addEventListener('click', () => {
        setStatus('Capturing snapshotâ€¦', '#FFA726');
        socket.emit('take_snapshot', {});
    });

    socket.on('snapshot_result', (data) => {
        showSnapshotImage(data.image);
        setStatus('Snapshot captured âœ…', '#4CAF50');
    });

    socket.on('snapshot_error', (data) => {
        setStatus(`âŒ ${data.error}`, '#ef5350');
    });

    // â”€â”€ upload image â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    uploadBtn.addEventListener('click', () => fileInput.click());

    fileInput.addEventListener('change', (e) => {
        const file = e.target.files[0];
        if (!file) return;
        const reader = new FileReader();
        reader.onload = (ev) => {
            _uploadedB64 = ev.target.result.split(',')[1];
            showSnapshotImage(_uploadedB64);
            setStatus('Image loaded â€” click Upload+Detect to analyse', '#90CAF9');
        };
        reader.readAsDataURL(file);
        fileInput.value = '';
    });

    // â”€â”€ upload + detect â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    detectUploadBtn.addEventListener('click', () => {
        if (!_uploadedB64) { setStatus('Upload an image first', '#ef5350'); return; }
        setStatus('Running detectionâ€¦', '#FFA726');
        const confidence = parseInt(confidenceSlider.value) / 100;
        socket.emit('camera_detect', { image: _uploadedB64, confidence });
    });

    socket.on('detection_result', (data) => {
        if (!data.result_image) return;
        _lastResultB64 = data.result_image;
        camImg.src = `data:image/png;base64,${data.result_image}`;
        camImg.style.display = 'block';
        downloadBtn.style.display = 'inline-block';
        setStatus(`âœ… Found ${data.detection_count} object(s)!`, '#4CAF50');
    });

    socket.on('detection_error', (data) => {
        setStatus(`âŒ ${data.error}`, '#ef5350');
    });

    // â”€â”€ live detections from VideoObjectDetection brick â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    socket.on('detection', (entry) => {
        _detections.unshift(entry);
        if (_detections.length > MAX_DETECTIONS) _detections.pop();
        renderDetections();
    });

    function renderDetections() {
        if (!detectionsList) return;
        if (_detections.length === 0) {
            detectionsList.innerHTML = '<li style="color:#555;font-size:13px;">Waiting for detectionsâ€¦</li>';
            return;
        }
        detectionsList.innerHTML = _detections.map(d => {
            const pct = Math.floor((d.confidence || 0) * 100);
            const ts  = d.timestamp ? new Date(d.timestamp).toLocaleTimeString() : '';
            return `<li style="display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid rgba(255,255,255,0.06);font-size:13px;">
                <span style="color:#4CAF50;font-weight:600;">${d.content}</span>
                <span style="color:#aaa;">${pct}% &nbsp; <span style="color:#555;">${ts}</span></span>
            </li>`;
        }).join('');
    }
    // -- record GIF
    const recordBtn = document.getElementById('camera-record-btn');
    if (recordBtn) {
        recordBtn.addEventListener('click', () => {
            setStatus('Recording 5s GIF... please wait', '#FFA726');
            recordBtn.disabled = true;
            socket.emit('camera_record', { duration: 5 });
            setTimeout(() => { recordBtn.disabled = false; }, 7000);
        });
    }
    socket.on('record_result', (data) => {
        if (!data.gif) { setStatus('Recording returned no data', '#ef5350'); return; }
        _lastResultB64 = data.gif;
        camImg.src = 'data:image/gif;base64,' + data.gif;
        camImg.style.display = 'block';
        downloadBtn.style.display = 'inline-block';
        setStatus('GIF recorded (' + data.frames + ' frames)', '#4CAF50');
    });
    socket.on('record_error', (data) => { setStatus(data.error, '#ef5350'); });

    // ===== DIAGNOSTICS =====
    // postMessage from camera iframe only (ignore MetaMask/wallet/browser-ext noise on same origin)
    window.addEventListener('message', (event) => {
        const d = event.data;
        let s = '';
        try {
            s = typeof d === 'string' ? d : JSON.stringify(d || {});
        } catch (_) { return; }
        if (/metamask|chainChanged|metamask-provider|wallet/i.test(s)) return;

        const src = event.origin || 'unknown';
        const fromBrick = src.includes(`:${VIDEO_HTTP_PORT}`) || /^\s*\/9j\//.test(typeof d === 'string' ? d : '');
        if (!fromBrick && !(typeof d === 'string' && d.startsWith('/9j/'))) return;

        const preview = typeof d === 'string'
            ? d.substring(0, 200)
            : (d instanceof ArrayBuffer ? 'ArrayBuffer:' + d.byteLength + 'bytes' : s.substring(0, 200));
        setStatus('postMessage from ' + src + ': ' + preview, '#FFA726');
        socket.emit('diag_result', { source: 'postMessage', origin: src, preview: preview });
        if (typeof d === 'string' && d.startsWith('/9j/')) {
            socket.emit('frame_from_browser', { image: d });
        }
    });

    // 2. Diag button: probe common frame endpoints and report
    const diagBtn = document.getElementById('camera-diag-btn');
    if (diagBtn) {
        diagBtn.addEventListener('click', () => {
            setStatus('Running camera diagnostics...', '#FFA726');
            const host = window.location.hostname;
            const paths = ['/stream', '/', '/snapshot', '/frame', '/video', '/embed'];
            const results = [];
            const tryNext = (i) => {
                if (i >= paths.length) {
                    const summary = results.join(' | ');
                    setStatus('Diag: ' + summary, '#90CAF9');
                    socket.emit('diag_result', { source: 'fetch_probe', results: results });
                    return;
                }
                // Plain http dashboard; https page cannot fetch http MJPEG cross-origin easily
                const url = 'http://' + host + ':' + VIDEO_HTTP_PORT + paths[i];
                fetch(url, { mode: 'cors', signal: AbortSignal.timeout(3000) })
                    .then(r => {
                        const ct = r.headers.get('Content-Type') || 'no-ct';
                        results.push(paths[i] + '=' + r.status + '(' + ct + ')');
                        // If it's an image, try to read it as a frame
                        if (ct.includes('jpeg') || ct.includes('image')) {
                            return r.blob().then(blob => {
                                const reader = new FileReader();
                                reader.onload = () => {
                                    const b64 = reader.result.split(',')[1];
                                    if (b64) socket.emit('frame_from_browser', { image: b64, source: url });
                                };
                                reader.readAsDataURL(blob);
                            });
                        }
                        // If it's HTML, read first 500 chars
                        if (ct.includes('html') || ct.includes('text')) {
                            return r.text().then(txt => {
                                results.push('HTML:' + txt.substring(0, 300));
                                socket.emit('diag_result', { source: 'embed_html', html: txt.substring(0, 2000) });
                            });
                        }
                    })
                    .catch(e => results.push(paths[i] + '=ERR(' + e.message + ')'))
                    .finally(() => tryNext(i + 1));
            };
            tryNext(0);
        });
    }

    // 3. Backend requests embed HTML via Python
    socket.on('diag_embed_html', (data) => {
        setStatus('Python embed HTML: ' + data.html.substring(0, 200), '#90CAF9');
    });


    // Show probe results from Python in the camera status bar
    socket.on('probe_results', (data) => {
        const _ph = ':' + VIDEO_HTTP_PORT;
        const rows = (data.results || []).map(r => {
            const path = r.url && r.url.split(_ph)[1] != null ? r.url.split(_ph)[1] : r.url || '?';
            if (r.error) return path + '=ERR';
            const flags = (r.jpeg ? 'JPEG!' : '') + (r.ws.length ? ' WS:'+r.ws[0] : '') + (r.fetch.length ? ' FETCH:'+r.fetch[0] : '') + (r.srcs.length ? ' SRC:'+r.srcs[0] : '');
            return path + '=' + r.ct.split(';')[0] + (flags ? '['+flags+']' : '');
        }).join(' | ');
        setStatus('PROBE: ' + rows, '#90CAF9');
        console.log('PROBE RESULTS:', JSON.stringify(data.results, null, 2));
        socket.emit('diag_result', { source: 'probe_display', summary: rows });
    });

    socket.on('diag_result_ack', (data) => {
        setStatus('Diag received: ' + JSON.stringify(data).substring(0, 150), '#aaa');
    });
    // -- download
    downloadBtn.addEventListener('click', () => {
        if (!_lastResultB64) return;
        const a = document.createElement('a');
        const mime = _lastResultB64.startsWith('/9j/') ? 'jpeg' : 'png';
        a.href = `data:image/${mime};base64,${_lastResultB64}`;
        a.download = `aria-capture-${Date.now()}.${mime}`;
        document.body.appendChild(a); a.click(); document.body.removeChild(a);
    });
})();

