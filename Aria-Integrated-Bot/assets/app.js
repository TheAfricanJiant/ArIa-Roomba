// ── Element references ──────────────────────────────────────────────────────
const powerBtn    = document.getElementById('motor-power-btn');
const powerText   = document.getElementById('power-text');
const speedSlider = document.getElementById('speed-slider');
const speedVal    = document.getElementById('speed-val');

const encL     = document.getElementById('enc-l');
const encR     = document.getElementById('enc-r');
const imuAccel = document.getElementById('imu-accel');
const imuGyro  = document.getElementById('imu-gyro');

const ekfX     = document.getElementById('ekf-x');
const ekfY     = document.getElementById('ekf-y');
const ekfTheta = document.getElementById('ekf-theta');
const ekfDist  = document.getElementById('ekf-dist');

const mapCanvas   = document.getElementById('map-canvas');
const mapCtx      = mapCanvas.getContext('2d');
const coveragePct = document.getElementById('coverage-pct');
const setGoalBtn  = document.getElementById('set-goal-btn');
const drawZoneBtn = document.getElementById('draw-zone-btn');
const startNavBtn = document.getElementById('start-nav-btn');
const saveRoutineBtn = document.getElementById('save-routine-btn');
const clearGoalBtn= document.getElementById('clear-goal-btn');
const routinesSelect = document.getElementById('routines-select');

let _settingGoal = false;
let _drawingZone = false;
let _waypoints = []; // array of {x, y}
let _zoneStart = null; // {x, y}
let _zone = null; // {x_min, y_min, x_max, y_max}
let _routinesData = [];

// ── Cell state constants (match Python OccupancyGrid uint8 values) ──────────
const CELL_UNKNOWN  = 0;
const CELL_FREE     = 1;
const CELL_CLEANED  = 2;
const CELL_WALL     = 3;
const CELL_OBSTACLE = 4;

// ── Socket ───────────────────────────────────────────────────────────────────
const socket = io(`http://${window.location.host}`);

// ── Tab switching ─────────────────────────────────────────────────────────────
function switchTab(name) {
    document.querySelectorAll('.tab-panel').forEach(p => p.classList.add('hidden'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.getElementById(`tab-${name}`).classList.remove('hidden');
    document.getElementById(`tab-${name}-btn`).classList.add('active');
}

// ── Socket event wiring ───────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
    socket.on('connect', () => { socket.emit('get_initial_state', {}); });
    socket.on('state_update',     (s) => updateUI(s));
    socket.on('telemetry_update', (d) => updateTelemetry(d));
    socket.on('ekf_update',       (e) => updateEKF(e));
    socket.on('map_update',       (m) => renderMap(m));
    
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

    // Tool buttons
    setGoalBtn.addEventListener('click', () => {
        _settingGoal = true;
        _drawingZone = false;
        mapCanvas.style.cursor = 'crosshair';
        setGoalBtn.style.background = '#005e60';
        drawZoneBtn.style.background = '#008184';
        setGoalBtn.textContent = 'Click to add points...';
        drawZoneBtn.textContent = '🔲 Draw Zone';
        _zoneStart = null;
    });

    drawZoneBtn.addEventListener('click', () => {
        _drawingZone = true;
        _settingGoal = false;
        mapCanvas.style.cursor = 'crosshair';
        drawZoneBtn.style.background = '#005e60';
        setGoalBtn.style.background = '#008184';
        drawZoneBtn.textContent = 'Click 2 corners...';
        setGoalBtn.textContent = '📍 Add Waypoints';
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
        setGoalBtn.textContent = '📍 Add Waypoints';
        drawZoneBtn.textContent = '🔲 Draw Zone';
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
    
    // ── Video detection controls ──
    const confidenceSlider = document.getElementById('confidenceSlider');
    const confidenceValueDisplay = document.getElementById('confidenceValueDisplay');
    const recentDetectionsList = document.getElementById('recentDetections');
    
    if (confidenceSlider) {
        confidenceSlider.addEventListener('input', (e) => {
            confidenceValueDisplay.textContent = parseFloat(e.target.value).toFixed(2);
            socket.emit('override_th', parseFloat(e.target.value));
        });
    }

    socket.on('detection', (msg) => {
        const li = document.createElement('li');
        li.style.background = '#f5f5f5';
        li.style.margin = '8px 0';
        li.style.padding = '8px 12px';
        li.style.borderRadius = '6px';
        li.style.fontFamily = 'monospace';
        li.style.boxShadow = '0 1px 3px rgba(0,0,0,0.1)';
        
        const pct = Math.round(msg.confidence * 100);
        const timeStr = new Date(msg.timestamp).toLocaleTimeString();
        
        li.innerHTML = `<strong>${msg.content}</strong> <span style="color: #008184;">(${pct}%)</span> <span style="float: right; color: #888;">${timeStr}</span>`;
        
        recentDetectionsList.prepend(li);
        
        if (recentDetectionsList.children.length > 20) {
            recentDetectionsList.removeChild(recentDetectionsList.lastChild);
        }
    });
});

// ── UI helpers ────────────────────────────────────────────────────────────────
function updateUI(state) {
    const isOn = state.motors_on;
    powerBtn.className   = isOn ? 'led-on' : 'led-off';
    powerText.textContent = isOn ? 'MOTORS ON' : 'MOTORS OFF';
    speedSlider.value    = state.speed;
    speedVal.textContent = state.speed;
    
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
    encL.textContent     = data.enc_l;
    encR.textContent     = data.enc_r;
    imuAccel.textContent = `X: ${data.accel_x.toFixed(2)} | Y: ${data.accel_y.toFixed(2)} | Z: ${data.accel_z.toFixed(2)}`;
    imuGyro.textContent  = `X: ${data.gyro_x.toFixed(2)} | Y: ${data.gyro_y.toFixed(2)} | Z: ${data.gyro_z.toFixed(2)}`;
}

function updateEKF(e) {
    ekfX.textContent     = e.x_cm.toFixed(2);
    ekfY.textContent     = e.y_cm.toFixed(2);
    ekfTheta.textContent = `${(e.theta_rad * 180 / Math.PI).toFixed(1)}°`;
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
    _drawMap(_lastMapData); // always redraw — works even without map data
}

// ── Map state ─────────────────────────────────────────────────────────────────
let _latestPose  = null;
let _trajectory  = [];
let _lastMapData = null;

// Viewport: always show robot at centre, ±300 cm visible
const HALF_VIEW = 300;  // cm
const MAP_PX    = 600;  // canvas px
const PX_PER_CM = MAP_PX / (HALF_VIEW * 2);

/** World cm → canvas px, centred on robot. */
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
    // m can be null on first render — draw background/trajectory/robot without grid
    const cols       = m ? m.cols       : 33;
    const rows       = m ? m.rows       : 33;
    const data       = m ? m.data       : null;
    const origin_col = m ? m.origin_col : 16;
    const origin_row = m ? m.origin_row : 16;
    const cell_cm    = m ? m.cell_cm    : 30;

    mapCanvas.width  = MAP_PX;
    mapCanvas.height = MAP_PX;

    // 1 ── Background
    mapCtx.fillStyle = '#E8EEEE';
    mapCtx.fillRect(0, 0, MAP_PX, MAP_PX);

    // 2 ── Occupancy grid cells (only non-UNKNOWN, skip if no data)
    const cellPx = cell_cm * PX_PER_CM;
    if (data) {
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

    // 3 ── Grid lines
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

    // 4 ── Origin crosshair (blue dashes)
    const op = w2p(0, 0);
    mapCtx.strokeStyle = 'rgba(33,150,243,0.6)';
    mapCtx.lineWidth   = 1.5;
    mapCtx.setLineDash([5, 4]);
    mapCtx.beginPath(); mapCtx.moveTo(op.x - 14, op.y); mapCtx.lineTo(op.x + 14, op.y); mapCtx.stroke();
    mapCtx.beginPath(); mapCtx.moveTo(op.x, op.y - 14); mapCtx.lineTo(op.x, op.y + 14); mapCtx.stroke();
    mapCtx.setLineDash([]);

    // 5 ── Trajectory path (red line)
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

    // 6 ── Robot marker (always at canvas centre)
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

    // 7 ── Zone and Waypoints
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

    // 8 ── Coordinate label (bottom-left)
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
    return '#E8EEEE';                             // unknown — same as background
}