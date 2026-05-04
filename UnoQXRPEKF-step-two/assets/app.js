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

    socket.on('disconnect', () => {
        const err = document.getElementById('error-container');
        if (err) { err.textContent = 'Connection lost.'; err.style.display = 'block'; }
    });

    powerBtn.addEventListener('click', () => socket.emit('toggle_power', {}));
    speedSlider.addEventListener('input',  (e) => { speedVal.textContent = e.target.value; });
    speedSlider.addEventListener('change', (e) => socket.emit('set_speed', { speed: parseInt(e.target.value) }));
});

// ── UI helpers ────────────────────────────────────────────────────────────────
function updateUI(state) {
    const isOn = state.motors_on;
    powerBtn.className   = isOn ? 'led-on' : 'led-off';
    powerText.textContent = isOn ? 'MOTORS ON' : 'MOTORS OFF';
    speedSlider.value    = state.speed;
    speedVal.textContent = state.speed;
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

    // 7 ── Coordinate label (bottom-left)
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