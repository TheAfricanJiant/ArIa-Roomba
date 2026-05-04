// ── Element references ─────────────────────────────────────────────────────
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

const mapCanvas    = document.getElementById('map-canvas');
const mapCtx       = mapCanvas.getContext('2d');
const coveragePct  = document.getElementById('coverage-pct');

const socketDebug  = document.getElementById('socket-debug'); // may be null now — that's fine

// ── Grid constants (MUST match Python OccupancyGrid uint8 values) ───────────
// occupancy_grid.py:  UNKNOWN=0, FREE=1, CLEANED=2, WALL=3, OBSTACLE=4
const CELL_UNKNOWN  = 0;
const CELL_FREE     = 1;
const CELL_CLEANED  = 2;
const CELL_WALL     = 3;
const CELL_OBSTACLE = 4;

// ── Socket ──────────────────────────────────────────────────────────────────
const socket = io(`http://${window.location.host}`);

// ── Tab switching ────────────────────────────────────────────────────────────
function switchTab(name) {
    document.querySelectorAll('.tab-panel').forEach(p => p.classList.add('hidden'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.getElementById(`tab-${name}`).classList.remove('hidden');
    document.getElementById(`tab-${name}-btn`).classList.add('active');
}

// ── Socket events ────────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
    socket.on('connect', () => {
        socket.emit('get_initial_state', {});
    });

    socket.on('state_update',    (s) => updateUI(s));
    socket.on('telemetry_update',(d) => updateTelemetry(d));
    socket.on('ekf_update',      (e) => updateEKF(e));
    socket.on('map_update',      (m) => renderMap(m));

    socket.on('disconnect', () => {
        const err = document.getElementById('error-container');
        if (err) {
            err.textContent = 'Connection to the board lost. Please check the connection.';
            err.style.display = 'block';
        }
    });

    // Motor button
    powerBtn.addEventListener('click', () => socket.emit('toggle_power', {}));

    // Speed slider — live label update
    speedSlider.addEventListener('input', (e) => {
        speedVal.textContent = e.target.value;
    });
    // Send to server only on release
    speedSlider.addEventListener('change', (e) => {
        socket.emit('set_speed', { speed: parseInt(e.target.value) });
    });
});

// ── UI update helpers ────────────────────────────────────────────────────────
function updateUI(state) {
    const isOn = state.motors_on;
    powerBtn.className = isOn ? 'led-on' : 'led-off';
    powerText.textContent = isOn ? 'MOTORS ON' : 'MOTORS OFF';
    speedSlider.value = state.speed;
    speedVal.textContent = state.speed;
}

function updateTelemetry(data) {
    encL.textContent = data.enc_l;
    encR.textContent = data.enc_r;
    imuAccel.textContent = `X: ${data.accel_x.toFixed(2)} | Y: ${data.accel_y.toFixed(2)} | Z: ${data.accel_z.toFixed(2)}`;
    imuGyro.textContent  = `X: ${data.gyro_x.toFixed(2)} | Y: ${data.gyro_y.toFixed(2)} | Z: ${data.gyro_z.toFixed(2)}`;
}

function updateEKF(e) {
    ekfX.textContent     = e.x_cm.toFixed(2);
    ekfY.textContent     = e.y_cm.toFixed(2);
    const deg = (e.theta_rad * 180 / Math.PI).toFixed(1);
    ekfTheta.textContent = `${deg}°`;
    const dist = Math.sqrt(e.x_cm ** 2 + e.y_cm ** 2).toFixed(2);
    ekfDist.textContent  = dist;
    if (socketDebug) socketDebug.textContent = `LAST EKF: ${JSON.stringify(e)}`;
}

// ── Map rendering ────────────────────────────────────────────────────────────
let _latestPose = null; // updated by ekf_update for live robot marker
let _trajectory = [];   // historical path points

socket.on('ekf_update', (e) => { 
    _latestPose = e; 
    
    // Append to trajectory if moved > 1cm to keep rendering fast
    if (_trajectory.length === 0) {
        _trajectory.push({x: e.x_cm, y: e.y_cm});
    } else {
        const last = _trajectory[_trajectory.length - 1];
        const dist = Math.hypot(e.x_cm - last.x, e.y_cm - last.y);
        if (dist > 1.0) {
            _trajectory.push({x: e.x_cm, y: e.y_cm});
        }
    }
});

function renderMap(m) {
    if (socketDebug && Math.random() < 0.1) socketDebug.textContent = `MAP: cols=${m.cols}, cov=${m.coverage}%`;
    const { cols, rows, data, origin_col, origin_row, coverage, cell_cm } = m;

    // Set intrinsic canvas pixel size (16 px per cell looks sharp at any screen width)
    const cellPx = 16;
    mapCanvas.width  = cols * cellPx;
    mapCanvas.height = rows * cellPx;

    // Draw cells
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            const val = data[r][c];
            mapCtx.fillStyle = cellColor(val);
            mapCtx.fillRect(c * cellPx, r * cellPx, cellPx, cellPx);
        }
    }

    // Grid lines (subtle)
    mapCtx.strokeStyle = 'rgba(0,0,0,0.08)';
    mapCtx.lineWidth = 0.5;
    for (let r = 0; r <= rows; r++) {
        mapCtx.beginPath();
        mapCtx.moveTo(0, r * cellPx);
        mapCtx.lineTo(cols * cellPx, r * cellPx);
        mapCtx.stroke();
    }
    for (let c = 0; c <= cols; c++) {
        mapCtx.beginPath();
        mapCtx.moveTo(c * cellPx, 0);
        mapCtx.lineTo(c * cellPx, rows * cellPx);
        mapCtx.stroke();
    }

    // Draw trajectory path
    if (_trajectory.length > 1) {
        mapCtx.beginPath();
        mapCtx.strokeStyle = 'rgba(233, 30, 99, 0.8)'; // Pink/Red line
        mapCtx.lineWidth = 2;
        
        for (let i = 0; i < _trajectory.length; i++) {
            const pt = _trajectory[i];
            const pxX = (origin_col + pt.x / cell_cm) * cellPx;
            const pxY = (origin_row - pt.y / cell_cm) * cellPx;
            if (i === 0) mapCtx.moveTo(pxX, pxY);
            else mapCtx.lineTo(pxX, pxY);
        }
        mapCtx.stroke();
    }

    // Draw robot position
    if (_latestPose) {
        const pxPerCm = cellPx / cell_cm;
        const robotPxX = (origin_col + _latestPose.x_cm / cell_cm) * cellPx;
        const robotPxY = (origin_row - _latestPose.y_cm / cell_cm) * cellPx;
        const theta = _latestPose.theta_rad;
        const r = cellPx * 0.7;

        mapCtx.save();
        mapCtx.translate(robotPxX, robotPxY);
        mapCtx.rotate(-theta);  // canvas Y is flipped

        // Draw filled circle body
        mapCtx.beginPath();
        mapCtx.arc(0, 0, r * 0.6, 0, Math.PI * 2);
        mapCtx.fillStyle = '#008184';
        mapCtx.fill();

        // Draw heading arrow
        mapCtx.beginPath();
        mapCtx.moveTo(0, 0);
        mapCtx.lineTo(r, 0);
        mapCtx.strokeStyle = '#ffffff';
        mapCtx.lineWidth = 2;
        mapCtx.stroke();

        mapCtx.restore();
    }

    // Update coverage badge
    coveragePct.textContent = `${coverage}%`;
}

function cellColor(val) {
    if (val === CELL_WALL)     return '#263238';  // WALL — dark charcoal
    if (val === CELL_OBSTACLE) return '#b71c1c';  // OBSTACLE — deep red
    if (val === CELL_CLEANED)  return '#26a69a';  // CLEANED — teal green
    if (val === CELL_FREE)     return '#b2dfdb';  // FREE — light teal
    return '#ECF1F1';                             // UNKNOWN — light grey
}