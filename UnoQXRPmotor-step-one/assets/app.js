const powerBtn = document.getElementById('motor-power-btn');
const powerText = document.getElementById('power-text');
const speedSlider = document.getElementById('speed-slider');
const speedVal = document.getElementById('speed-val');

const encL = document.getElementById('enc-l');
const encR = document.getElementById('enc-r');
const imuAccel = document.getElementById('imu-accel');
const imuGyro = document.getElementById('imu-gyro');

const socket = io(`http://${window.location.host}`);

document.addEventListener('DOMContentLoaded', () => {
    socket.on('connect', () => {
        socket.emit('get_initial_state', {});
    });

    socket.on('state_update', (state) => {
        updateUI(state);
    });

    socket.on('telemetry_update', (data) => {
        updateTelemetry(data);
    });

    powerBtn.addEventListener('click', () => {
        socket.emit('toggle_power', {});
    });

    speedSlider.addEventListener('input', (e) => {
        speedVal.textContent = e.target.value;
    });

    speedSlider.addEventListener('change', (e) => {
        socket.emit('set_speed', { speed: parseInt(e.target.value) });
    });
});

function updateUI(state) {
    const isOn = state.motors_on;
    powerBtn.className = isOn ? 'power-on' : 'power-off';
    powerText.textContent = isOn ? 'MOTORS ON' : 'MOTORS OFF';
    
    speedSlider.value = state.speed;
    speedVal.textContent = state.speed;
}

function updateTelemetry(data) {
    encL.textContent = data.enc_l;
    encR.textContent = data.enc_r;
    imuAccel.textContent = `X: ${data.accel_x.toFixed(2)} | Y: ${data.accel_y.toFixed(2)} | Z: ${data.accel_z.toFixed(2)}`;
    imuGyro.textContent = `X: ${data.gyro_x.toFixed(2)} | Y: ${data.gyro_y.toFixed(2)} | Z: ${data.gyro_z.toFixed(2)}`;
}