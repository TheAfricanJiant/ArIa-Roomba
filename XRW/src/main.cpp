/**
 * XRP Controller – Motors + Encoders + IMU
 *
 * Receives commands over serial (via socat TCP bridge on port 5000):
 *   M,<left>,<right>\n   — set motor speeds (-255..255)
 *   R\n                   — reset encoder counts to zero
 *
 * Publishes telemetry at 100ms:
 *   T,encL,encR,ax,ay,az,gx,gy,gz\n
 *
 * Motor direction:
 *   Positive values = forward for that wheel.
 *
 * Hardware note:
 *   Left motor is on Motor 4 port — original Motor L port DRV8835
 *   was damaged by a shorted motor cable.
 *   Right motor is on Motor R port.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <stdarg.h>

// Left motor — Motor 4 port
#define MOTOR_L_PH  10
#define MOTOR_L_EN  11
#define ENC_L_A      8
#define ENC_L_B      9

// Right motor — Motor R port
#define MOTOR_R_PH  14
#define MOTOR_R_EN  15
#define ENC_R_A     12
#define ENC_R_B     13

#define IMU_SDA_PIN      18
#define IMU_SCL_PIN      19
#define IMU_I2C_ADDRESS  0x6B

// ── Direction invert flags ───────────────────────────────────────────────────
#define MOTOR_L_INVERT true
#define MOTOR_R_INVERT true

// ── Encoders ─────────────────────────────────────────────────────────────────
volatile long encL = 0;
volatile long encR = 0;

enum DriveMode {
    DRIVE_RAW_PWM,
    DRIVE_AUTO_VELOCITY
};

DriveMode driveMode = DRIVE_RAW_PWM;

// M commands stay raw for manual driving; A commands close wheel speed with encoders.
const float AUTO_MAX_TICKS_PER_SEC = 1200.0f;
const float AUTO_KP = 0.08f;
const float AUTO_KI = 0.18f;
const int   AUTO_MIN_PWM = 42;
const unsigned long AUTO_CONTROL_INTERVAL_MS = 50;
const unsigned long AUTO_COMMAND_TIMEOUT_MS  = 450;
// AUTO_ENC_L_SIGN = -1: the Motor 4 port + MOTOR_L_INVERT=true combination means
// that when a positive A, target is commanded the left encoder counts DOWN.
// The -1 corrects measuredLps back to positive so the PI loop is stable.
// Do NOT change this — the firmware was correct and working.
const int AUTO_ENC_L_SIGN = -1;
const int AUTO_ENC_R_SIGN = 1;

float targetLps = 0.0f;
float targetRps = 0.0f;
float integralL = 0.0f;
float integralR = 0.0f;
int autoPwmL = 0;
int autoPwmR = 0;
float measuredLps = 0.0f;
float measuredRps = 0.0f;
long lastControlEncL = 0;
long lastControlEncR = 0;
unsigned long lastControlTime = 0;
unsigned long lastAutoCommandTime = 0;


void onEncL() {
    bool chB = (digitalRead(ENC_L_B) == HIGH);
    encL += chB ? -1 : 1;
}

void onEncR() {
    bool chB = (digitalRead(ENC_R_B) == HIGH);
    encR += chB ? 1 : -1;
}


// ── IMU ───────────────────────────────────────────────────────────────────────
Adafruit_LSM6DSOX imu;

// ── Helpers ───────────────────────────────────────────────────────────────────
void printSerialf(const char* fmt, ...) {
    char buf[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
}

// ── Motor control ─────────────────────────────────────────────────────────────
void setMotors(int l, int r) {
    l = constrain(l, -255, 255);
    r = constrain(r, -255, 255);

    if (MOTOR_L_INVERT) l = -l;
    if (MOTOR_R_INVERT) r = -r;

    digitalWrite(MOTOR_L_PH, l >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_R_PH, r >= 0 ? HIGH : LOW);
    analogWrite(MOTOR_L_EN, abs(l));
    analogWrite(MOTOR_R_EN, abs(r));
}

void stopMotors() {
    analogWrite(MOTOR_L_EN, 0);
    analogWrite(MOTOR_R_EN, 0);
}

void resetAutoController() {
    noInterrupts();
    lastControlEncL = encL;
    lastControlEncR = encR;
    interrupts();
    integralL = 0.0f;
    integralR = 0.0f;
    autoPwmL = 0;
    autoPwmR = 0;
    measuredLps = 0.0f;
    measuredRps = 0.0f;
    lastControlTime = millis();
}

int applyMinPwm(float cmd, float target) {
    int pwm = (int)roundf(cmd);
    pwm = constrain(pwm, -255, 255);
    if (target != 0.0f && abs(pwm) < AUTO_MIN_PWM) {
        pwm = target > 0.0f ? AUTO_MIN_PWM : -AUTO_MIN_PWM;
    }
    return pwm;
}

void setAutoTargetsFromBasis(int leftBasis, int rightBasis) {
    leftBasis = constrain(leftBasis, -255, 255);
    rightBasis = constrain(rightBasis, -255, 255);

    targetLps = (leftBasis / 255.0f) * AUTO_MAX_TICKS_PER_SEC;
    targetRps = (rightBasis / 255.0f) * AUTO_MAX_TICKS_PER_SEC;
    lastAutoCommandTime = millis();

    if (driveMode != DRIVE_AUTO_VELOCITY) {
        driveMode = DRIVE_AUTO_VELOCITY;
        resetAutoController();
    }

    if (leftBasis == 0 && rightBasis == 0) {
        targetLps = 0.0f;
        targetRps = 0.0f;
        resetAutoController();
        stopMotors();
    }
}

void updateAutoVelocity() {
    if (driveMode != DRIVE_AUTO_VELOCITY) return;

    unsigned long now = millis();
    if (now - lastAutoCommandTime > AUTO_COMMAND_TIMEOUT_MS) {
        targetLps = 0.0f;
        targetRps = 0.0f;
        resetAutoController();
        stopMotors();
        return;
    }

    if (now - lastControlTime < AUTO_CONTROL_INTERVAL_MS) return;

    float dt = (now - lastControlTime) / 1000.0f;
    if (dt <= 0.0f) return;

    long currentL;
    long currentR;
    noInterrupts();
    currentL = encL;
    currentR = encR;
    interrupts();

    measuredLps = AUTO_ENC_L_SIGN * (currentL - lastControlEncL) / dt;
    measuredRps = AUTO_ENC_R_SIGN * (currentR - lastControlEncR) / dt;
    lastControlEncL = currentL;
    lastControlEncR = currentR;
    lastControlTime = now;

    float errorL = targetLps - measuredLps;
    float errorR = targetRps - measuredRps;
    integralL = constrain(integralL + errorL * dt, -600.0f, 600.0f);
    integralR = constrain(integralR + errorR * dt, -600.0f, 600.0f);

    if (targetLps == 0.0f) integralL = 0.0f;
    if (targetRps == 0.0f) integralR = 0.0f;

    float ffL = (targetLps / AUTO_MAX_TICKS_PER_SEC) * 255.0f;
    float ffR = (targetRps / AUTO_MAX_TICKS_PER_SEC) * 255.0f;
    autoPwmL = applyMinPwm(ffL + AUTO_KP * errorL + AUTO_KI * integralL, targetLps);
    autoPwmR = applyMinPwm(ffR + AUTO_KP * errorR + AUTO_KI * integralR, targetRps);

    if (targetLps == 0.0f) autoPwmL = 0;
    if (targetRps == 0.0f) autoPwmR = 0;
    setMotors(autoPwmL, autoPwmR);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
unsigned long lastTelemetryTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    pinMode(MOTOR_L_PH, OUTPUT);
    pinMode(MOTOR_L_EN, OUTPUT);
    pinMode(MOTOR_R_PH, OUTPUT);
    pinMode(MOTOR_R_EN, OUTPUT);

    analogWriteFreq(20000);

    stopMotors();

    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), onEncL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), onEncR, RISING);
    resetAutoController();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Wire1.setSDA(IMU_SDA_PIN);
    Wire1.setSCL(IMU_SCL_PIN);
    Wire1.begin();

    Serial.print("LSM6DSOX init ... ");
    if (!imu.begin_I2C(IMU_I2C_ADDRESS, &Wire1)) {
        Serial.println("FAILED");
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH); delay(200);
            digitalWrite(LED_BUILTIN, LOW);  delay(200);
        }
    }
    Serial.println("OK");

    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
    // ── Handle incoming commands ──
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd.startsWith("M,")) {
            int c1 = cmd.indexOf(',');
            int c2 = cmd.indexOf(',', c1 + 1);
            if (c1 != -1 && c2 != -1) {
                int left  = cmd.substring(c1 + 1, c2).toInt();
                int right = cmd.substring(c2 + 1).toInt();
                driveMode = DRIVE_RAW_PWM;
                targetLps = 0.0f;
                targetRps = 0.0f;
                resetAutoController();
                setMotors(left, right);
            }
        } else if (cmd.startsWith("A,")) {
            int c1 = cmd.indexOf(',');
            int c2 = cmd.indexOf(',', c1 + 1);
            if (c1 != -1 && c2 != -1) {
                int left  = cmd.substring(c1 + 1, c2).toInt();
                int right = cmd.substring(c2 + 1).toInt();
                setAutoTargetsFromBasis(left, right);
            }
        } else if (cmd == "R" || cmd.startsWith("R,")) {
            noInterrupts();
            encL = 0;
            encR = 0;
            interrupts();
            resetAutoController();
            Serial.println("R,OK");
        }
    }

    // ── Publish telemetry every 100ms ──
    updateAutoVelocity();

    unsigned long now = millis();
    if (now - lastTelemetryTime >= 100) {
        lastTelemetryTime = now;

        sensors_event_t accel, gyro, temp;
        imu.getEvent(&accel, &gyro, &temp);

        printSerialf("T,%ld,%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%d,%d\n",
                     encL, encR,
                     accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                     gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
                     measuredLps, measuredRps,
                     autoPwmL, autoPwmR,
                     driveMode == DRIVE_AUTO_VELOCITY ? 1 : 0);
    }
}
