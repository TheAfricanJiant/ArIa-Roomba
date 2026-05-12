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
 *   Set MOTOR_L_INVERT / MOTOR_R_INVERT to true if a motor runs backwards.
 *   Positive values = forward for that wheel.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <stdarg.h>

#define MOTOR_L_PH 6
#define MOTOR_L_EN 7
#define MOTOR_R_PH 14
#define MOTOR_R_EN 15
#define ENC_L_A    4
#define ENC_R_A    12

#define IMU_SDA_PIN      18
#define IMU_SCL_PIN      19
#define IMU_I2C_ADDRESS  0x6B

// ── Direction invert flags ───────────────────────────────────────────────────
// Set to true if that motor runs backwards when commanded forward.
// Flip MOTOR_L_INVERT if left motor is wrong; MOTOR_R_INVERT for right.
#define MOTOR_L_INVERT true
#define MOTOR_R_INVERT true

// ── Encoders ─────────────────────────────────────────────────────────────────
// Fix (2026-05): ISRs read the actual phase-pin state at interrupt time rather
// than relying on the dirL/dirR globals written by setMotors().  This gives the
// correct sign during deceleration, back-EMF roll-back, or external pushes.
volatile long encL = 0;
volatile long encR = 0;

void onEncL() { encL += (digitalRead(MOTOR_L_PH) == HIGH) ? 1 : -1; }
void onEncR() { encR += (digitalRead(MOTOR_R_PH) == HIGH) ? 1 : -1; }

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

    // Apply inversion flags
    if (MOTOR_L_INVERT) l = -l;
    if (MOTOR_R_INVERT) r = -r;

    // Write phase pin first; ISRs now read the pin directly so no dirL/dirR needed.
    digitalWrite(MOTOR_L_PH, l >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_R_PH, r >= 0 ? HIGH : LOW);
    analogWrite(MOTOR_L_EN, abs(l));
    analogWrite(MOTOR_R_EN, abs(r));
}

void stopMotors() {
    analogWrite(MOTOR_L_EN, 0);
    analogWrite(MOTOR_R_EN, 0);
    // Phase pins left in their last state so ISRs still sign ticks correctly
    // during any brief coast after the enable goes low.
}

// ── Setup ─────────────────────────────────────────────────────────────────────
unsigned long lastTelemetryTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    pinMode(MOTOR_L_PH, OUTPUT); pinMode(MOTOR_L_EN, OUTPUT);
    pinMode(MOTOR_R_PH, OUTPUT); pinMode(MOTOR_R_EN, OUTPUT);
    stopMotors();

    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), onEncL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), onEncR, RISING);

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
            // M,left,right
            int c1 = cmd.indexOf(',');
            int c2 = cmd.indexOf(',', c1 + 1);
            if (c1 != -1 && c2 != -1) {
                int left  = cmd.substring(c1 + 1, c2).toInt();
                int right = cmd.substring(c2 + 1).toInt();
                setMotors(left, right);
            }
        } else if (cmd == "R" || cmd.startsWith("R,")) {
            // Encoder reset — zero both counters
            noInterrupts();
            encL = 0;
            encR = 0;
            interrupts();
            Serial.println("R,OK");  // ack
        }
    }

    // ── Publish telemetry every 100ms ──
    unsigned long now = millis();
    if (now - lastTelemetryTime >= 100) {
        lastTelemetryTime = now;

        sensors_event_t accel, gyro, temp;
        imu.getEvent(&accel, &gyro, &temp);

        printSerialf("T,%ld,%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                     encL, encR,
                     accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                     gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    }
}
