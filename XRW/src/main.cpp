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


void onEncL() {
    bool chB = (digitalRead(ENC_L_B) == HIGH);
    encL += chB ? 1 : -1;   // was -1 : 1
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
                setMotors(left, right);
            }
        } else if (cmd == "R" || cmd.startsWith("R,")) {
            noInterrupts();
            encL = 0;
            encR = 0;
            interrupts();
            Serial.println("R,OK");
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