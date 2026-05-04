/**
 * XRP Controller – IMU + Encoder Status Monitor
 * Combines LSM6DSOX IMU data with wheel encoder counts.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <stdarg.h>

#define MOTOR_L_PH 6
#define MOTOR_L_EN 7
#define MOTOR_R_PH 14
#define MOTOR_R_EN 15
#define ENC_L_A 4
#define ENC_R_A 12

#define IMU_SDA_PIN       18
#define IMU_SCL_PIN       19
#define IMU_I2C_ADDRESS   0x6B      // SA0 pulled high on XRP Beta board
#define STEP_DURATION_MS  1500
#define STATUS_INTERVAL_MS 200
#define TEST_SPEED        160

volatile long encL = 0;
volatile long encR = 0;

Adafruit_LSM6DSOX imu;

void onEncL() { encL++; }
void onEncR() { encR++; }

void printSerialf(const char* fmt, ...) {
  char buf[180];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
}

void setMotors(int l, int r) {
  digitalWrite(MOTOR_L_PH, l >= 0 ? HIGH : LOW);
  analogWrite(MOTOR_L_EN, abs(l));
  digitalWrite(MOTOR_R_PH, r >= 0 ? LOW : HIGH);
  analogWrite(MOTOR_R_EN, abs(r));
}

void stopMotors() {
  analogWrite(MOTOR_L_EN, 0);
  analogWrite(MOTOR_R_EN, 0);
}

unsigned long lastTelemetryTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  pinMode(MOTOR_L_PH, OUTPUT);
  pinMode(MOTOR_L_EN, OUTPUT);
  pinMode(MOTOR_R_PH, OUTPUT);
  pinMode(MOTOR_R_EN, OUTPUT);
  stopMotors();

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onEncR, RISING);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("=== XRP IMU + Encoder Monitor ===\n");

  Wire1.setSDA(IMU_SDA_PIN);
  Wire1.setSCL(IMU_SCL_PIN);
  Wire1.begin();

  Serial.print("Connecting to LSM6DSOX on Wire1 (GP18/GP19) at 0x6B ... ");
  if (!imu.begin_I2C(IMU_I2C_ADDRESS, &Wire1)) {
    Serial.println("FAILED\n");
    Serial.println("Check that the XRP power switch is ON and the board is powered.");
    while (true) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }
  Serial.println("OK\n");

  imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

  Serial.println("  Accel : ±2 g @ 104 Hz");
  Serial.println("  Gyro  : ±250 °/s @ 104 Hz\n");
}

void loop() {
  // 1. Check for incoming commands from Uno Q via USB
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("M,")) {
      int comma1 = cmd.indexOf(',');
      int comma2 = cmd.indexOf(',', comma1 + 1);
      if (comma1 != -1 && comma2 != -1) {
        int left = cmd.substring(comma1 + 1, comma2).toInt();
        int right = cmd.substring(comma2 + 1).toInt();
        setMotors(left, right);
      }
    }
  }

  // 2. Publish telemetry every 100ms
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
