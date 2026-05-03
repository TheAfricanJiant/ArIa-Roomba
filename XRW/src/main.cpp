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

void printStatus(unsigned long elapsedMs, long deltaL, long deltaR) {
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);
  printSerialf("  %6lu ms │ L=%5ld R=%5ld │ A=%6.2f,%6.2f,%6.2f │ G=%6.2f,%6.2f,%6.2f │ T=%5.1f\n",
               elapsedMs,
               deltaL,
               deltaR,
               accel.acceleration.x,
               accel.acceleration.y,
               accel.acceleration.z,
               gyro.gyro.x,
               gyro.gyro.y,
               gyro.gyro.z,
               temp.temperature);
}

void runStep(const char* label, int l, int r) {
  long startL = encL;
  long startR = encR;
  unsigned long t0 = millis();
  unsigned long nextPrint = t0;

  printSerialf("\n[%s] L=%d R=%d\n", label, l, r);
  setMotors(l, r);

  while (millis() - t0 < STEP_DURATION_MS) {
    unsigned long now = millis();
    if (now >= nextPrint) {
      printStatus(now - t0, encL - startL, encR - startR);
      nextPrint += STATUS_INTERVAL_MS;
    }
  }

  stopMotors();
  printSerialf("DONE L=%ld R=%ld\n", encL - startL, encR - startR);
  delay(300);
}

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
  runStep("FORWARD", TEST_SPEED, TEST_SPEED * 3 / 4);
  runStep("BACK", -TEST_SPEED, -TEST_SPEED * 3 / 4);
  runStep("SPIN L", -TEST_SPEED, TEST_SPEED);
  runStep("SPIN R", TEST_SPEED, -TEST_SPEED);

  Serial.println("=== CYCLE COMPLETE ===\n");
  delay(2000);
}
