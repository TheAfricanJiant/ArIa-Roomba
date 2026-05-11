/**
 * XRP Controller Board – I2C Pin Finder + LSM6DSOX IMU
 *
 *  *** READ THIS FIRST ***
 *
 *  The IMU is powered from the XRP's MAIN POWER RAIL, not directly from USB.
 *  The main rail goes through the POWER SWITCH on the board.
 *
 *  ► Flip the XRP power switch ON while USB is connected.
 *
 *  Without this, the I2C bus is completely empty regardless of pins tried.
 *
 *  If power is already on and nothing shows up, this sketch performs a
 *  brute-force scan across every valid I2C pin pair on the RP2040.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

// ── After running the scanner, fill these in and set SCAN_MODE false ────────
#define SCAN_MODE       true    // true = scan all pins;  false = run IMU

#define IMU_SDA_PIN     4       // fill in after scan finds the right pins
#define IMU_SCL_PIN     5
#define USE_WIRE1       false   // true if IMU is on Wire1 (I2C1)

#define PRINT_INTERVAL_MS  200

Adafruit_LSM6DSOX imu;

// ── Valid I2C pin pairs for RP2040 ───────────────────────────────────────────
struct PinPair { uint8_t sda, scl, bus; };
const PinPair PAIRS[] = {
    // I2C0 (Wire)
    { 0,  1, 0}, { 4,  5, 0}, { 8,  9, 0},
    {12, 13, 0}, {16, 17, 0}, {20, 21, 0},
    // I2C1 (Wire1)
    { 2,  3, 1}, { 6,  7, 1}, {10, 11, 1},
    {14, 15, 1}, {18, 19, 1}, {26, 27, 1},
};
const size_t NUM_PAIRS = sizeof(PAIRS) / sizeof(PAIRS[0]);
const uint8_t HUNT_ADDRS[] = {0x6A, 0x6B};

void runScanner();
void runIMU(uint8_t sda, uint8_t scl, bool useWire1);
void configureIMU();
void printSensorEvent(sensors_event_t &a, sensors_event_t &g, sensors_event_t &t);

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    while (!Serial) {
        digitalWrite(LED_BUILTIN, HIGH); delay(150);
        digitalWrite(LED_BUILTIN, LOW);  delay(150);
    }
    digitalWrite(LED_BUILTIN, LOW);

    Serial.println("\n====================================");
    Serial.println("  XRP I2C Pin Scanner / IMU Reader  ");
    Serial.println("====================================\n");

    if (SCAN_MODE) {
        Serial.println("IMPORTANT: Make sure the XRP POWER SWITCH is ON!");
        Serial.println("The IMU has no power without it, even when USB is plugged in.\n");
        runScanner();
    } else {
        runIMU(IMU_SDA_PIN, IMU_SCL_PIN, USE_WIRE1);
    }
}

void loop() {
    if (SCAN_MODE) {
        delay(5000);
        Serial.println("\n--- Re-scanning (flip power switch then wait) ---\n");
        runScanner();
        return;
    }
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
        lastPrint = millis();
        sensors_event_t accel, gyro, temp;
        imu.getEvent(&accel, &gyro, &temp);
        printSensorEvent(accel, gyro, temp);
    }
}

w
            }
        }
        w.end();
        delay(5);
    }

    if (!anyFound) {
        Serial.println("  Nothing found.");
        Serial.println("  → Is the XRP power switch ON?");
        Serial.println("  → Are batteries installed or external 5V on the barrel jack?");
    }
}

void runIMU(uint8_t sda, uint8_t scl, bool useWire1) {
    TwoWire &bus = useWire1 ? Wire1 : Wire;
    bus.setSDA(sda); bus.setSCL(scl); bus.begin();

    Serial.print("Initialising IMU on SDA=GP"); Serial.print(sda);
    Serial.print(" SCL=GP"); Serial.println(scl);

    if (!imu.begin_I2C(0x6A, &bus) && !imu.begin_I2C(0x6B, &bus)) {
        Serial.println("[ERROR] Not found! Run SCAN_MODE=true first.");
        while (true) delay(1000);
    }
    Serial.println("[OK] IMU found!\n");
    configureIMU();
}

void configureIMU() {
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
    Serial.println("  Config: Accel ±2g @ 104Hz | Gyro ±250dps @ 104Hz\n");
    Serial.println("  Time(ms) | Ax(m/s²)  Ay(m/s²)  Az(m/s²) | Gx Gy Gz (rad/s) | Temp(°C)");
    Serial.println("  ---------+-----------------------------------------+------------------+---------");
}

void printSensorEvent(sensors_event_t &accel, sensors_event_t &gyro, sensors_event_t &temp) {
    char buf[120];
    snprintf(buf, sizeof(buf),
             "  %8lu | %8.4f  %8.4f  %8.4f | %8.4f %8.4f %8.4f | %6.2f",
             millis(),
             accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
             gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, temp.temperature);
    Serial.println(buf);
}