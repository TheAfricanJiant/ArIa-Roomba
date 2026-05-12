// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

#include <Arduino_RouterBridge.h>
#include <Servo.h>

/**
 * ARIA DASHBOARD — UNO Q COPROCESSOR SKETCH
 *
 * All sensor I/O is handled here (Linux talks via RouterBridge RPC).
 *
 * Vacuum motor (L298N):
 *   ENB → D10 (PWM)    IN3 → D11    IN4 → D9
 *
 * Ultrasonic sensors (HC-SR04):
 *   Front:  Trig → D4   Echo → D5
 *   Right:  Trig → D7   Echo → D8
 *   Left:   Trig → D3   Echo → D2
 *
 * Cleaning servo (360° continuous):
 *   Signal → D6
 *   Speed -100…+100 (0 = stop, negative = CW, positive = CCW)
 *
 * Python Bridge RPC calls available:
 *   Bridge.call("set_vacuum_pwm", 0-255)
 *   Bridge.call("set_brush_servo", -100..100)
 *   Bridge.call("set_led_state", 0/1)
 *   Bridge.call("get_us_front")  → float cm
 *   Bridge.call("get_us_right")  → float cm
 *   Bridge.call("get_us_left")   → float cm
 *   Bridge.call("reset_encoders_ack", 1)  ← notification only
 */

// ── Vacuum pins ──────────────────────────────────────────────────────────────
static const int VAC_ENB = 10;
static const int VAC_IN3 = 11;
static const int VAC_IN4 = 9;

// ── Ultrasonic pins ──────────────────────────────────────────────────────────
static const int US_F_TRIG = 4;  static const int US_F_ECHO = 5;
static const int US_R_TRIG = 7;  static const int US_R_ECHO = 8;
static const int US_L_TRIG = 3;  static const int US_L_ECHO = 2;

// ── Servo pin ────────────────────────────────────────────────────────────────
static const int SERVO_PIN = 6;

// ── Globals ──────────────────────────────────────────────────────────────────
Servo cleanServo;

// Cached ultrasonic readings (cm), updated every loop iteration
volatile float _us_front = 999.0f;
volatile float _us_right = 999.0f;
volatile float _us_left  = 999.0f;

static unsigned long _lastUS = 0;
static const unsigned long US_INTERVAL_MS = 60;  // measure all 3 every 60ms


// ── Ultrasonic helper ─────────────────────────────────────────────────────────
float measureCM(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long us = pulseIn(echoPin, HIGH, 25000UL);  // 25ms timeout ≈ 430cm max
    if (us == 0) return 999.0f;                 // no echo = out of range
    return us / 58.0f;                          // convert to cm
}


// ── Vacuum control ───────────────────────────────────────────────────────────
static int clamp_pwm(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }

void apply_vacuum(int pwm) {
    int p = clamp_pwm(pwm);
    digitalWrite(VAC_IN3, HIGH);
    digitalWrite(VAC_IN4, LOW);
    analogWrite(VAC_ENB, p);
}

// ── Servo control ─────────────────────────────────────────────────────────────
// 360° continuous servo: 1500µs = stop, 1000µs = full CW, 2000µs = full CCW
void apply_servo(int speed) {
    // speed: -100 (full CW) … 0 (stop) … +100 (full CCW)
    speed = speed < -100 ? -100 : (speed > 100 ? 100 : speed);
    int us = 1500 + (speed * 5);  // maps ±100 → ±500µs around 1500
    cleanServo.writeMicroseconds(us);
}


// ── Bridge RPC handlers ──────────────────────────────────────────────────────
void set_vacuum_pwm(int pwm)     { apply_vacuum(pwm); }
void set_brush_servo(int speed)  { apply_servo(speed); }
void set_led_state(bool state)   { digitalWrite(LED_BUILTIN, state ? LOW : HIGH); }

float get_us_front() { return _us_front; }
float get_us_right() { return _us_right; }
float get_us_left()  { return _us_left;  }


// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    // LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // HIGH = OFF on UNO Q

    // Vacuum motor
    pinMode(VAC_ENB, OUTPUT);
    pinMode(VAC_IN3, OUTPUT);
    pinMode(VAC_IN4, OUTPUT);
    apply_vacuum(0);

    // Ultrasonic sensor pins
    pinMode(US_F_TRIG, OUTPUT); pinMode(US_F_ECHO, INPUT);
    pinMode(US_R_TRIG, OUTPUT); pinMode(US_R_ECHO, INPUT);
    pinMode(US_L_TRIG, OUTPUT); pinMode(US_L_ECHO, INPUT);

    // Servo
    cleanServo.attach(SERVO_PIN);
    apply_servo(0);  // stop on boot

    // Register Bridge RPCs
    Bridge.begin();
    Bridge.provide("set_vacuum_pwm",   set_vacuum_pwm);
    Bridge.provide("set_brush_servo",  set_brush_servo);
    Bridge.provide("set_led_state",    set_led_state);
    Bridge.provide("get_us_front",     get_us_front);
    Bridge.provide("get_us_right",     get_us_right);
    Bridge.provide("get_us_left",      get_us_left);
}


// ── Loop — continuously refresh ultrasonic cache ──────────────────────────────
void loop() {
    unsigned long now = millis();
    if (now - _lastUS >= US_INTERVAL_MS) {
        _lastUS = now;
        // Measure sequentially (each ~20ms max due to 25ms timeout)
        _us_front = measureCM(US_F_TRIG, US_F_ECHO);
        _us_right = measureCM(US_R_TRIG, US_R_ECHO);
        _us_left  = measureCM(US_L_TRIG, US_L_ECHO);
    }
}
