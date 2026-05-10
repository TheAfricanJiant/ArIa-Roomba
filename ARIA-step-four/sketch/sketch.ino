// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

#include <Arduino_RouterBridge.h>

/**
 * ARIA DASHBOARD - COPROCESSOR SKETCH
 * 
 * The XRP robot controller uses its own serial link over USB-C.
 * This sketch is for UNO Q local I/O controlled from Linux via RouterBridge.
 *
 * Vacuum motor is wired to an L298N:
 *   ENB -> D10 (PWM)
 *   IN3 -> D11
 *   IN4 -> D9
 *
 * Linux side should call:
 *   Bridge.call("set_vacuum_pwm", pwm_0_255)
 */

// L298N vacuum pins
static const int VAC_ENB = 10;  // PWM
static const int VAC_IN3 = 11;  // direction
static const int VAC_IN4 = 9;   // direction

static int _vac_pwm = 0;

static int clamp_pwm(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return v;
}

void apply_vacuum(int pwm) {
    _vac_pwm = clamp_pwm(pwm);

    // Choose a fixed direction for the vacuum motor (suction).
    // If your motor spins the wrong way, swap IN3/IN4 logic.
    digitalWrite(VAC_IN3, HIGH);
    digitalWrite(VAC_IN4, LOW);

    analogWrite(VAC_ENB, _vac_pwm);
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // LED OFF

    pinMode(VAC_ENB, OUTPUT);
    pinMode(VAC_IN3, OUTPUT);
    pinMode(VAC_IN4, OUTPUT);
    apply_vacuum(0); // vacuum OFF at boot

    Bridge.begin();
    Bridge.provide("set_led_state", set_led_state);
    Bridge.provide("set_vacuum_pwm", set_vacuum_pwm);
}

void loop() {}

void set_led_state(bool state) {
    // LOW state means LED is ON
    digitalWrite(LED_BUILTIN, state ? LOW : HIGH);
}

void set_vacuum_pwm(int pwm) {
    apply_vacuum(pwm);
}
