// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

#include <Arduino_RouterBridge.h>

/**
 * ARIA DASHBOARD - COPROCESSOR SKETCH
 * 
 * NOTE: Since the XRP is plugged into the Uno Q via the USB Type-C port,
 * the serial communication (/dev/ttyACM0) is handled entirely by the 
 * Linux Python script (main.py) running on the host! 
 * 
 * This sketch simply sits idle, but provides a `set_led_state` RPC endpoint 
 * in case you want to blink the Uno Q's onboard LED from Python later.
 */

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // LED OFF

    Bridge.begin();
    Bridge.provide("set_led_state", set_led_state);
}

void loop() {}

void set_led_state(bool state) {
    // LOW state means LED is ON
    digitalWrite(LED_BUILTIN, state ? LOW : HIGH);
}
