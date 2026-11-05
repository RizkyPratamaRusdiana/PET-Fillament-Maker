#include <Arduino.h>

// Minimal standalone SSR/MOSFET heater output test - isolates the heater
// switching from everything else (stepper, LCD, menu, watchdog, PID).
// Build/upload with:
//   pio run -e ssr_test -t upload
//   pio device monitor -b 9600
//
// Output starts OFF and stays OFF until you explicitly send '1'.
// Send '1' -> turn output ON (full logic HIGH to the SSR/MOSFET signal pin)
// Send '0' -> turn output OFF
//
// Watch the SSR module's own indicator LED (most modules have one) and/or
// measure continuity/voltage on its load side to confirm it actually
// switches the heater circuit.

static const uint8_t HEATER_PWM_PIN = A3; // CoolEn header

void setup() {
  pinMode(HEATER_PWM_PIN, OUTPUT);
  digitalWrite(HEATER_PWM_PIN, LOW); // start OFF - never default to on
  Serial.begin(9600);
  Serial.println(F("SSR/MOSFET test starting. Output is OFF."));
  Serial.println(F("Send '1' to turn ON, '0' to turn OFF."));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      digitalWrite(HEATER_PWM_PIN, HIGH);
      Serial.println(F("Heater output: ON"));
    } else if (c == '0') {
      digitalWrite(HEATER_PWM_PIN, LOW);
      Serial.println(F("Heater output: OFF"));
    }
  }
}
