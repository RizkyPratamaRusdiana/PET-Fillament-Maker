#include <Arduino.h>

// Pin probe - finds which Nano pin each shield header (e.g. Abort / Hold /
// Resume) is really wired to. Every pin is an INPUT_PULLUP; any level change
// is printed. Turn / press the encoder and watch which pins toggle.
//   pio run -e pin_probe_nano -t upload && pio device monitor -e nano
// D0/D1 are Serial and are skipped. A6/A7 are analog-only (read via ADC,
// no internal pull-up, so they may float if nothing drives them).

static const uint8_t PINS[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
                               A0, A1, A2, A3, A4, A5};
static const char *const NAMES[] = {"D2", "D3", "D4", "D5", "D6", "D7", "D8", "D9",
                                    "D10", "D11", "D12", "D13",
                                    "A0", "A1", "A2", "A3", "A4", "A5"};
static const uint8_t N = sizeof(PINS);
uint8_t last[N + 2];

uint8_t readAnalogPin(uint8_t ch) { return analogRead(ch) > 512 ? 1 : 0; }

void setup() {
  Serial.begin(9600);
  for (uint8_t i = 0; i < N; i++) {
    pinMode(PINS[i], INPUT_PULLUP);
  }
  delay(5);
  Serial.println(F("Pin probe. Idle levels:"));
  for (uint8_t i = 0; i < N; i++) {
    last[i] = digitalRead(PINS[i]);
    Serial.print(NAMES[i]); Serial.print('='); Serial.print(last[i]); Serial.print(' ');
  }
  last[N] = readAnalogPin(A6);
  last[N + 1] = readAnalogPin(A7);
  Serial.print(F("A6=")); Serial.print(last[N]);
  Serial.print(F(" A7=")); Serial.println(last[N + 1]);
  Serial.println(F("Turn / press the encoder now..."));
}

void loop() {
  for (uint8_t i = 0; i < N; i++) {
    uint8_t v = digitalRead(PINS[i]);
    if (v != last[i]) {
      last[i] = v;
      Serial.print(NAMES[i]); Serial.print(F(" -> ")); Serial.println(v ? F("HIGH") : F("LOW"));
    }
  }
}
