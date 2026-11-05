#include <Arduino.h>

// Step-pin finder (interactive) - for a CNC shield whose STEP routing is in
// doubt. Enables the drivers (D8 LOW); type a pin number in the serial
// monitor (e.g. "2" + Enter) and only that pin is pulsed for 3 seconds at a
// slow, always-startable rate. The pin that makes the motor turn is the
// real STEP line for that socket.
//   pio run -e step_finder_nano -t upload && pio device monitor -e nano
//
// Only these pins can be pulsed:
//   2 3 4 5 6 7 9 10 11 12 13
// Deliberately excluded:
//   D0/D1  - Serial
//   D8     - driver ENABLE (held LOW = enabled)
//   A0-A2  - rotary encoder (its contacts short to GND -> output fight)
//   A3     - heater SSR (would switch the heater!)
//   A4/A5  - LCD I2C
// The pin goes back to INPUT after its turn, so nothing stays driven.

static const uint8_t ENABLE_PIN = 8;
static const uint8_t ALLOWED[] = {2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13};
static const unsigned long PULSE_MS = 3000;
static const unsigned int STEP_HALF_PERIOD_US = 2500;  // 200 steps/s

bool isAllowed(long pin) {
  for (uint8_t i = 0; i < sizeof(ALLOWED); i++)
    if (ALLOWED[i] == pin) return true;
  return false;
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50);
  for (uint8_t i = 0; i < sizeof(ALLOWED); i++) pinMode(ALLOWED[i], INPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // drivers enabled
  Serial.println(F("Step-pin finder. Drivers enabled (D8 LOW)."));
  Serial.println(F("Type a pin number + Enter: 2 3 4 5 6 7 9 10 11 12 13"));
}

void loop() {
  if (!Serial.available()) return;
  long pin = Serial.parseInt();
  while (Serial.available()) Serial.read();  // drop the rest of the line
  if (!isAllowed(pin)) {
    Serial.print(F("D"));
    Serial.print(pin);
    Serial.println(F(" not allowed. Use: 2 3 4 5 6 7 9 10 11 12 13"));
    return;
  }
  Serial.print(F(">>> pulsing D"));
  Serial.print(pin);
  Serial.println(F(" for 3s at 200 steps/s..."));
  pinMode(pin, OUTPUT);
  unsigned long start = millis();
  while (millis() - start < PULSE_MS) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(STEP_HALF_PERIOD_US);
    digitalWrite(pin, LOW);
    delayMicroseconds(STEP_HALF_PERIOD_US);
  }
  pinMode(pin, INPUT);
  Serial.println(F("done. Next pin?"));
}
