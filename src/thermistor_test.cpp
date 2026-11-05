#include <Arduino.h>

// Minimal standalone ADC test - isolates the thermistor wiring from
// everything else (stepper, LCD, menu, watchdog). Build/upload with the
// "thermistor_test" PlatformIO environment, e.g.:
//   pio run -e thermistor_test -t upload
//   pio device monitor -b 9600
//
// Wiring: 5V -> 4.7k resistor -> THERMISTOR_PIN (A6, Nano only) -> NTC -> GND.

static const uint8_t THERMISTOR_PIN = A6;

void setup() {
  Serial.begin(9600);
  Serial.println(F("Thermistor ADC test starting..."));
  Serial.println(F("adc should read ~950-990 at room temp with a 100k NTC + 4.7k divider."));
  Serial.println(F("adc near 0   = likely short / no resistor / wrong pin"));
  Serial.println(F("adc near 1023 = likely open circuit / NTC not connected"));
}

void loop() {
  int adc = analogRead(THERMISTOR_PIN);
  float voltage = adc * 5.0f / 1023.0f;
  Serial.print(F("adc="));
  Serial.print(adc);
  Serial.print(F(" volts="));
  Serial.println(voltage, 3);
  delay(500);
}
