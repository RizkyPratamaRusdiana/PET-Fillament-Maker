#include <Arduino.h>
#include <util/atomic.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <stdio.h>
#include <string.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <math.h>

// ---------------------------------------------------------------------------
// PET Bottle Filament Maker - single stepper (spool winder / puller) control
//
// Hardware (either option, selected via the PlatformIO env - see platformio.ini):
//   - Arduino UNO  (ATmega328P) + CNC Shield V3   -> env:uno
//   - Arduino Nano (ATmega328P) + CNC Shield V4   -> env:nano / env:nano_new
// Only the X-axis driver socket is populated. That one stepper both pulls
// the softened PET strip and winds the finished filament onto the spool.
// Heater cartridge + E3D-style NTC thermistor are driven directly by this
// Arduino (PID + thermal runaway protection - see the "Heater / temperature
// control" section below).
//
// Pins used (X driver socket):
//   X.STEP   -> D2 on V3 (GRBL pinout), D5 on this V4 board
//   X.DIR    -> D5 on V3,               D2 on this V4 board
//   ENABLE   -> D8   (shared driver enable, active LOW)
// The V4 swap is selected by -DCNC_SHIELD_V4 in the nano envs.
// Operator panel - KY-040 rotary encoder:
//   CLK -> A0   (Abort header on the CNC shield)
//   DT  -> A1   (Hold header)
//   SW  -> A2   (Resume header)
//   +   -> 5V,  GND -> GND
// A0/A1 have no external interrupt, so the encoder is decoded by a
// pin-change interrupt (PCINT1, port C) - see "Encoder handling" below.
// Polling alone would miss detents while loop() is busy with the
// thermistor read / LCD I2C writes.
// LCD I2C uses the ATmega328P's fixed I2C bus: A4 = SDA, A5 = SCL (same on
// UNO and Nano).
// Heater:
//   Thermistor  -> A6  (analog-input-only pin, Nano only - the UNO has no
//                        A6 header; voltage divider:
//                        5V -> 4.7k resistor -> A6 -> NTC -> GND)
//   Heater/SSR  -> A3  (CoolEn header; to the MOSFET/SSR module signal
//                        input; module GND shared with Arduino GND, heater
//                        itself on its own 12V supply)
//   A3 has no hardware PWM, so the PID output is applied as slow
//   time-proportional on/off switching (HEATER_WINDOW_MS) - which also suits
//   an SSR far better than 490Hz PWM would.
// Note: microstepping is set by the shield's jumpers (MS1/MS2/MS3) - MIN/MAX
// speed below are in (micro)steps/sec, so keep the jumper setting the same
// on V3 and V4, or re-check the speeds after switching shields.
// ---------------------------------------------------------------------------

// ---- Pin definitions --------------------------------------------------
#if defined(CNC_SHIELD_V4)
// This CNC Shield V4 routes the X socket's STEP/DIR swapped vs. the GRBL
// pinout (verified on the hardware with src/step_finder.cpp: pulsing D5
// moves the motor, D2 does not). Set via build_flags in platformio.ini.
static const uint8_t X_STEP_PIN = 5;
static const uint8_t X_DIR_PIN = 2;
#define X_STEP_PORTD_BIT PD5  // D5 = PD5, used by the step ISR's direct port write
#else
static const uint8_t X_STEP_PIN = 2;   // CNC Shield V3 (GRBL pinout)
static const uint8_t X_DIR_PIN = 5;
#define X_STEP_PORTD_BIT PD2  // D2 = PD2
#endif
static const uint8_t STEPPERS_ENABLE_PIN = 8;

// CLK/DT must stay on A0/A1 (PC0/PC1) - the PCINT1 decoder below reads
// them straight from PINC bits 0 and 1.
static const uint8_t ENCODER_CLK_PIN = A0;
static const uint8_t ENCODER_DT_PIN = A1;
static const uint8_t ENCODER_SW_PIN = A2;

static const uint8_t THERMISTOR_PIN = A6;
static const uint8_t HEATER_PIN = A3;

// ---- Debug output ----------------------------------------------------------
// 1 = print encoder turns / button presses to Serial (9600 baud). Keep the
// messages short: if the 64-byte TX buffer fills, Serial.print() blocks and
// the stepper stutters. Set to 0 once the panel is confirmed working.
#define DEBUG_SERIAL 1

// ---- LCD -----------------------------------------------------------------
// Address is commonly 0x27 or 0x3F depending on the PCF8574 module; run an
// I2C scanner sketch once to confirm if the display stays blank.
static const uint8_t LCD_I2C_ADDRESS = 0x27;
LiquidCrystal_I2C lcdHw(LCD_I2C_ADDRESS, 16, 2);

// Thin wrapper: every LCD call is skipped while the display isn't detected.
// Without this, a missing/miswired LCD makes each I2C transfer run into the
// Wire timeout - one row redraw then takes seconds, stalls the stepper and
// trips the 2s watchdog (and the old Nano bootloader can then get stuck in
// a watchdog reset loop). reinitLcdIfDue() re-probes so a reconnected LCD
// comes back on its own.
struct GuardedLcd {
  bool present = false;
  void init() { if (present) lcdHw.init(); }
  void backlight() { if (present) lcdHw.backlight(); }
  void noBacklight() { if (present) lcdHw.noBacklight(); }
  void clear() { if (present) lcdHw.clear(); }
  void setCursor(uint8_t col, uint8_t row) { if (present) lcdHw.setCursor(col, row); }
  template <typename T> void print(T v) { if (present) lcdHw.print(v); }
};
GuardedLcd lcd;

bool probeLcd() {
  Wire.beginTransmission(LCD_I2C_ADDRESS);
  return Wire.endTransmission() == 0;
}

// ---- Motion tuning (adjust to match motor steps/rev, microstepping and
// spool mechanics) ----------------------------------------------------------
static const float MIN_SPEED_SPS = 50.0f;    // steps/sec
static const float MAX_SPEED_SPS = 3000.0f;  // steps/sec - verify the motor actually reaches this without skipping steps
static const float SPEED_STEP_SPS = 10.0f;   // change per encoder detent
static const float DEFAULT_SPEED_SPS = 1000.0f;

static const float MIN_ACCEL_SPS2 = 50.0f;    // steps/sec^2
static const float MAX_ACCEL_SPS2 = 2000.0f;  // steps/sec^2
static const float ACCEL_STEP_SPS2 = 50.0f;   // change per encoder detent
static const float DEFAULT_ACCELERATION_SPS2 = 300.0f;

// ---- Step generation (Timer1, CTC mode, /8 prescaler = 2MHz tick) -------
// Motion runs entirely in interrupts, so nothing in loop() (LCD I2C writes,
// thermistor reads, Serial) can disturb it:
//   - Timer1 ISR: emits one STEP pulse per compare match.
//   - Timer2 ISR (every 2ms, see rampTick()): S-curve speed ramp, hands
//     Timer1 the next step period, and finishes a stop.
// Previously the ramp ran in loop(), and a 5-10ms LCD/ADC stall right at a
// start/stop (the LCD redraws RUN/STP exactly then) turned into a speed
// jump - felt as a stutter when starting and stopping.
// 16-bit OCR1A at 2MHz -> slowest possible rate ~30.5 steps/s.
static const float TIMER1_TICK_HZ = 2000000.0f;
// Timer2 CTC, clk/128 = 125kHz, OCR2A 249 -> 500Hz ramp tick.
static const float RAMP_TICK_S = 0.002f;

// Speed the motor starts from / is stopped at (Serial: V<sps>). Low values
// crawl through the slow full-step range, where every 1.8deg step is felt
// as a "tick"; a moderate start speed (~80-150 for an unloaded motor) skips
// that range - the motor can start and stop there instantly. Too high and
// the motor stalls/buzzes at start under load. >= 31 (Timer1 floor).
static const float START_STOP_SPEED_DEFAULT_SPS = 32.0f;
static const float START_STOP_SPEED_MIN_SPS = 31.0f;
static const float START_STOP_SPEED_MAX_SPS = 400.0f;
// S-curve smoothing (Serial: J<seconds>): time for the acceleration itself
// to build up from 0 to the set acceleration (and to fade back to 0 when
// reaching the target speed). 0 = plain linear ramp (abrupt jerk at the
// start/end of every ramp); larger = softer, but the ramp takes longer.
static const float RAMP_JERK_TIME_DEFAULT_S = 0.4f;
static const float RAMP_JERK_TIME_MAX_S = 2.0f;
volatile float startStopSpeedSps = START_STOP_SPEED_DEFAULT_SPS;  // read by the ramp ISR
volatile float rampJerkTimeS = RAMP_JERK_TIME_DEFAULT_S;          // read by the ramp ISR

// Driver enable timing around a move:
//  - settle: after ENABLE the coils snap the rotor to the nearest full-step
//    position (the "tek"); stepping straight away adds that snap to the
//    first step. Waiting lets the rotor come to rest first.
//  - hold: keep the coils energised for a moment after the last step so
//    the rotor is held still while it settles, instead of being released
//    mid-motion (a small bounce/jerk at every stop).
static const unsigned long DRIVER_SETTLE_MS = 30;
static const unsigned long DRIVER_HOLD_AFTER_STOP_MS = 300;
volatile bool driverDisablePending = false;     // set by the ramp ISR on stop
volatile unsigned long driverStoppedAtMs = 0;

volatile uint16_t nextStepPeriod = 0xFFFF;  // applied by the ISR right after a compare match
// Owned by the ramp ISR while the motor runs; only touched from loop()
// inside ATOMIC_BLOCKs (startRun) or while the ramp timer is off.
volatile float currentSpeedSps = 0.0f;      // actual speed being generated
volatile float currentAccelSps2 = 0.0f;     // signed; S-curve state

// Jog (Serial G<steps>): move an exact number of steps at the start/stop
// speed, no ramp. Mainly a diagnostic - e.g. G200 is one full revolution
// in full-step mode but only 1/16 rev (22.5deg) at 1/16 microstepping.
volatile bool jogActive = false;
volatile bool jogDone = false;
volatile uint32_t jogStepsRemaining = 0;

ISR(TIMER1_COMPA_vect) {
  // Direct port write: digitalWrite() is ~4us.
  PORTD |= _BV(X_STEP_PORTD_BIT);
  __builtin_avr_delay_cycles(32);  // 2us high - A4988 needs >=1us, DRV8825 >=1.9us
  PORTD &= ~_BV(X_STEP_PORTD_BIT);
  // Safe to change OCR1A here: TCNT1 has just been reset to 0, so the new
  // compare value can't already be behind the counter (which would make it
  // wrap through 65535 and cause a ~33ms hiccup).
  OCR1A = nextStepPeriod;
  if (jogActive && --jogStepsRemaining == 0) {
    TIMSK1 &= ~_BV(OCIE1A);
    TCCR1B = 0;
    jogActive = false;
    jogDone = true;
  }
}

void setStepRate(float sps) {
  uint16_t period = (uint16_t)(TIMER1_TICK_HZ / sps) - 1;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { nextStepPeriod = period; }
}

void stepTimerStart(float sps) {
  setStepRate(sps);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = nextStepPeriod;
    TIFR1 = _BV(OCF1A);
    TIMSK1 |= _BV(OCIE1A);
    TCCR1B = _BV(WGM12) | _BV(CS11);  // CTC, clk/8
  }
}

void stepTimerStop() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TIMSK1 &= ~_BV(OCIE1A);
    TCCR1B = 0;
  }
}

// uint8_t-sized so reads/writes are atomic: the ramp ISR changes
// STOPPING -> STOPPED on its own.
enum RunState : uint8_t { STATE_STOPPED, STATE_RUNNING, STATE_STOPPING };
volatile RunState state = STATE_STOPPED;

void rampTimerStart() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR2A = _BV(WGM21);  // CTC
    TCCR2B = 0;
    TCNT2 = 0;
    OCR2A = 249;  // 125kHz / 250 = 500Hz
    TIFR2 = _BV(OCF2A);
    TIMSK2 |= _BV(OCIE2A);
    TCCR2B = _BV(CS22) | _BV(CS20);  // clk/128
  }
}

void rampTimerStop() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TIMSK2 &= ~_BV(OCIE2A);
    TCCR2B = 0;
  }
}

volatile float targetSpeed = DEFAULT_SPEED_SPS;      // read by the ramp ISR
bool directionForward = true; // true = normal winding direction
volatile float accelerationSps2 = DEFAULT_ACCELERATION_SPS2;  // read by the ramp ISR

long lastEncoderRaw = 0;
bool lastButtonReading = HIGH;
bool buttonState = HIGH;
unsigned long lastButtonChangeMs = 0;
static const unsigned long DEBOUNCE_MS = 30;

volatile bool lcdDirty = true; // (also set by the ramp ISR) force first draw of the speed/status/direction fields
bool menuDirty = false; // same idea, for the menu/edit/message screens
unsigned long lastRow0UpdateMs = 0;
static const unsigned long ROW0_MIN_INTERVAL_MS = 50; // rate-limit fast encoder spinning

// Spinner in the top-right corner, animates only while RUNNING - a cheap
// single-character I2C write, negligible impact on stepper timing.
unsigned long lastSpinnerMs = 0;
static const unsigned long SPINNER_INTERVAL_MS = 250;
uint8_t spinnerFrame = 0;
const char SPINNER_FRAMES[4] = {'|', '/', '-', '\\'};

// Bottom row shows the current/target temperature.
unsigned long lastRow1UpdateMs = 0;
// A 16-char row1 rewrite is a much heavier I2C transaction than the 1-char
// spinner - kept modest so loop() stays responsive (steps come from Timer1,
// so this no longer affects motor smoothness).
static const unsigned long ROW1_UPDATE_INTERVAL_MS = 2000;

// Periodically re-init the LCD as a self-heal against I2C glitches (loose
// wiring / motor EMI can occasionally corrupt a transaction and blank the
// display even though the MCU itself keeps running fine). This does not fix
// the underlying electrical cause - check wiring/shielding too.
unsigned long lastLcdReinitMs = 0;
static const unsigned long LCD_REINIT_INTERVAL_MS = 30000;

// ---- Heater / temperature control (E3D-style NTC + MOSFET/SSR) -----------
// Divider: 5V -> THERMISTOR_R_SERIES -> THERMISTOR_PIN -> NTC -> GND.
// R0/T0/BETA below are typical values for a genuine E3D thermistor
// (Semitec 104GT-2 style, 100k @ 25C) - re-derive them if yours differs.
static const float THERMISTOR_R_SERIES = 4700.0f;
static const float THERMISTOR_R0 = 100000.0f;
static const float THERMISTOR_T0_C = 25.0f;
static const float THERMISTOR_BETA = 4267.0f;

static const float MAX_SAFE_TEMP_C = 280.0f; // hard cutoff, above PET's normal range
static const float MAX_SETPOINT_C = 260.0f;
static const float TEMP_STEP_C = 5.0f;       // change per encoder detent

float currentTempC = 0.0f;
float targetTempC = 0.0f; // 0 = heater intentionally off

// Light smoothing of currentTempC on top of the 15-sample median in
// readTemperatureC() (which already rejects stepper EMI spikes). Kept
// deliberately FAST: the old filter (alpha 0.08 + 3C clamp) could only
// follow ~1C/s, so a heater that really rises faster than that was read as
// much colder than it was and the PID kept heating -> 10-20C overshoot.
// alpha 0.35 at 250ms ~ 0.6s time constant; the 10C/cycle clamp only stops
// a single absurd sample, not a real temperature rise.
// Sensor-fault detection still uses the raw reading, so a genuine
// disconnect is still caught promptly.
bool tempFilterInitialized = false;
static const float TEMP_EMA_ALPHA = 0.35f;
static const float MAX_TEMP_STEP_C = 10.0f; // per-cycle clamp for a single bad sample

// PID. Defaults below were tuned on this machine's heater (relay autotune
// + model fit + step tests at 50/100/150/200/240C, see git history):
// overshoot <= ~2C and +-0.6C hold at 240C. Re-tune live over Serial with
// P<kp> I<ki> K<kd> F<kff> while logging with L if the heater, block or
// thermistor mounting changes. Output is 0..255 (= 0..100% heater duty).
// Keep Kd moderate (<= ~100): higher values made this heater oscillate on
// hold (the thermistor sees heat several seconds late).
//   - Derivative acts on the MEASURED temperature (not the error), so it
//     brakes as soon as the temperature rises fast, and a setpoint change
//     doesn't cause a derivative kick.
//   - Integral only accumulates within PID_INTEGRAL_BAND_C of the setpoint
//     and not while the output is saturated (anti-windup). Far below the
//     setpoint P alone already gives full power; letting I build up there
//     is what used to push the temperature past the setpoint.
//   - Feed-forward: Kff * (setpoint - ambient) is added up front. Measured
//     on this heater, holding a temperature needs ~0.8 * (T - 25) of the
//     0..255 output (50C: 21, 100C: 55, 150C: 100), so the PID only has to
//     correct the small remainder. Without it the integral alone would have
//     to supply ~170 at PET temperatures - beyond PID_I_TERM_MAX, and slow
//     to build up from zero. Kff is set a bit below the measured 0.8 so the
//     integral (always positive at hold) trims the rest.
static const float HEATER_KP_DEFAULT = 12.0f;
static const float HEATER_KI_DEFAULT = 0.25f;
static const float HEATER_KD_DEFAULT = 75.0f;
static const float HEATER_KFF_DEFAULT = 0.6f;
static const float HEATER_AMBIENT_C = 25.0f;
static const float PID_INTEGRAL_BAND_C = 10.0f;
static const float PID_I_TERM_MAX = 120.0f;     // max integral contribution (of 255)
static const float PID_D_FILTER_ALPHA = 0.5f;   // smoothing of the dT/dt estimate
float heaterKp = HEATER_KP_DEFAULT;
float heaterKi = HEATER_KI_DEFAULT;
float heaterKd = HEATER_KD_DEFAULT;
float heaterKff = HEATER_KFF_DEFAULT;
float pidIntegral = 0.0f;
float pidLastTempC = 0.0f;
float pidTempRate = 0.0f;   // filtered dT/dt, C/s
// Last terms, for the L(og) output.
float pidLastP = 0.0f, pidLastI = 0.0f, pidLastD = 0.0f, pidLastFF = 0.0f;
uint8_t heaterLastOutput = 0;
bool heaterLogEnabled = false;
unsigned long lastHeaterLogMs = 0;
static const unsigned long HEATER_LOG_INTERVAL_MS = 1000;

unsigned long lastHeaterUpdateMs = 0;
static const unsigned long HEATER_UPDATE_INTERVAL_MS = 250;

// A single noisy/invalid ADC sample cuts the heater immediately (fail-safe)
// but only latches a full FAULT_SENSOR after it persists across several
// consecutive checks - so one stepper-noise glitch doesn't force a
// power-cycle, while a genuinely disconnected/shorted sensor still does.
uint8_t consecutiveSensorFaults = 0;
static const uint8_t SENSOR_FAULT_THRESHOLD = 3;

// Thermal runaway protection: while actively heating toward a setpoint,
// temperature must rise by at least RUNAWAY_MIN_RISE_C within
// RUNAWAY_WINDOW_MS, or something is wrong (heater not actually heating,
// thermistor fell off the block, etc.) and the heater is latched off.
// This - plus the sensor-fault and MAX_SAFE_TEMP_C checks in updateHeater()
// - is a simplified version of what firmware like Marlin calls thermal
// runaway protection; it is not exhaustive, but covers the main failure
// modes. A fault latches until the MCU is reset (power-cycled) - deliberate,
// so a real fault can't be waved away by a stray button press.
enum HeaterFault { FAULT_NONE, FAULT_SENSOR, FAULT_OVERTEMP, FAULT_RUNAWAY };
HeaterFault heaterFault = FAULT_NONE;
float runawayReferenceTempC = 0.0f;
unsigned long runawayReferenceMs = 0;
static const float RUNAWAY_MIN_RISE_C = 1.0f;
static const unsigned long RUNAWAY_WINDOW_MS = 120000;
// The "must keep rising" rule only makes sense while still clearly climbing
// toward a setpoint for the first time. A PID naturally slows down well
// before it actually reaches the setpoint (output shrinks as error
// shrinks), so this margin needs to be generous - RUNAWAY_MIN_RISE_C itself
// is too tight and gets hit by the slow final approach, not a real fault.
// Once within this margin, holding/coasting is the correct behaviour, not
// a fault. Reset whenever the setpoint changes.
static const float RUNAWAY_DISARM_MARGIN_C = 15.0f;
bool reachedTargetOnce = false;

bool readTemperatureC(float &outTempC) {
  // Median of several samples spread over a few milliseconds (not just
  // back-to-back) - the stepper's switching noise can inject a spike that
  // lasts longer than a single microsecond-scale ADC read, so samples taken
  // literally back-to-back can still mostly land inside one noise burst. A
  // small gap between samples means they land at different points relative
  // to the stepper's switching, so a burst is far less likely to corrupt
  // the majority of them. After sorting, the lowest and highest
  // TRIM_EACH_SIDE samples are dropped (whatever minority a burst still
  // corrupted) and the rest are AVERAGED. Averaging instead of taking the
  // median matters at PET temperatures: with a 100k NTC + 4.7k pull-up the
  // ADC only reads ~50 counts at 240C, ~1-2C per count, and the median is
  // stuck on whole counts. The natural +-1 count ADC noise averaged over 9
  // samples gives sub-count resolution, which keeps the PID's D term from
  // reacting to quantisation steps.
  static const uint8_t SAMPLES = 15;
  int samples[SAMPLES];
  for (uint8_t i = 0; i < SAMPLES; i++) {
    samples[i] = analogRead(THERMISTOR_PIN);
    delayMicroseconds(200);
  }
  for (uint8_t i = 1; i < SAMPLES; i++) {
    int key = samples[i];
    int8_t j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }
  static const uint8_t TRIM_EACH_SIDE = 3;
  long sum = 0;
  for (uint8_t i = TRIM_EACH_SIDE; i < SAMPLES - TRIM_EACH_SIDE; i++) sum += samples[i];
  float adc = (float)sum / (float)(SAMPLES - 2 * TRIM_EACH_SIDE);
  if (adc <= 2.0f || adc >= 1021.0f) return false; // shorted or open sensor
  float r = THERMISTOR_R_SERIES * adc / (1023.0f - adc);
  float t0K = THERMISTOR_T0_C + 273.15f;
  float lnR = logf(r / THERMISTOR_R0);
  float tK = 1.0f / (1.0f / t0K + lnR / THERMISTOR_BETA);
  outTempC = tK - 273.15f;
  return true;
}

// Time-proportional output: within every HEATER_WINDOW_MS window the heater
// is ON for (heaterDuty / 255) of the window. The PID updates heaterDuty
// every 250ms; driveHeaterOutput() runs every loop() and does the switching.
static const unsigned long HEATER_WINDOW_MS = 1000;
uint8_t heaterDuty = 0; // 0..255
unsigned long heaterWindowStartMs = 0;

void setHeaterPwm(uint8_t pwm) {
  if (heaterFault != FAULT_NONE) pwm = 0; // fault latch overrides everything
  heaterDuty = pwm;
  if (pwm == 0) digitalWrite(HEATER_PIN, LOW); // cut immediately, don't wait for the window
}

void driveHeaterOutput() {
  unsigned long now = millis();
  if (now - heaterWindowStartMs >= HEATER_WINDOW_MS) heaterWindowStartMs = now;
  unsigned long onMs = ((unsigned long)heaterDuty * HEATER_WINDOW_MS) / 255UL;
  bool on = (heaterFault == FAULT_NONE) && (now - heaterWindowStartMs) < onMs;
  digitalWrite(HEATER_PIN, on ? HIGH : LOW);
}

void setTargetTemp(float tempC) {
  if (tempC < 0.0f) tempC = 0.0f;
  if (tempC > MAX_SETPOINT_C) tempC = MAX_SETPOINT_C;
  targetTempC = tempC;
  reachedTargetOnce = false; // a new setpoint means a fresh climb to track
  runawayReferenceTempC = currentTempC;
  runawayReferenceMs = millis();
  lcdDirty = true;
  menuDirty = true;
}

void updateHeater() {
  unsigned long now = millis();
  if (now - lastHeaterUpdateMs < HEATER_UPDATE_INTERVAL_MS) return;
  float dt = (now - lastHeaterUpdateMs) / 1000.0f;
  lastHeaterUpdateMs = now;

  if (heaterFault != FAULT_NONE) {
    setHeaterPwm(0);
    return;
  }

  float measured;
  if (!readTemperatureC(measured)) {
    setHeaterPwm(0); // always safe to sit idle while the reading can't be trusted
    consecutiveSensorFaults++;
    if (consecutiveSensorFaults >= SENSOR_FAULT_THRESHOLD) {
      heaterFault = FAULT_SENSOR;
      lcdDirty = true;
    }
    return;
  }
  consecutiveSensorFaults = 0;

  if (!tempFilterInitialized) {
    currentTempC = measured;
    pidLastTempC = measured;
    tempFilterInitialized = true;
  } else {
    // Per-cycle clamp: stops one absurd sample that slipped past the median
    // filter from yanking the reading, without slowing down a real rise.
    float delta = measured - currentTempC;
    if (delta > MAX_TEMP_STEP_C) delta = MAX_TEMP_STEP_C;
    if (delta < -MAX_TEMP_STEP_C) delta = -MAX_TEMP_STEP_C;
    currentTempC += TEMP_EMA_ALPHA * delta;
  }

  if (currentTempC >= MAX_SAFE_TEMP_C) {
    heaterFault = FAULT_OVERTEMP;
    lcdDirty = true;
    setHeaterPwm(0);
    return;
  }

  // dT/dt on the measurement, smoothed - updated every cycle (even with the
  // heater off) so it's already valid the moment heating starts.
  if (dt > 0.0f) {
    float rate = (currentTempC - pidLastTempC) / dt;
    pidTempRate += PID_D_FILTER_ALPHA * (rate - pidTempRate);
  }
  pidLastTempC = currentTempC;

  if (targetTempC <= 0.0f) {
    pidIntegral = 0.0f;
    pidLastP = pidLastI = pidLastD = pidLastFF = 0.0f;
    heaterLastOutput = 0;
    runawayReferenceTempC = currentTempC;
    runawayReferenceMs = now;
    setHeaterPwm(0);
    return;
  }

  if (currentTempC >= targetTempC - RUNAWAY_DISARM_MARGIN_C) {
    reachedTargetOnce = true; // close enough to/at setpoint - holding/coasting steady is fine from here
  }

  if (!reachedTargetOnce) {
    if (currentTempC >= runawayReferenceTempC + RUNAWAY_MIN_RISE_C) {
      runawayReferenceTempC = currentTempC;
      runawayReferenceMs = now;
    } else if ((now - runawayReferenceMs) > RUNAWAY_WINDOW_MS) {
      heaterFault = FAULT_RUNAWAY;
      lcdDirty = true;
      setHeaterPwm(0);
      return;
    }
  }

  float error = targetTempC - currentTempC;
  float ffTerm = heaterKff * (targetTempC - HEATER_AMBIENT_C);
  if (ffTerm < 0.0f) ffTerm = 0.0f;
  float pTerm = heaterKp * error;
  float dTerm = -heaterKd * pidTempRate;

  if (error > PID_INTEGRAL_BAND_C) {
    // Far below setpoint: P alone saturates the heater anyway. Start the
    // integral from zero once inside the band instead of pre-loaded.
    pidIntegral = 0.0f;
  } else if (heaterKi > 0.0f) {
    float unsaturated = ffTerm + pTerm + heaterKi * pidIntegral + dTerm;
    bool pushingIntoLimit = (unsaturated >= 255.0f && error > 0.0f) ||
                            (unsaturated <= 0.0f && error < 0.0f);
    if (!pushingIntoLimit) pidIntegral += error * dt;
    float iMax = PID_I_TERM_MAX / heaterKi;
    pidIntegral = constrain(pidIntegral, -iMax, iMax);
  }
  float iTerm = heaterKi * pidIntegral;

  float output = constrain(ffTerm + pTerm + iTerm + dTerm, 0.0f, 255.0f);
  pidLastFF = ffTerm;
  pidLastP = pTerm;
  pidLastI = iTerm;
  pidLastD = dTerm;
  heaterLastOutput = (uint8_t)output;
  setHeaterPwm(heaterLastOutput);
}

// "L" serial command: one short line per second for watching / tuning the
// heater. Kept under the 64-byte TX buffer so Serial.print never blocks.
//   T=<temp> S=<setpoint> O=<0-255 duty> p=<P> i=<I> d=<D> f=<feed-forward>
void logHeater() {
  if (!heaterLogEnabled) return;
  unsigned long now = millis();
  if (now - lastHeaterLogMs < HEATER_LOG_INTERVAL_MS) return;
  lastHeaterLogMs = now;
  Serial.print(F("T="));
  Serial.print(currentTempC, 1);
  Serial.print(F(" S="));
  Serial.print((int)targetTempC);
  Serial.print(F(" O="));
  Serial.print(heaterLastOutput);
  Serial.print(F(" p="));
  Serial.print((int)pidLastP);
  Serial.print(F(" i="));
  Serial.print((int)pidLastI);
  Serial.print(F(" d="));
  Serial.print((int)pidLastD);
  Serial.print(F(" f="));
  Serial.println((int)pidLastFF);
}

// ---- Settings menu (long-press SW to open) --------------------------------
enum UiMode { MODE_MAIN, MODE_MENU, MODE_EDIT_SPEED, MODE_EDIT_DIRECTION, MODE_EDIT_ACCEL, MODE_EDIT_TEMP, MODE_MESSAGE };
UiMode uiMode = MODE_MAIN;

enum MenuItem {
  MENU_SPEED = 0,
  MENU_DIRECTION = 1,
  MENU_ACCEL = 2,
  MENU_TEMP = 3,
  MENU_SAVE = 4,
  MENU_EXIT = 5
};
static const uint8_t MENU_ITEM_COUNT = 6;
uint8_t selectedMenuIndex = MENU_SPEED;

unsigned long lastMenuUpdateMs = 0;
static const unsigned long MENU_MIN_INTERVAL_MS = 50; // rate-limit fast encoder spinning

unsigned long lastMenuActivityMs = 0;
static const unsigned long MENU_IDLE_TIMEOUT_MS = 10000; // auto-exit to main if untouched

// Generic transient message screen (used for warnings & confirmations).
unsigned long messageStartMs = 0;
static const unsigned long MESSAGE_MS = 1500;

static const unsigned long LONG_PRESS_MS = 700;
unsigned long pressStartMs = 0;
bool longPressFired = false;

// ---- Persisted settings (EEPROM, wear-levelled) ---------------------------
// "Save Settings" writes to a NEW slot each time (round-robin, tagged with a
// rising sequence number) instead of always rewriting the same bytes. Uno's
// internal EEPROM is rated ~100k write cycles per cell; spreading writes
// across EEPROM_SLOT_COUNT slots multiplies that endurance by the same
// factor. On boot, all slots are scanned and the one with the highest
// sequence number (the most recent save) wins.
struct PersistedSettings {
  uint16_t magic;
  float speed;
  uint8_t direction; // 1 = forward, 0 = reverse
  float acceleration;
  float targetTemp;
  float startStopSpeed;  // V
  float jerkTime;        // J
};
// Bumped whenever the struct changes so an old save is never misread
// (0xF17B: + targetTemp, safety-relevant; 0xF17C: + V/J motion settings).
// A bump means previously saved settings are ignored once - re-save them.
static const uint16_t SETTINGS_MAGIC = 0xF17C;

struct EepromSlot {
  uint32_t sequence;
  PersistedSettings settings;
};
static const uint8_t EEPROM_SLOT_COUNT = 32;
static const int EEPROM_BASE_ADDR = 0;

uint8_t lastSavedSlot = EEPROM_SLOT_COUNT - 1; // so the first save lands on slot 0
uint32_t lastSavedSequence = 0;

int eepromSlotAddress(uint8_t index) {
  return EEPROM_BASE_ADDR + (int)index * (int)sizeof(EepromSlot);
}

void loadSettingsFromEeprom() {
  int bestIndex = -1;
  uint32_t bestSeq = 0;
  for (uint8_t i = 0; i < EEPROM_SLOT_COUNT; i++) {
    EepromSlot slot;
    EEPROM.get(eepromSlotAddress(i), slot);
    if (slot.settings.magic == SETTINGS_MAGIC && (bestIndex < 0 || slot.sequence > bestSeq)) {
      bestIndex = i;
      bestSeq = slot.sequence;
    }
  }
  if (bestIndex < 0) return; // nothing saved yet, keep compiled-in defaults

  EepromSlot best;
  EEPROM.get(eepromSlotAddress(bestIndex), best);
  lastSavedSlot = (uint8_t)bestIndex;
  lastSavedSequence = best.sequence;

  const PersistedSettings &s = best.settings;
  if (s.speed >= MIN_SPEED_SPS && s.speed <= MAX_SPEED_SPS) targetSpeed = s.speed;
  if (s.acceleration >= MIN_ACCEL_SPS2 && s.acceleration <= MAX_ACCEL_SPS2) accelerationSps2 = s.acceleration;
  if (s.startStopSpeed >= START_STOP_SPEED_MIN_SPS && s.startStopSpeed <= START_STOP_SPEED_MAX_SPS)
    startStopSpeedSps = s.startStopSpeed;
  if (s.jerkTime >= 0.0f && s.jerkTime <= RAMP_JERK_TIME_MAX_S) rampJerkTimeS = s.jerkTime;
  // Deliberately NOT restoring s.targetTemp - the heater must never resume
  // heating on its own after a reset/power-cycle. It's still written by
  // saveSettingsToEeprom() (kept for struct/slot compatibility), just never
  // applied here; the operator always has to re-set the temperature by hand
  // each session.
  directionForward = (s.direction != 0);
}

void saveSettingsToEeprom() {
  uint8_t nextSlot = (lastSavedSlot + 1) % EEPROM_SLOT_COUNT;
  EepromSlot slot;
  slot.sequence = lastSavedSequence + 1;
  slot.settings.magic = SETTINGS_MAGIC;
  slot.settings.speed = targetSpeed;
  slot.settings.direction = directionForward ? 1 : 0;
  slot.settings.acceleration = accelerationSps2;
  slot.settings.targetTemp = targetTempC;
  slot.settings.startStopSpeed = startStopSpeedSps;
  slot.settings.jerkTime = rampJerkTimeS;

  EEPROM.put(eepromSlotAddress(nextSlot), slot);
  lastSavedSlot = nextSlot;
  lastSavedSequence = slot.sequence;
}

// Forward declarations - implemented in the LCD section further down.
void padTo16(char *buf);
void startJog(long steps);
void updateLcdRow0Fields();
void updateLcdSpinner();
void updateLcdRow1();

// ---------------------------------------------------------------------------
void enableDriver(bool enable) {
  // CNC shield stepper drivers are enabled with the ENABLE line LOW.
  digitalWrite(STEPPERS_ENABLE_PIN, enable ? LOW : HIGH);
}

void startRun() {
  if (state == STATE_RUNNING) return;
  if (jogActive) {
    Serial.println(F("Jog in progress"));
    return;
  }
  if (state == STATE_STOPPED) {
    // Direction can only change while stopped, so it's only set here.
    digitalWrite(X_DIR_PIN, directionForward ? HIGH : LOW);
    if (driverDisablePending) {
      // Restarted within the post-stop hold: coils are still energised and
      // the rotor is already held, so no settle wait is needed.
      driverDisablePending = false;
      delayMicroseconds(5);  // DIR setup time before the first step
    } else {
      enableDriver(true);
      delay(DRIVER_SETTLE_MS);  // let the rotor snap/settle before stepping
    }
    float startSpeed = startStopSpeedSps;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      currentSpeedSps = startSpeed;  // ramp up from here
      currentAccelSps2 = 0.0f;       // S-curve starts with zero accel
      state = STATE_RUNNING;
    }
    stepTimerStart(startSpeed);
    rampTimerStart();
  } else {
    // From STATE_STOPPING: just ramp back up without stopping first.
    state = STATE_RUNNING;
  }
  lcdDirty = true;
}

void requestStop() {
  if (state == STATE_STOPPED) return;
  state = STATE_STOPPING;  // the ramp ISR slows down, then stops
  lcdDirty = true;
}

// ---- Start/stop cycle test (Serial: C) -------------------------------------
// Repeats run -> stop -> idle -> run ... with the current speed/accel/V/J,
// so driver current (Vref) or motion settings can be adjusted while feeling
// the effect on every start and stop.
//   C          toggle on/off (defaults / last used times)
//   C4         run 4s per cycle (idle unchanged)
//   C4,2       run 4s, idle 2s
//   C0 or X    stop the cycle (X also stops the motor)
// The encoder button also ends it. Run time counts from the start command,
// so it includes the acceleration ramp.
enum CyclePhase : uint8_t { CYCLE_RUN, CYCLE_STOPPING, CYCLE_IDLE };
bool cycleEnabled = false;
CyclePhase cyclePhase = CYCLE_IDLE;
unsigned long cyclePhaseStartMs = 0;
unsigned long cycleRunMs = 4000;
unsigned long cycleIdleMs = 2000;
uint32_t cycleCount = 0;

void stopCycle() {
  if (!cycleEnabled) return;
  cycleEnabled = false;
  Serial.print(F("Cycle off after "));
  Serial.print(cycleCount);
  Serial.println(F(" starts"));
}

void startCycle() {
  cycleEnabled = true;
  cycleCount = 0;
  cyclePhase = CYCLE_IDLE;
  cyclePhaseStartMs = millis() - cycleIdleMs;  // first start right away
  if (state != STATE_STOPPED) {
    requestStop();
    cyclePhase = CYCLE_STOPPING;
  }
  Serial.print(F("Cycle on: run "));
  Serial.print(cycleRunMs / 1000.0f, 1);
  Serial.print(F("s idle "));
  Serial.print(cycleIdleMs / 1000.0f, 1);
  Serial.println(F("s"));
}

void handleCycleCommand(const String &arg) {
  if (arg.length() == 0) {
    if (cycleEnabled) {
      stopCycle();
      requestStop();
    } else {
      startCycle();
    }
    return;
  }
  float runS = arg.toFloat();
  if (runS <= 0.0f) {
    stopCycle();
    requestStop();
    return;
  }
  cycleRunMs = (unsigned long)(constrain(runS, 0.5f, 600.0f) * 1000.0f);
  int comma = arg.indexOf(',');
  if (comma >= 0) {
    float idleS = arg.substring(comma + 1).toFloat();
    cycleIdleMs = (unsigned long)(constrain(idleS, 0.3f, 600.0f) * 1000.0f);
  }
  startCycle();
}

void updateCycle() {
  if (!cycleEnabled) return;
  unsigned long now = millis();
  switch (cyclePhase) {
    case CYCLE_RUN:
      if (now - cyclePhaseStartMs >= cycleRunMs) {
        requestStop();
        cyclePhase = CYCLE_STOPPING;
      }
      break;
    case CYCLE_STOPPING:
      if (state == STATE_STOPPED) {
        cyclePhase = CYCLE_IDLE;
        cyclePhaseStartMs = now;
      }
      break;
    case CYCLE_IDLE:
      if (now - cyclePhaseStartMs >= cycleIdleMs) {
        startRun();
        if (state == STATE_RUNNING) {
          cycleCount++;
          Serial.print(F("Cycle "));
          Serial.println(cycleCount);
          cyclePhase = CYCLE_RUN;
          cyclePhaseStartMs = now;
        }
      }
      break;
  }
}

void toggleRunStop() {
  if (cycleEnabled) {  // button ends a running cycle test
    stopCycle();
    requestStop();
    return;
  }
  if (state == STATE_STOPPED) {
    startRun();
  } else {
    requestStop();
  }
}

void toggleDirection() {
  if (state != STATE_STOPPED) {
    Serial.println(F("Stop the motor before changing direction."));
    return;
  }
  directionForward = !directionForward;
  lcdDirty = true;
  menuDirty = true;
  Serial.print(F("Direction: "));
  Serial.println(directionForward ? F("FORWARD") : F("REVERSE"));
}

void setSpeed(float sps) {
  if (sps < MIN_SPEED_SPS) sps = MIN_SPEED_SPS;
  if (sps > MAX_SPEED_SPS) sps = MAX_SPEED_SPS;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { targetSpeed = sps; }  // read by the ramp ISR
  lcdDirty = true;
  menuDirty = true;
}

void setAccel(float value) {
  if (value < MIN_ACCEL_SPS2) value = MIN_ACCEL_SPS2;
  if (value > MAX_ACCEL_SPS2) value = MAX_ACCEL_SPS2;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { accelerationSps2 = value; }  // read by the ramp ISR
  menuDirty = true;
}

// ---- Settings menu ---------------------------------------------------
void copyFlashLabel(char *dst, size_t dstSize, const __FlashStringHelper *src) {
  strncpy_P(dst, reinterpret_cast<PGM_P>(src), dstSize - 1);
  dst[dstSize - 1] = '\0';
}

const __FlashStringHelper *menuItemLabel(uint8_t index) {
  switch (index) {
    case MENU_SPEED: return F("Speed");
    case MENU_DIRECTION: return F("Direction");
    case MENU_ACCEL: return F("Acceleration");
    case MENU_TEMP: return F("Temperature");
    case MENU_SAVE: return F("Save Settings");
    default: return F("Exit");
  }
}

void updateMenuScreen() {
  char label0[16];
  copyFlashLabel(label0, sizeof(label0), menuItemLabel(selectedMenuIndex));
  char row0[17];
  snprintf(row0, sizeof(row0), ">%s", label0);
  padTo16(row0);
  lcd.setCursor(0, 0);
  lcd.print(row0);

  // No wraparound: only preview the next item if one actually follows.
  char row1[17];
  if (selectedMenuIndex + 1 < MENU_ITEM_COUNT) {
    char label1[16];
    copyFlashLabel(label1, sizeof(label1), menuItemLabel(selectedMenuIndex + 1));
    snprintf(row1, sizeof(row1), " %s", label1);
  } else {
    row1[0] = '\0';
  }
  padTo16(row1);
  lcd.setCursor(0, 1);
  lcd.print(row1);
}

void updateEditSpeedScreen() {
  char row1[17];
  lcd.setCursor(0, 0);
  lcd.print(F("Set Speed:      "));
  snprintf(row1, sizeof(row1), ">%d sps", (int)targetSpeed);
  padTo16(row1);
  lcd.setCursor(0, 1);
  lcd.print(row1);
}

void updateEditDirectionScreen() {
  char row1[17];
  lcd.setCursor(0, 0);
  lcd.print(F("Set Direction:  "));
  snprintf(row1, sizeof(row1), ">%s", directionForward ? "FORWARD" : "REVERSE");
  padTo16(row1);
  lcd.setCursor(0, 1);
  lcd.print(row1);
}

void updateEditAccelScreen() {
  char row1[17];
  lcd.setCursor(0, 0);
  lcd.print(F("Acceleration:   "));
  snprintf(row1, sizeof(row1), ">%d sps^2", (int)accelerationSps2);
  padTo16(row1);
  lcd.setCursor(0, 1);
  lcd.print(row1);
}

void updateEditTempScreen() {
  char row1[17];
  lcd.setCursor(0, 0);
  lcd.print(F("Set Temp (0=off)"));
  if (targetTempC <= 0.0f) {
    snprintf(row1, sizeof(row1), ">OFF");
  } else {
    snprintf(row1, sizeof(row1), ">%dC", (int)targetTempC);
  }
  padTo16(row1);
  lcd.setCursor(0, 1);
  lcd.print(row1);
}

void showTransientMessage(const __FlashStringHelper *line0, const __FlashStringHelper *line1) {
  uiMode = MODE_MESSAGE;
  messageStartMs = millis();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1);
}

void enterMenu() {
  uiMode = MODE_MENU;
  lastMenuActivityMs = millis();
  lcd.clear();
  updateMenuScreen();
}

void exitToMain() {
  uiMode = MODE_MAIN;
  lcd.clear();
  updateLcdRow0Fields();
  updateLcdSpinner();
  updateLcdRow1();
  lcdDirty = false;
}

void selectMenuItem() {
  lastMenuActivityMs = millis();
  switch (selectedMenuIndex) {
    case MENU_SPEED:
      uiMode = MODE_EDIT_SPEED;
      lcd.clear();
      updateEditSpeedScreen();
      break;
    case MENU_DIRECTION:
      if (state != STATE_STOPPED) {
        showTransientMessage(F("Stop motor      "), F("first!          "));
      } else {
        uiMode = MODE_EDIT_DIRECTION;
        lcd.clear();
        updateEditDirectionScreen();
      }
      break;
    case MENU_ACCEL:
      uiMode = MODE_EDIT_ACCEL;
      lcd.clear();
      updateEditAccelScreen();
      break;
    case MENU_TEMP:
      uiMode = MODE_EDIT_TEMP;
      lcd.clear();
      updateEditTempScreen();
      break;
    case MENU_SAVE:
      saveSettingsToEeprom();
      showTransientMessage(F("Settings        "), F("saved!          "));
      break;
    case MENU_EXIT:
    default:
      exitToMain();
      break;
  }
}

void onButtonShortPress() {
#if DEBUG_SERIAL
  Serial.println(F("SW short"));
#endif
  switch (uiMode) {
    case MODE_MAIN:
      toggleRunStop();
      break;
    case MODE_MENU:
      selectMenuItem();
      break;
    case MODE_EDIT_SPEED:
    case MODE_EDIT_DIRECTION:
    case MODE_EDIT_ACCEL:
    case MODE_EDIT_TEMP:
    case MODE_MESSAGE:
      enterMenu(); // confirm (value already applied live) & go back up to the list
      break;
  }
}

void onButtonLongPress() {
#if DEBUG_SERIAL
  Serial.println(F("SW long"));
#endif
  if (uiMode == MODE_MAIN) {
    enterMenu();
  } else {
    exitToMain();
  }
}

void updateUiTimeouts() {
  unsigned long now = millis();
  if (uiMode == MODE_MESSAGE && (now - messageStartMs) > MESSAGE_MS) {
    enterMenu();
  } else if (uiMode != MODE_MAIN && uiMode != MODE_MESSAGE &&
             (now - lastMenuActivityMs) > MENU_IDLE_TIMEOUT_MS) {
    exitToMain();
  }
}

// ---- Encoder handling -------------------------------------------------
// Quadrature decoder on pin-change interrupt PCINT1 (A0 = PC0 = CLK,
// A1 = PC1 = DT). Same transition table / count direction as the
// PJRC Encoder library: 4 counts per KY-040 detent. Invalid double-bit
// jumps (noise / missed edge) are ignored instead of guessed.
volatile long encoderCount = 0;
volatile uint8_t encoderPrevAB = 0;
static const int8_t ENCODER_TABLE[16] = {
  // index = old | (new << 2); columns = old AB 00,01,10,11 (bit0 = CLK)
  0, 1, -1, 0,      // new 00
  -1, 0, 0, 1,      // new 01
  1, 0, 0, -1,      // new 10
  0, -1, 1, 0       // new 11
};

ISR(PCINT1_vect) {
  uint8_t ab = PINC & 0x03;
  encoderCount += ENCODER_TABLE[encoderPrevAB | (ab << 2)];
  encoderPrevAB = ab;
}

void setupEncoder() {
  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
  encoderPrevAB = PINC & 0x03;
  PCMSK1 |= _BV(PCINT8) | _BV(PCINT9); // A0, A1
  PCIFR = _BV(PCIF1);                  // drop any stale flag
  PCICR |= _BV(PCIE1);
}

long readEncoder() {
  noInterrupts();
  long v = encoderCount;
  interrupts();
  return v;
}

void handleEncoder() {
  long raw = readEncoder();
  long delta = raw - lastEncoderRaw;
  // Most KY-040 style encoders report 4 counts per detent.
  long detents = delta / 4;
  if (detents == 0) return;
  lastEncoderRaw += detents * 4;
#if DEBUG_SERIAL
  Serial.print(F("ENC "));
  Serial.print(detents > 0 ? '+' : '-');
  Serial.print(F(" cnt="));
  Serial.println(raw);
#endif

  switch (uiMode) {
    case MODE_MAIN:
      // Speed can only be changed from the settings menu (long-press SW).
      break;
    case MODE_MENU: {
      int16_t newIndex = (int16_t)selectedMenuIndex + detents;
      if (newIndex < 0) newIndex = 0;
      if (newIndex > MENU_ITEM_COUNT - 1) newIndex = MENU_ITEM_COUNT - 1;
      selectedMenuIndex = (uint8_t)newIndex;
      menuDirty = true;
      lastMenuActivityMs = millis();
      break;
    }
    case MODE_EDIT_SPEED:
      setSpeed(targetSpeed + detents * SPEED_STEP_SPS);
      lastMenuActivityMs = millis();
      break;
    case MODE_EDIT_DIRECTION:
      directionForward = (detents > 0);
      menuDirty = true;
      lastMenuActivityMs = millis();
      break;
    case MODE_EDIT_ACCEL:
      setAccel(accelerationSps2 + detents * ACCEL_STEP_SPS2);
      lastMenuActivityMs = millis();
      break;
    case MODE_EDIT_TEMP:
      setTargetTemp(targetTempC + detents * TEMP_STEP_C);
      lastMenuActivityMs = millis();
      break;
    default:
      break;
  }
}

void handleButton() {
  bool reading = digitalRead(ENCODER_SW_PIN);
  if (reading != lastButtonReading) {
    lastButtonChangeMs = millis();
  }
  if ((millis() - lastButtonChangeMs) > DEBOUNCE_MS && reading != buttonState) {
    buttonState = reading;
    if (buttonState == LOW) { // pressed (active LOW with INPUT_PULLUP)
      pressStartMs = millis();
      longPressFired = false;
    } else if (!longPressFired) {
      onButtonShortPress();
    }
  }
  if (buttonState == LOW && !longPressFired && (millis() - pressStartMs) >= LONG_PRESS_MS) {
    longPressFired = true;
    onButtonLongPress();
  }
  lastButtonReading = reading;
}

// ---- Serial command handling -------------------------------------------
// Commands, e.g. "S500", "R", "X", "D", "?". A newline terminator is
// preferred, but if none arrives within SERIAL_IDLE_TIMEOUT_MS of the last
// character, the buffered line is processed anyway - some serial monitor
// tools don't append a newline unless explicitly configured to.
static const unsigned long SERIAL_IDLE_TIMEOUT_MS = 100;

void processCommandLine(const String &line) {
  if (line.length() == 0) return;
  char cmd = line.charAt(0);
  switch (cmd) {
    case 'S':
    case 's': {
      float value = line.substring(1).toFloat();
      setSpeed(value);
      Serial.print(F("Speed set to "));
      Serial.println(targetSpeed);
      break;
    }
    case 'R':
    case 'r':
      startRun();
      Serial.println(F("Running"));
      break;
    case 'X':
    case 'x':
      stopCycle();
      requestStop();
      Serial.println(F("Stopping"));
      break;
    case 'D':
    case 'd':
      toggleDirection();
      break;
    case 'C':
    case 'c':
      handleCycleCommand(line.substring(1));
      break;
    case 'G':
    case 'g':
      startJog(line.substring(1).toInt());
      break;
    case 'V':
    case 'v': {
      float value = constrain(line.substring(1).toFloat(), START_STOP_SPEED_MIN_SPS, START_STOP_SPEED_MAX_SPS);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { startStopSpeedSps = value; }
      Serial.print(F("Start/stop speed "));
      Serial.println(value);
      break;
    }
    case 'J':
    case 'j': {
      float value = constrain(line.substring(1).toFloat(), 0.0f, RAMP_JERK_TIME_MAX_S);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { rampJerkTimeS = value; }
      Serial.print(F("S-curve jerk time "));
      Serial.println(value);
      break;
    }
    case 'A':
    case 'a': {
      float value = line.substring(1).toFloat();
      setAccel(value);
      Serial.print(F("Acceleration set to "));
      Serial.println(accelerationSps2);
      break;
    }
    case 'P':
    case 'p':
      heaterKp = line.substring(1).toFloat();
      Serial.print(F("Kp=")); Serial.println(heaterKp);
      break;
    case 'I':
    case 'i':
      heaterKi = line.substring(1).toFloat();
      pidIntegral = 0.0f; // integral scale changed - restart it
      Serial.print(F("Ki=")); Serial.println(heaterKi);
      break;
    case 'K':
    case 'k':
      heaterKd = line.substring(1).toFloat();
      Serial.print(F("Kd=")); Serial.println(heaterKd);
      break;
    case 'F':
    case 'f':
      heaterKff = line.substring(1).toFloat();
      Serial.print(F("Kff=")); Serial.println(heaterKff);
      break;
    case 'L':
    case 'l':
      heaterLogEnabled = !heaterLogEnabled;
      Serial.println(heaterLogEnabled ? F("Log ON") : F("Log OFF"));
      break;
    case 'T':
    case 't': {
      float value = line.substring(1).toFloat();
      setTargetTemp(value);
      Serial.print(F("Target temp set to "));
      Serial.println(targetTempC);
      break;
    }
    case 'B':
    case 'b': {
      // Backlight toggle - for checking the LCD backlight circuit (measure
      // the LCD's K pin to GND: ~0V when ON, floating/high when OFF).
      static bool backlightOn = true;
      backlightOn = !backlightOn;
      if (backlightOn) lcd.backlight(); else lcd.noBacklight();
      Serial.print(F("Backlight "));
      Serial.print(backlightOn ? F("ON") : F("OFF"));
      Serial.println(lcd.present ? F("") : F(" (LCD not detected!)"));
      break;
    }
    case '?':
      Serial.print(F("state="));
      Serial.print(state == STATE_RUNNING ? F("RUNNING") : state == STATE_STOPPING ? F("STOPPING") : F("STOPPED"));
      Serial.print(F(" speed="));
      Serial.print(targetSpeed);
      Serial.print(F(" dir="));
      Serial.print(directionForward ? F("FWD") : F("REV"));
      Serial.print(F(" accel="));
      Serial.print(accelerationSps2);
      Serial.print(F(" V="));
      Serial.print(startStopSpeedSps);
      Serial.print(F(" J="));
      Serial.print(rampJerkTimeS);
      Serial.print(F(" temp="));
      Serial.print(currentTempC);
      Serial.print(F("/"));
      Serial.print(targetTempC);
      Serial.print(F(" Kp="));
      Serial.print(heaterKp);
      Serial.print(F(" Ki="));
      Serial.print(heaterKi);
      Serial.print(F(" Kd="));
      Serial.print(heaterKd);
      Serial.print(F(" Kff="));
      Serial.print(heaterKff);
      Serial.print(F(" fault="));
      switch (heaterFault) {
        case FAULT_SENSOR: Serial.println(F("SENSOR")); break;
        case FAULT_OVERTEMP: Serial.println(F("OVERTEMP")); break;
        case FAULT_RUNAWAY: Serial.println(F("RUNAWAY")); break;
        default: Serial.println(F("NONE")); break;
      }
      break;
    default:
      Serial.println(F("Unknown command"));
      break;
  }
}

void handleSerial() {
  static String line;
  static unsigned long lastCharMs = 0;

  while (Serial.available()) {
    char c = Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r') {
      processCommandLine(line);
      line = "";
    } else {
      line += c;
    }
  }

  if (line.length() > 0 && (millis() - lastCharMs) > SERIAL_IDLE_TIMEOUT_MS) {
    processCommandLine(line);
    line = "";
  }
}

// ---- Stepper speed ramp (runs in the Timer2 ISR) -------------------------
// S-curve (jerk-limited) speed ramp toward targetSpeed (RUNNING) or down to
// startStopSpeedSps and then a stop (STOPPING).
//
// A plain linear ramp switches the acceleration fully on/off instantly,
// which a full-stepping motor feels as a jolt at the start and end of
// every speed change. Here the acceleration itself changes at a limited
// rate (jerk), so speed eases in and eases out:
//   - aMaxNow = sqrt(2 * jerk * |speed error|) is the largest acceleration
//     that can still be faded back to 0 exactly as the target is reached,
//     so the ramp starts easing out early instead of overshooting.
// Called every RAMP_TICK_S exactly, independent of loop().
static void rampTick() {
  if (state == STATE_STOPPED) return;
  const float dt = RAMP_TICK_S;

  float speed = currentSpeedSps;
  float accel = currentAccelSps2;
  float accelMax = accelerationSps2;
  float floorSpeed = startStopSpeedSps;
  float target = (state == STATE_RUNNING) ? targetSpeed : floorSpeed;
  float err = target - speed;
  float jerkTime = rampJerkTimeS;
  float jerk = (jerkTime > 0.0f) ? accelMax / jerkTime : 1e9f;

  float aMaxNow = sqrtf(2.0f * jerk * fabsf(err));
  if (aMaxNow > accelMax) aMaxNow = accelMax;
  float aWanted = (err >= 0.0f) ? aMaxNow : -aMaxNow;

  float da = aWanted - accel;
  float daMax = jerk * dt;
  if (da > daMax) da = daMax;
  if (da < -daMax) da = -daMax;
  accel += da;

  float newSpeed = speed + accel * dt;
  // Snap onto the target instead of overshooting past it.
  if ((err >= 0.0f && newSpeed >= target) || (err < 0.0f && newSpeed <= target)) {
    newSpeed = target;
    accel = 0.0f;
  }
  // Never ramp below the start/stop speed while running (a target speed
  // below it is fine: the floor only applies to the stop ramp).
  if (state == STATE_STOPPING && newSpeed < floorSpeed) newSpeed = floorSpeed;
  if (newSpeed < START_STOP_SPEED_MIN_SPS) newSpeed = START_STOP_SPEED_MIN_SPS;

  if (state == STATE_STOPPING && newSpeed <= floorSpeed) {
    stepTimerStop();
    currentSpeedSps = 0.0f;
    currentAccelSps2 = 0.0f;
    state = STATE_STOPPED;
    // Coils stay energised for DRIVER_HOLD_AFTER_STOP_MS; loop() disables
    // the driver afterwards (releaseDriverIfDue()).
    driverStoppedAtMs = millis();
    driverDisablePending = true;
    lcdDirty = true;
    rampTimerStop();
    return;
  }
  currentSpeedSps = newSpeed;
  currentAccelSps2 = accel;
  setStepRate(newSpeed);
}

// G<steps>: exact-step move (negative = opposite direction). Only while
// stopped. Uses the start/stop speed (V) so it starts without a ramp.
void startJog(long steps) {
  if (state != STATE_STOPPED || jogActive) {
    Serial.println(F("Stop the motor first"));
    return;
  }
  if (steps == 0) return;
  bool forward = directionForward;
  if (steps < 0) {
    forward = !forward;
    steps = -steps;
  }
  digitalWrite(X_DIR_PIN, forward ? HIGH : LOW);
  if (driverDisablePending) {
    driverDisablePending = false;  // coils still held - no settle needed
    delayMicroseconds(5);
  } else {
    enableDriver(true);
    delay(DRIVER_SETTLE_MS);
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    jogStepsRemaining = (uint32_t)steps;
    jogDone = false;
    jogActive = true;
  }
  stepTimerStart(startStopSpeedSps);
  Serial.print(F("Jog "));
  Serial.print(steps);
  Serial.println(F(" steps"));
}

void finishJogIfDone() {
  if (!jogDone) return;
  jogDone = false;
  driverStoppedAtMs = millis();
  driverDisablePending = true;  // released after the normal post-stop hold
  Serial.println(F("Jog done"));
}

// Turns the driver off DRIVER_HOLD_AFTER_STOP_MS after the motor stopped.
void releaseDriverIfDue() {
  if (!driverDisablePending || state != STATE_STOPPED || jogActive) return;
  unsigned long stoppedAt;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { stoppedAt = driverStoppedAtMs; }
  if (millis() - stoppedAt < DRIVER_HOLD_AFTER_STOP_MS) return;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (driverDisablePending && state == STATE_STOPPED) {
      enableDriver(false);
      driverDisablePending = false;
    }
  }
}

// ISR_NOBLOCK re-enables interrupts on entry, so the (more time-critical)
// Timer1 step ISR can preempt this float math - step timing stays exact.
// The tick takes well under 2ms, so it never re-enters itself.
ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  rampTick();
}

// ---- LCD -----------------------------------------------------------------
// Layout (16x2):
//   Row0: "S:<speed> <RUN/STP/IDL> <dir>      <spinner>"
//   Row1: "Temp:<current>/<target>C" (or FAULT message if heaterFault is set)
// Each region below is only rewritten when it actually changes (or on its
// own slow timer), never on every loop - a forced full-screen redraw would
// block long enough on I2C to make the stepper stutter.

void padTo16(char *buf) {
  size_t len = strlen(buf);
  for (size_t i = len; i < 16; i++) buf[i] = ' ';
  buf[16] = '\0';
}

void updateLcdRow0Fields() {
  lcd.setCursor(0, 0);
  if (heaterFault != FAULT_NONE) {
    char line[17];
    switch (heaterFault) {
      case FAULT_SENSOR: snprintf(line, sizeof(line), "FAULT:SENSOR"); break;
      case FAULT_OVERTEMP: snprintf(line, sizeof(line), "FAULT:OVERTEMP"); break;
      case FAULT_RUNAWAY: snprintf(line, sizeof(line), "FAULT:RUNAWAY"); break;
      default: line[0] = '\0'; break;
    }
    padTo16(line);
    lcd.print(line);
    return;
  }
  lcd.print(F("S:"));
  char speedBuf[5];
  snprintf(speedBuf, sizeof(speedBuf), "%-4d", (int)targetSpeed);
  lcd.print(speedBuf);
  lcd.print(' ');
  switch (state) {
    case STATE_RUNNING: lcd.print(F("RUN")); break;
    case STATE_STOPPING: lcd.print(F("STP")); break;
    default: lcd.print(F("IDL")); break;
  }
  lcd.print(' ');
  lcd.print(directionForward ? '>' : '<');
  lcd.print(F("  "));
}

void updateLcdSpinner() {
  lcd.setCursor(15, 0);
  lcd.print(state == STATE_RUNNING ? SPINNER_FRAMES[spinnerFrame] : ' ');
}

void formatTempLine(char *out, size_t outSize) {
  int cur = (int)(currentTempC + 0.5f);
  if (targetTempC <= 0.0f) {
    snprintf(out, outSize, "Temp:%dC (off)", cur);
  } else {
    int tgt = (int)(targetTempC + 0.5f);
    snprintf(out, outSize, "Temp:%d/%dC", cur, tgt);
  }
}

void updateLcdRow1() {
  char line[18];
  if (heaterFault != FAULT_NONE) {
    snprintf(line, sizeof(line), "PWR CYCLE TO FIX");
  } else {
    formatTempLine(line, sizeof(line));
  }
  padTo16(line);
  lcd.setCursor(0, 1);
  lcd.print(line);
}

void updateMainScreen() {
  unsigned long now = millis();

  if (lcdDirty && (now - lastRow0UpdateMs) >= ROW0_MIN_INTERVAL_MS) {
    lastRow0UpdateMs = now;
    updateLcdRow0Fields();
    lcdDirty = false;
  }

  if (now - lastSpinnerMs >= SPINNER_INTERVAL_MS) {
    lastSpinnerMs = now;
    if (state == STATE_RUNNING) spinnerFrame = (spinnerFrame + 1) % 4;
    updateLcdSpinner();
  }

  if (now - lastRow1UpdateMs >= ROW1_UPDATE_INTERVAL_MS) {
    lastRow1UpdateMs = now;
    updateLcdRow1();
  }
}

void updateLcd() {
  if (uiMode == MODE_MAIN) {
    updateMainScreen();
    return;
  }

  if (!menuDirty) return;
  unsigned long now = millis();
  if ((now - lastMenuUpdateMs) < MENU_MIN_INTERVAL_MS) return;
  lastMenuUpdateMs = now;
  menuDirty = false;

  switch (uiMode) {
    case MODE_MENU: updateMenuScreen(); break;
    case MODE_EDIT_SPEED: updateEditSpeedScreen(); break;
    case MODE_EDIT_DIRECTION: updateEditDirectionScreen(); break;
    case MODE_EDIT_ACCEL: updateEditAccelScreen(); break;
    case MODE_EDIT_TEMP: updateEditTempScreen(); break;
    default: break;
  }
}

void printCentered(uint8_t row, const __FlashStringHelper *text) {
  size_t len = strlen_P(reinterpret_cast<PGM_P>(text));
  uint8_t col = (len >= 16) ? 0 : (16 - len) / 2;
  lcd.setCursor(col, row);
  lcd.print(text);
}

void showSplashScreen() {
  lcd.clear();
  printCentered(0, F("PET Filament"));
  printCentered(1, F("Maker v1"));
  delay(1500);
  lcd.clear();
}

void reinitLcdIfDue() {
  // Never do this while the motor is actively running - the HD44780 init
  // sequence has mandatory delays baked in that block loop() (encoder
  // response, speed ramp) for a noticeable moment. Defer until stopped;
  // don't advance the timer meanwhile so it fires as soon as it safely can.
  if (state == STATE_RUNNING) return;

  unsigned long now = millis();
  if (now - lastLcdReinitMs < LCD_REINIT_INTERVAL_MS) return;
  lastLcdReinitMs = now;

  bool wasPresent = lcd.present;
  lcd.present = probeLcd();
  if (lcd.present != wasPresent) {
    Serial.println(lcd.present ? F("LCD detected") : F("LCD lost"));
  }
  if (!lcd.present) return;

  lcd.init();
  lcd.backlight();

  // Only force the small row0/menu fields to redraw immediately. The
  // spinner and row1 are left to pick back up on their own existing
  // schedule (next tick, up to a few seconds away) instead of piling every
  // element into this same instant - that would turn one already-heavy
  // lcd.init() into a much bigger I2C burst and block loop() longer.
  lcdDirty = true;
  menuDirty = true;
  lastRow0UpdateMs = 0;
  lastMenuUpdateMs = 0;
}

// ---------------------------------------------------------------------------
// One-shot I2C scanner at boot - prints every responding address so the LCD
// address (LCD_I2C_ADDRESS, usually 0x27 or 0x3F) can be confirmed.
void scanI2cBus() {
  Serial.println(F("I2C scan (SDA=A4, SCL=A5)..."));
  // Idle I2C lines must sit HIGH (pulled up by the LCD backpack). A line
  // stuck LOW = shorted to GND, SDA/SCL swapped onto something else, or the
  // module unpowered - and would hang an un-timed-out Wire transfer.
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delay(2);
  bool sdaHigh = digitalRead(SDA);
  bool sclHigh = digitalRead(SCL);
  Serial.print(F("  idle SDA="));
  Serial.print(sdaHigh ? F("HIGH") : F("LOW"));
  Serial.print(F(" SCL="));
  Serial.println(sclHigh ? F("HIGH") : F("LOW"));
  if (!sdaHigh || !sclHigh) {
    Serial.println(F("  bus stuck LOW - check LCD wiring/5V/GND"));
  }

  Wire.begin();
  // Never let a broken bus hang the firmware (setup() runs before the
  // watchdog is armed): abort any transfer after 25ms and reset TWI.
  Wire.setWireTimeout(25000, true);
  uint8_t found = 0;
  uint8_t timeouts = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 5) {
      timeouts++;
      if (timeouts >= 3) {
        Serial.println(F("  timeouts - bus not responding, scan aborted"));
        break;
      }
      continue;
    }
    if (err == 0) {
      Serial.print(F("  found 0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == LCD_I2C_ADDRESS) Serial.print(F(" (LCD)"));
      Serial.println();
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("  no I2C devices - check SDA/SCL/5V/GND wiring"));
  } else {
    if (!probeLcd()) {
      Serial.print(F("  WARNING: nothing at LCD_I2C_ADDRESS 0x"));
      Serial.println(LCD_I2C_ADDRESS, HEX);
    }
  }
}

// ---------------------------------------------------------------------------
// Nano clones with the OLD bootloader don't clear the watchdog after a
// watchdog reset: WDRF stays set, which keeps the WDT forced on, so the
// board can end up in an endless reset loop. Clear MCUSR and disable the
// WDT in .init3 - i.e. before the Arduino core's init() and setup() even
// run - which is the avr-libc recommended fix. Harmless on Optiboot.
uint8_t resetCause __attribute__((section(".noinit")));
void earlyWatchdogDisable() __attribute__((naked, used, section(".init3")));
void earlyWatchdogDisable() {
  resetCause = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

void setup() {
  wdt_disable(); // belt-and-braces; the real clear already happened in earlyWatchdogDisable()

  // Force the heater output to a known OFF state as the very first thing,
  // before anything else runs - a floating pin during the rest of setup()
  // should never be able to turn the heater on.
  digitalWrite(HEATER_PIN, LOW);
  pinMode(HEATER_PIN, OUTPUT);

  Serial.begin(9600);

  loadSettingsFromEeprom();

  pinMode(STEPPERS_ENABLE_PIN, OUTPUT);
  enableDriver(false);

  setupEncoder();
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  lastButtonReading = digitalRead(ENCODER_SW_PIN);
  buttonState = lastButtonReading;

  pinMode(X_STEP_PIN, OUTPUT);
  digitalWrite(X_STEP_PIN, LOW);
  pinMode(X_DIR_PIN, OUTPUT);

  scanI2cBus();
  lcd.present = probeLcd();
  if (!lcd.present) Serial.println(F("LCD not found - running without display"));

  lcd.init();
  lcd.backlight();
  showSplashScreen();

  Serial.println(F("PET Bottle Filament Maker ready."));
  Serial.println(F("Commands: S A V J G<steps> C<run,idle> R X D T<tempC> L P I K F B ?"));

  // Enable last, after every blocking setup step (including the splash
  // delay) has already completed - a true loop() hang now gets the MCU
  // reset within 2s instead of staying stuck until someone power-cycles it.
  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset();

  handleSerial();
  handleEncoder();
  handleButton();
  finishJogIfDone();
  releaseDriverIfDue();
  updateCycle();
  updateUiTimeouts();
  reinitLcdIfDue();
  updateHeater();
  driveHeaterOutput();
  logHeater();
  updateLcd();
}
