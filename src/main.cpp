#include <Arduino.h>
#include <AccelStepper.h>
#include <Encoder.h>
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
// Hardware: Arduino UNO + CNC Shield V3, only the X-axis driver socket is
// populated. That one stepper both pulls the softened PET strip and winds
// the finished filament onto the spool. Heater cartridge + E3D-style NTC
// thermistor are driven directly by this Arduino (PID + thermal runaway
// protection - see the "Heater / temperature control" section below).
//
// CNC shield pins used (X driver socket):
//   X.STEP   -> D2
//   X.DIR    -> D5
//   ENABLE   -> D8   (shared driver enable, active LOW)
// Spare CNC shield headers reused for the operator panel (Y/Z sockets are
// left unpopulated so these lines are free):
//   Encoder CLK -> D3  (Y.STEP header, keeps hardware interrupt support)
//   Encoder DT  -> D6  (Y.DIR header)
//   Encoder SW  -> D4  (Z.STEP header)
// LCD I2C uses the UNO's fixed I2C bus: A4 = SDA, A5 = SCL.
// Heater:
//   Thermistor  -> A0  (voltage divider: A1 -> 4.7k resistor -> A0 -> NTC -> GND)
//   5V for the divider is sourced from pin A1 itself, driven HIGH as a fixed
//   output - the divider only draws a fraction of a mA, well within a GPIO's
//   output capability, so this is just for wiring convenience (A1 is right
//   next to A0) rather than routing a wire back to the shield's 5V rail.
//   Heater PWM  -> D9  (to MOSFET/SSR module signal input; module GND shared
//                        with Arduino GND, heater itself on its own 12V supply)
// ---------------------------------------------------------------------------

// ---- Pin definitions --------------------------------------------------
static const uint8_t X_STEP_PIN = 2;
static const uint8_t X_DIR_PIN = 5;
static const uint8_t STEPPERS_ENABLE_PIN = 8;

static const uint8_t ENCODER_CLK_PIN = 3;
static const uint8_t ENCODER_DT_PIN = 6;
static const uint8_t ENCODER_SW_PIN = 4;

static const uint8_t THERMISTOR_PIN = A0;
static const uint8_t THERMISTOR_EXCITE_PIN = A1; // driven HIGH as the divider's "5V"
static const uint8_t HEATER_PWM_PIN = 9;

// ---- LCD -----------------------------------------------------------------
// Address is commonly 0x27 or 0x3F depending on the PCF8574 module; run an
// I2C scanner sketch once to confirm if the display stays blank.
static const uint8_t LCD_I2C_ADDRESS = 0x27;
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);

// ---- Motion tuning (adjust to match motor steps/rev, microstepping and
// spool mechanics) ----------------------------------------------------------
static const float MIN_SPEED_SPS = 50.0f;    // steps/sec
static const float MAX_SPEED_SPS = 3000.0f;  // steps/sec - verify the motor actually reaches this without skipping steps
static const float SPEED_STEP_SPS = 10.0f;   // change per encoder detent
static const float DEFAULT_SPEED_SPS = 300.0f;

static const float MIN_ACCEL_SPS2 = 50.0f;    // steps/sec^2
static const float MAX_ACCEL_SPS2 = 2000.0f;  // steps/sec^2
static const float ACCEL_STEP_SPS2 = 50.0f;   // change per encoder detent
static const float DEFAULT_ACCELERATION_SPS2 = 300.0f;

static const long BIG_TARGET = 2000000000L;   // "run forever" target
static const long REARM_THRESHOLD = 10000L;   // re-centre position before overflow

AccelStepper stepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
Encoder encoder(ENCODER_CLK_PIN, ENCODER_DT_PIN);

enum RunState { STATE_STOPPED, STATE_RUNNING, STATE_STOPPING };
RunState state = STATE_STOPPED;

float targetSpeed = DEFAULT_SPEED_SPS;
bool directionForward = true; // true = normal winding direction
float accelerationSps2 = DEFAULT_ACCELERATION_SPS2;

long lastEncoderRaw = 0;
bool lastButtonReading = HIGH;
bool buttonState = HIGH;
unsigned long lastButtonChangeMs = 0;
static const unsigned long DEBOUNCE_MS = 30;

bool lcdDirty = true; // force first draw of the speed/status/direction fields
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
// spinner - kept modest so it doesn't stall stepper.run().
static const unsigned long ROW1_UPDATE_INTERVAL_MS = 2000;

// Periodically re-init the LCD as a self-heal against I2C glitches (loose
// wiring / motor EMI can occasionally corrupt a transaction and blank the
// display even though the MCU itself keeps running fine). This does not fix
// the underlying electrical cause - check wiring/shielding too.
unsigned long lastLcdReinitMs = 0;
static const unsigned long LCD_REINIT_INTERVAL_MS = 30000;

// ---- Heater / temperature control (E3D-style NTC + MOSFET/SSR) -----------
// Divider: THERMISTOR_EXCITE_PIN (driven HIGH) -> THERMISTOR_R_SERIES ->
// THERMISTOR_PIN -> NTC -> GND.
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

// Smooths currentTempC across several updateHeater() cycles (~1s time
// constant) - rejects short EMI bursts from the stepper that a single
// in-place sample burst can't, since a real noise event rarely corrupts
// every 250ms-spaced reading in a row the way it can corrupt four samples
// taken back-to-back in <1ms. Sensor-fault detection still uses the raw
// (unsmoothed) reading in readTemperatureC(), so a genuine disconnect is
// still caught promptly.
bool tempFilterInitialized = false;
static const float TEMP_EMA_ALPHA = 0.08f;
static const float MAX_TEMP_STEP_C = 3.0f; // slew-rate clamp per update cycle, see updateHeater()

// PID - placeholders. This thermal system (heater mass, insulation, ambient
// airflow) is not modelled; tune Kp/Ki/Kd against your actual hardware
// (e.g. watch step response to a setpoint change and adjust) before relying
// on this for unattended operation.
static const float HEATER_KP = 22.0f;
static const float HEATER_KI = 1.2f;
static const float HEATER_KD = 90.0f;
float pidIntegral = 0.0f;
float pidLastError = 0.0f;

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
  // the majority of them. A median then ignores whatever minority is still
  // corrupted outright; a plain average would still get pulled toward it.
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
  int adc = (samples[SAMPLES / 2 - 1] + samples[SAMPLES / 2]) / 2;
  if (adc <= 2 || adc >= 1021) return false; // shorted or open sensor
  float r = THERMISTOR_R_SERIES * (float)adc / (1023.0f - (float)adc);
  float t0K = THERMISTOR_T0_C + 273.15f;
  float lnR = logf(r / THERMISTOR_R0);
  float tK = 1.0f / (1.0f / t0K + lnR / THERMISTOR_BETA);
  outTempC = tK - 273.15f;
  return true;
}

void setHeaterPwm(uint8_t pwm) {
  if (heaterFault != FAULT_NONE) pwm = 0; // fault latch overrides everything
  analogWrite(HEATER_PWM_PIN, pwm);
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
    tempFilterInitialized = true;
  } else {
    // Slew-rate limit: a real heater's thermal mass can't swing this much
    // in one 250ms cycle, so clamp how far a single reading can pull the
    // filtered value even if it slipped past the median filter above -
    // caps any one bad sample's effect on currentTempC to a few tenths of
    // a degree instead of letting it show up as a sudden jump.
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

  if (targetTempC <= 0.0f) {
    pidIntegral = 0.0f;
    pidLastError = 0.0f;
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
  pidIntegral = constrain(pidIntegral + error * dt, -100.0f, 100.0f); // anti-windup clamp
  float derivative = (dt > 0.0f) ? (error - pidLastError) / dt : 0.0f;
  pidLastError = error;

  float output = HEATER_KP * error + HEATER_KI * pidIntegral + HEATER_KD * derivative;
  output = constrain(output, 0.0f, 255.0f);
  setHeaterPwm((uint8_t)output);
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
};
// Bumped so old saves (pre-heater, smaller struct) are never misread as a
// valid targetTemp - safety-relevant since that field drives the heater.
static const uint16_t SETTINGS_MAGIC = 0xF17B;

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

  EEPROM.put(eepromSlotAddress(nextSlot), slot);
  lastSavedSlot = nextSlot;
  lastSavedSequence = slot.sequence;
}

// Forward declarations - implemented in the LCD section further down.
void padTo16(char *buf);
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
  enableDriver(true);
  long dirSign = directionForward ? 1L : -1L;
  stepper.setCurrentPosition(0);
  stepper.setAcceleration(accelerationSps2);
  stepper.setMaxSpeed(targetSpeed);
  stepper.moveTo(dirSign * BIG_TARGET);
  state = STATE_RUNNING;
  lcdDirty = true;
}

void requestStop() {
  if (state == STATE_STOPPED) return;
  stepper.stop(); // recalculates a moveTo() that decelerates smoothly
  state = STATE_STOPPING;
  lcdDirty = true;
}

void toggleRunStop() {
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
  targetSpeed = sps;
  lcdDirty = true;
  menuDirty = true;
}

void setAccel(float value) {
  if (value < MIN_ACCEL_SPS2) value = MIN_ACCEL_SPS2;
  if (value > MAX_ACCEL_SPS2) value = MAX_ACCEL_SPS2;
  accelerationSps2 = value;
  stepper.setAcceleration(accelerationSps2);
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
void handleEncoder() {
  long raw = encoder.read();
  long delta = raw - lastEncoderRaw;
  // Most KY-040 style encoders report 4 counts per detent.
  long detents = delta / 4;
  if (detents == 0) return;
  lastEncoderRaw += detents * 4;

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
      requestStop();
      Serial.println(F("Stopping"));
      break;
    case 'D':
    case 'd':
      toggleDirection();
      break;
    case 'T':
    case 't': {
      float value = line.substring(1).toFloat();
      setTargetTemp(value);
      Serial.print(F("Target temp set to "));
      Serial.println(targetTempC);
      break;
    }
    case '?':
      Serial.print(F("state="));
      Serial.print(state == STATE_RUNNING ? F("RUNNING") : state == STATE_STOPPING ? F("STOPPING") : F("STOPPED"));
      Serial.print(F(" speed="));
      Serial.print(targetSpeed);
      Serial.print(F(" dir="));
      Serial.print(directionForward ? F("FWD") : F("REV"));
      Serial.print(F(" temp="));
      Serial.print(currentTempC);
      Serial.print(F("/"));
      Serial.print(targetTempC);
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

// ---- Stepper state machine ----------------------------------------------
void updateStepperState() {
  if (state == STATE_RUNNING) {
    stepper.setMaxSpeed(targetSpeed);
    if (labs(stepper.distanceToGo()) < REARM_THRESHOLD) {
      long dirSign = directionForward ? 1L : -1L;
      stepper.setCurrentPosition(0);
      stepper.moveTo(dirSign * BIG_TARGET);
    }
  } else if (state == STATE_STOPPING) {
    if (stepper.distanceToGo() == 0) {
      state = STATE_STOPPED;
      enableDriver(false);
      lcdDirty = true;
    }
  }
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
  // sequence has mandatory delays baked in and stalls stepper.run() for long
  // enough to be felt as a jeda. Defer until the machine is idle/stopped;
  // don't advance the timer meanwhile so it fires as soon as it safely can.
  if (state == STATE_RUNNING) return;

  unsigned long now = millis();
  if (now - lastLcdReinitMs < LCD_REINIT_INTERVAL_MS) return;
  lastLcdReinitMs = now;

  lcd.init();
  lcd.backlight();

  // Only force the small row0/menu fields to redraw immediately. The
  // spinner and row1 are left to pick back up on their own existing
  // schedule (next tick, up to a few seconds away) instead of piling every
  // element into this same instant - that would turn one already-heavy
  // lcd.init() into a much bigger I2C burst and stall stepper.run().
  lcdDirty = true;
  menuDirty = true;
  lastRow0UpdateMs = 0;
  lastMenuUpdateMs = 0;
}

// ---------------------------------------------------------------------------
void setup() {
  wdt_disable(); // clear any watchdog state left over from a prior reset before it can loop-reset us

  // Force the heater output to a known OFF state as the very first thing,
  // before anything else runs - a floating pin during the rest of setup()
  // should never be able to turn the heater on.
  pinMode(HEATER_PWM_PIN, OUTPUT);
  analogWrite(HEATER_PWM_PIN, 0);

  pinMode(THERMISTOR_EXCITE_PIN, OUTPUT);
  digitalWrite(THERMISTOR_EXCITE_PIN, HIGH);

  Serial.begin(9600);

  loadSettingsFromEeprom();

  pinMode(STEPPERS_ENABLE_PIN, OUTPUT);
  enableDriver(false);

  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  lastButtonReading = digitalRead(ENCODER_SW_PIN);
  buttonState = lastButtonReading;

  stepper.setMaxSpeed(MAX_SPEED_SPS);
  stepper.setAcceleration(accelerationSps2);

  lcd.init();
  lcd.backlight();
  showSplashScreen();

  Serial.println(F("PET Bottle Filament Maker ready."));
  Serial.println(F("Commands: S<speed> R X D T<tempC> ?"));

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
  updateStepperState();
  updateUiTimeouts();
  reinitLcdIfDue();
  updateHeater();
  stepper.run();
  updateLcd();
}
