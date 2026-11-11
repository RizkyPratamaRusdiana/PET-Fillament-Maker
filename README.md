# PET Bottle Filament Maker

Firmware for a PET bottle filament maker built on an ATmega328P (Arduino Nano or
UNO) and a CNC shield. A single stepper pulls the softened PET strip through the
hot end and winds the finished filament onto the spool. The same board runs the
heater: an E3D-style NTC thermistor, PID control, and thermal runaway
protection. The operator panel is a KY-040 rotary encoder and a 16x2 I2C LCD.

**Firmware version: 1.1.0** (see [CHANGELOG.md](CHANGELOG.md)). The version is
shown on the LCD splash screen, in the boot message and by the Serial `?`
command.

## Hardware

| Part | Notes |
|------|-------|
| Arduino Nano (ATmega328P) + CNC Shield V4 | Main target (`env:nano` / `env:nano_new`) |
| Arduino UNO (ATmega328P) + CNC Shield V3 | Supported, but has no A6 pin, so no heater control (see below) |
| Stepper driver (A4988, tested) | X socket only. Y/Z/A sockets stay empty |
| NEMA 17 stepper (17HS4401: 1.8°, 1.7 A/phase) | Puller and spool winder |
| KY-040 rotary encoder with push button | Operator input |
| 16x2 LCD with PCF8574 I2C backpack | Address `0x27` (some modules use `0x3F`) |
| Heater cartridge (12 V) | Hot end heater |
| 100k NTC thermistor (E3D / Semitec 104GT-2 style) | Hot end temperature |
| 4.7 kΩ resistor | Thermistor pull-up (voltage divider) |
| SSR or logic-level MOSFET module | Switches the heater |

## Pin configuration

> **Wiring guide with every CNC shield marker (silkscreen label):
> [docs/PIN_MAPPING.md](docs/PIN_MAPPING.md)** (Bahasa Indonesia).

| Device pin | CNC shield marker | Arduino pin |
|------------|-------------------|-------------|
| A4988 driver | Socket **X** | STEP **D2** (V3) / **D5** (V4), DIR **D5** (V3) / **D2** (V4), EN **D8** |
| 17HS4401 motor | X motor header (1A 1B = coil A, 2A 2B = coil B) | — |
| KY-040 `CLK` | **Abort** | A0 |
| KY-040 `DT` | **Hold** | A1 |
| KY-040 `SW` | **Resume** | A2 |
| SSR / MOSFET `IN+` | **CoolEn** | A3 (Nano only) |
| LCD `SDA` / `SCL` | check your shield (see the wiring guide) | A4 / A5 |
| Thermistor divider | no marker, wire straight to the Nano pin | A6 (Nano only) |

All pins are defined at the top of [src/main.cpp](src/main.cpp). The UNO and
the Nano both use the ATmega328P, so the pin numbers are the same. The only
difference is the X STEP/DIR routing on the CNC shield.

### Stepper (X driver socket)

| Signal | CNC Shield V3 (UNO) | CNC Shield V4 (Nano) | Notes |
|--------|---------------------|----------------------|-------|
| X STEP | D2 | **D5** | Driven straight from the Timer1 ISR (direct `PORTD` write, 2 µs pulse) |
| X DIR  | D5 | **D2** | Set only while the motor is stopped |
| ENABLE | D8 | D8 | Shared by all driver sockets. **Active LOW** |

The CNC Shield V4 used here routes X STEP and X DIR swapped compared with the
GRBL pinout of the V3. This was confirmed on the hardware with
[src/step_finder.cpp](src/step_finder.cpp): pulsing D5 turns the motor and
pulsing D2 does not. The swap is selected at build time with
`-DCNC_SHIELD_V4`, which the `nano` / `nano_new` envs set in `build_flags`.

Microstepping is set by the MS1/MS2/MS3 jumpers under the X driver. Speeds in
the firmware are in (micro)steps per second. If you change the jumpers, or
move between a V3 and a V4 shield with different jumpers, re-check the speeds.

#### Driver current (A4988 + 17HS4401)

The 17HS4401 is rated 1.7 A per phase (typical datasheet values: 1.5 Ω,
2.8 mH, 40 N·cm; check the label on your motor). An A4988 without active
cooling runs hot above ~1.2-1.4 A, so set the current limit to about
**1.2 A** first. Go up to 1.4 A only if the motor skips steps under load,
and add a heatsink.

`Vref = I_limit × 8 × Rs`. Rs is the small sense resistor next to the
A4988 chip:

| Resistor marking | Rs | Vref at 1.2 A | Vref at 1.4 A |
|------------------|----|---------------|---------------|
| R100 (most clones) | 0.100 Ω | 0.96 V | 1.12 V |
| R068 | 0.068 Ω | 0.65 V | 0.76 V |
| R050 | 0.050 Ω | 0.48 V | 0.56 V |

Measure Vref between the trimpot and GND with the motor supply on.

With all three MS jumpers fitted (A4988 1/16 step), one revolution of the
17HS4401 is 200 × 16 = 3200 steps, so RPM = steps/s × 60 / 3200
(1000 steps/s ≈ 18.8 RPM, the 3000 steps/s maximum ≈ 56 RPM).

### Rotary encoder (KY-040)

The encoder plugs into the CNC shield's control headers, which are wired to
A0-A2:

| KY-040 pin | Arduino pin | CNC shield header | Notes |
|------------|-------------|-------------------|-------|
| CLK | A0 (PC0) | Abort  | Quadrature input, pin-change interrupt PCINT8 |
| DT  | A1 (PC1) | Hold   | Quadrature input, pin-change interrupt PCINT9 |
| SW  | A2 (PC2) | Resume | Push button, active LOW, internal pull-up, debounced in software |
| +   | 5V  | — | |
| GND | GND | — | |

CLK and DT **must** stay on A0/A1. The decoder in the `PCINT1_vect` ISR reads
them directly from bits 0 and 1 of `PINC`. A0/A1 have no external interrupt,
which is why a pin-change interrupt is used. Polling alone would miss detents
while `loop()` is busy with LCD I2C writes or thermistor reads.

### LCD (16x2, I2C)

| LCD backpack pin | Arduino pin |
|------------------|-------------|
| SDA | A4 (fixed hardware I2C) |
| SCL | A5 (fixed hardware I2C) |
| VCC | 5V |
| GND | GND |

The I2C address is `LCD_I2C_ADDRESS` (`0x27`). At boot the firmware checks that
SDA/SCL idle HIGH, scans the I2C bus, and prints every address it finds on
Serial. If the LCD is missing or miswired, the firmware keeps running without
it. Every LCD call is skipped, and the display is probed again later, so a
reconnected LCD comes back on its own.

### Heater and thermistor

| Signal | Arduino pin | CNC shield header | Notes |
|--------|-------------|-------------------|-------|
| Thermistor | **A6** | — | Analog-only pin. **Nano only**: the UNO has no A6 |
| Heater output | **A3** | CoolEn | To the SSR/MOSFET module signal input |

Thermistor voltage divider:

```
5V ──[ 4.7kΩ ]──┬── A6
                │
              [NTC 100k]
                │
GND ────────────┘
```

Heater output:

```
A3  ──────────────► SSR / MOSFET module  IN (+)
GND ──────────────► SSR / MOSFET module  IN (-) / GND
12V supply ─► module load side ─► heater cartridge ─► 12V supply GND
```

- The module's logic GND must be shared with the Arduino GND. The heater
  itself runs from its own 12 V supply.
- A3 has no hardware PWM. The PID output is applied as slow time-proportional
  on/off switching over a 1 s window, which suits an SSR much better than
  490 Hz PWM.
- A3 is driven LOW as the very first step in `setup()`, so the heater can never
  switch on while the rest of the board is starting up.

### Pin summary

| Pin | Use | Pin | Use |
|-----|-----|-----|-----|
| D0 | USB Serial RX | A0 | Encoder CLK (Abort) |
| D1 | USB Serial TX | A1 | Encoder DT (Hold) |
| D2 | X STEP (V3) / X DIR (V4) | A2 | Encoder SW (Resume) |
| D3 | free | A3 | Heater SSR/MOSFET (CoolEn) |
| D4 | free | A4 | LCD SDA |
| D5 | X DIR (V3) / X STEP (V4) | A5 | LCD SCL |
| D6, D7 | free | A6 | Thermistor (Nano only) |
| D8 | Driver ENABLE (active LOW) | A7 | free (analog input only) |
| D9-D13 | free | | |

Timers in use: Timer1 (step pulses) and Timer2 (500 Hz speed ramp). PWM on
D9/D10 (Timer1) and D3/D11 (Timer2) is therefore unavailable.

## Building and uploading

Requires [PlatformIO](https://platformio.org/) (VS Code extension or CLI).

### 1. Pick the env for your board

The board is chosen by the PlatformIO **env**, never by editing the code.

| Your hardware | Env | What the env sets |
|---------------|-----|-------------------|
| Arduino Nano clone (CH340, old bootloader) + CNC Shield V4 | **`nano`** (default) | `board = nanoatmega328`, `-DCNC_SHIELD_V4` |
| Arduino Nano with new Optiboot bootloader + CNC Shield V4 | `nano_new` | `board = nanoatmega328new`, `-DCNC_SHIELD_V4` |
| Arduino UNO + CNC Shield V3 | `uno` | `board = uno`, no shield flag |

Not sure which Nano bootloader you have? Try `nano` first. If the upload fails
with `stk500_getsync(): not in sync`, use `nano_new`.

### 2. How the board define works

`-DCNC_SHIELD_V4` is set in `build_flags` of the Nano envs in
[platformio.ini](platformio.ini). In [src/main.cpp](src/main.cpp) it selects
the X STEP/DIR pins:

```cpp
#if defined(CNC_SHIELD_V4)      // env nano / nano_new
  X_STEP_PIN = 5;  X_DIR_PIN = 2;   // V4: STEP/DIR swapped
#else                           // env uno
  X_STEP_PIN = 2;  X_DIR_PIN = 5;   // V3: GRBL pinout
#endif
```

This is the **only** pin difference between the builds. The boot message and
`?` show which one is running, for example
`PET Bottle Filament Maker v1.1.0 (UNO + CNC Shield V3) ready.`

To run a Nano on a V3 shield, or an UNO on a V4 shield, the flag has to match
the **shield**, not the Arduino: add or remove `-DCNC_SHIELD_V4` in that env.

### 3. Upload

Close every serial monitor first. An open monitor locks the COM port, and the
upload then fails with `could not open port ... Access is denied`.

```
pio device list                          # find the Arduino's COM port
pio run -e nano -t upload                # Nano + V4 (port auto-detected)
pio run -e uno  -t upload                # UNO + V3  (port auto-detected)
pio run -e uno  -t upload --upload-port COM14   # force a specific port
pio device monitor -b 9600               # serial monitor afterwards
```

In VS Code: open the PlatformIO sidebar, expand **Project Tasks → `<env>`**
and click **Upload**, or pick the env in the status bar and press the
**→ (Upload)** button.

To change the default env used by a plain `pio run`, edit `default_envs` in
the `[platformio]` section of [platformio.ini](platformio.ini).

### Hardware test sketches

Small standalone sketches for bringing up one part of the hardware at a time.
Each one has an env per board (`_uno`, `_nano`, `_nano_new`) where it applies.

| Env | Sketch | Purpose |
|-----|--------|---------|
| `thermistor_test_nano` | [src/thermistor_test.cpp](src/thermistor_test.cpp) | Prints the raw ADC value of A6 to check the thermistor divider |
| `ssr_test_nano` | [src/ssr_test.cpp](src/ssr_test.cpp) | Switches the heater output on A3 with `1` / `0` over Serial |
| `pin_probe_nano` | [src/pin_probe.cpp](src/pin_probe.cpp) | Prints every pin that changes level, to find which pin a shield header is wired to |
| `step_finder_nano` | [src/step_finder.cpp](src/step_finder.cpp) | Pulses one chosen pin for 3 s to find the real X STEP line |

```
pio run -e step_finder_nano -t upload
pio device monitor -b 9600
```

`src/tft_text.cpp` / `src/tft_text.h` / `src/font5x7.h` are a small ST7789
1.47" SPI display driver with a 16x2 LiquidCrystal-style text API. They are
kept for a future display upgrade and are not built into the current firmware.

## Operation

### LCD

```
S:1000 RUN >   |      <- target speed (steps/s), RUN / STP / IDL, direction (> / <), spinner
Temp:239/240C         <- current / target temperature ("Temp:25C (off)" when the heater is off)
```

On a heater fault the top row shows `FAULT:SENSOR`, `FAULT:OVERTEMP` or
`FAULT:RUNAWAY` and the bottom row shows `PWR CYCLE TO FIX`.

### Encoder

| Action | Main screen | Menu / edit screen |
|--------|-------------|--------------------|
| Short press | Start / stop the motor | Select item / confirm value |
| Long press (0.7 s) | Open the settings menu | Back to the main screen |
| Turn | — | Move through items / change the value |

Menu items: **Speed**, **Direction** (only while stopped), **Acceleration**,
**Temperature** (0 = heater off), **Save Settings**, **Exit**. The menu closes
by itself after 10 s without input.

**Save Settings** stores speed, direction, acceleration, and the start/stop
speed and S-curve time in EEPROM. Writes are wear-levelled over 32 slots. The
target temperature is deliberately **not** restored at power-up: the heater
never starts heating by itself after a reset.

### Serial commands (9600 baud)

| Command | Function |
|---------|----------|
| `S<sps>` | Target speed, steps/s (50-3000) |
| `A<sps2>` | Acceleration, steps/s² (50-2000, default 1000) |
| `V<sps>` | Start/stop speed (31-400, default 100) |
| `J<s>` | S-curve jerk time, seconds (0-2). Default 0 = linear ramp, same start/stop as the old AccelStepper firmware |
| `N` | Toggle the speed ramp (not saved in EEPROM). Default **ON** with A/V/J; OFF = instant start at the target speed and instant stop |
| `R` / `X` | Run / stop (with ramp) |
| `D` | Toggle direction (motor must be stopped) |
| `G<steps>` | Jog an exact number of steps at the start/stop speed |
| `C` / `C<run>` / `C<run>,<idle>` / `C0` | Start/stop cycle test, times in seconds |
| `T<°C>` | Heater setpoint (0 = off, max 260) |
| `P` / `I` / `K` / `F` `<value>` | PID Kp / Ki / Kd / feed-forward Kff |
| `L` | Toggle a heater log line every second (`T S O p i d f`) |
| `M` | Print the worst STEP pulse delay (µs) since the last `M`, then reset it. Large values mean steps are uneven |
| `B` | Toggle the LCD backlight |
| `?` | Print the full status |

## Heater safety

- **Sensor fault**: any invalid reading (open or shorted thermistor) cuts the
  heater immediately. Three in a row latch `FAULT:SENSOR`.
- **Over-temperature**: 280 °C latches `FAULT:OVERTEMP`.
- **Thermal runaway**: while heating towards a new setpoint, the temperature
  must rise at least 1 °C every 120 s until it is within 15 °C of the
  setpoint. Otherwise `FAULT:RUNAWAY` is latched.
- A latched fault keeps the heater off until the board is reset or
  power-cycled.
- A 2 s watchdog resets the MCU if `loop()` hangs. The watchdog is cleared in
  `.init3`, so Nano clones with the old bootloader do not get stuck in a
  watchdog reset loop.

These checks cover the main failure modes, but they are not a substitute for a
thermal fuse and for keeping the machine attended while it is hot.
