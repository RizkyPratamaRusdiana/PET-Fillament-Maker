# Changelog

Versions follow [Semantic Versioning](https://semver.org/). Each release is
tagged `v<version>` in git, and the same number is `FIRMWARE_VERSION` in
[src/main.cpp](src/main.cpp).

## 1.1.0

Motion
- Speed ramp profile changed to a short linear ramp, closer to the original
  AccelStepper firmware: J default 0 (linear), acceleration default
  1000 steps/s² (was 300), start/stop speed default 100 steps/s (was 32).
- New Serial command `N`: toggle the ramp on/off. Off = instant start at the
  target speed and instant stop. Default ON, not saved in EEPROM.

Diagnostics
- New Serial command `M`: worst STEP pulse latency since the last `M`, to
  check whether anything delays the step interrupt.
- Boot message prints the reset cause (POWER-ON / EXTERNAL / BROWNOUT /
  WATCHDOG) from MCUSR. Optiboot (UNO, `nano_new`) clears it, so it reads
  `unknown` there.
- Firmware version and build target shown at boot, on the LCD splash and by `?`.

Build
- `uno` env no longer hard-codes `COM43`; the upload port is auto-detected
  like the Nano envs.

Docs
- New wiring guide [docs/PIN_MAPPING.md](docs/PIN_MAPPING.md): every device pin
  to CNC shield marker to Arduino pin, V3 vs V4 differences, A4988 socket
  and microstep jumpers, and how to verify a marker.
- README: board/env selection, the `CNC_SHIELD_V4` define, upload steps,
  A4988 Vref for the 17HS4401.

## Earlier (unversioned)

- `e608be2` Port to Nano + CNC Shield V4, interrupt-driven motion, retuned heater.
- `43b52a5` Initial UNO + CNC Shield V3 firmware (AccelStepper).
