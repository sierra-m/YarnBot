# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

YarnBot is a single-file Arduino sketch (`YarnBot.ino`) for an **ESP32-C6**. It reads two RC PWM receiver channels (throttle and steering), mixes them tank-style, and drives two DC motors through two IBT-4 H-bridge boards. It also cuts motor power for good when a 3S LiPo drops to about 9.9 V.

## Build

Run from this folder (the folder name already matches the sketch, so no subfolder is needed):

```
arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc --warnings all .
arduino-cli upload -p <port> --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc .
```

`CDCOnBoot=cdc` routes `Serial` to the C6's native USB port; without it, `Serial` goes to UART0 (GPIO16/17) and nothing appears on the USB serial monitor. On native USB the port is usually `/dev/ttyACM0`.

There are no tests or linter. Verification means compiling cleanly with `--warnings all`, then bench-testing on hardware via the Serial monitor at 115200 baud, which prints a debug line at 5 Hz.

Toolchain notes:
- Requires Arduino-ESP32 core **3.x** (3.3.12 is installed); 2.x does not support the C6. The code uses the 3.x LEDC API (`ledcAttach(pin, freq, res)` and `ledcWrite(pin, duty)`, both keyed by pin, not channel). Don't use the 2.x `ledcSetup`/`ledcAttachPin`.
- Use the official 64-bit arduino-cli at `~/.local/bin/arduino-cli`, with its data in `~/.arduino15`. The snap package ships a 32-bit binary that can't install the 3.x core's tools.

## Architecture

Each pass of `loop()` runs this pipeline:

1. **RC input:** `rcBegin` sets up the MCPWM capture peripheral: one capture timer (`RC_CAP_RES_HZ`, 10 MHz) with one channel per RC input on both edges. The hardware latches the timer at each edge, and the `rcOnCapture` callback computes the width from those timestamps, so interrupt latency doesn't matter. Don't go back to `micros()` in a GPIO ISR: on the C6 it jittered by up to ±130 µs. A falling edge only counts after a rising edge, and widths outside 800–2200 µs are discarded. `loop()` copies the volatile values under the `rcMux` spinlock. `rcNormalize` maps widths to −1..+1 with a center deadband.
2. **Battery:** `updateBattery` samples about every 20 ms with `analogReadMilliVolts` and smooths with an exponential moving average (EMA). If the smoothed value stays below `CUTOFF_V` for `CUTOFF_HOLD_MS`, it sets `cutoffLatched`, which only a reset clears.
3. **Mix:** `left = T + S` and `right = T − S`, each clamped to ±1. Tank-style mixing is intentional, including in reverse: the nose always turns toward the stick. Corner behavior: centered throttle with full steering spins in place; full throttle with full steering freezes the inside motor.
4. **Safety gates:** motors are forced to 0 when `cutoffLatched` is set, when the signal is lost (no pulse for `RC_TIMEOUT_MS`), or when the bot is not `armed`. Arming requires both sticks to read neutral, and signal loss disarms, so the sticks must be re-centered.
5. **Output:** a slew limit (`SLEW_PER_SEC`) applies, except that cutoff and signal loss stop the motors immediately. `setMotor` sends PWM to IN1 for forward and IN2 for reverse; both at 0 lets the motor coast. PWM is 20 kHz at 10-bit resolution. `pwmWrite` calls `ledcWrite` only when the duty changes.

Debug switches: `RC_STATS` prints per-channel pulse min/max/avg/spread once per second. `RC_LOOPBACK` outputs a fixed `LOOPBACK_US` pulse with LEDC on `PIN_LOOPBACK` and forces the motors off; jumper that pin to an RC input to check measurement accuracy (the expected spread is a few µs at most).

All tunables are `constexpr` values at the top of the file: pins, RC calibration, reverse and invert flags, PWM settings, divider resistors, `BAT_CAL`, and cutoff thresholds. Keep new configuration there. To correct battery-reading error, users scale `BAT_CAL` by (multimeter reading ÷ printed voltage) rather than editing the divider math.

Hardware assumptions: the battery divider is R1 = 330k over R2 = 100k into GPIO2 (ADC1), with a 100 nF capacitor recommended at the ADC pin. Default pins avoid the C6 strapping pins (8, 9, 15), USB (12, 13) and UART0 (16, 17).
