# YarnBot

YarnBot is an ESP32-C6 sketch for a two-motor, tank-steered robot. It reads two channels from a standard RC receiver, mixes them into left and right motor commands, and drives the motors through two IBT-4 H-bridge boards. It also watches the 3S LiPo and shuts the motors off before the battery is over-discharged.

## Hardware

- An ESP32-C6 board
- 2 × IBT-4 motor driver boards
- 2 × brushed DC motors
- An RC transmitter and receiver giving standard servo PWM (1000–2000 µs, ~50 Hz)
- A 3S LiPo (12.6 V full)
- A battery voltage divider: R1 = 330 kΩ (battery + to ADC pin), R2 = 100 kΩ (ADC pin to GND), plus a 100 nF capacitor from the ADC pin to GND

### Wiring (default pins)

| Signal | ESP32-C6 GPIO |
|---|---|
| RC throttle channel | 18 |
| RC steering channel | 20 |
| Left IBT-4 IN1 / IN2 | 19 / 21 |
| Right IBT-4 IN1 / IN2 | 22 / 23 |
| Battery divider midpoint | 2 |

The receiver, both IBT-4 boards, the ESP32 and the battery must all share a common ground. If your board doesn't break out some of these pins, change the pin constants at the top of `YarnBot.ino`. Avoid GPIO 8, 9 and 15 (strapping pins), 12 and 13 (USB) and 16 and 17 (UART0).

## Controls

Throttle drives forward and reverse; steering turns left and right. The sketch combines them tank-style: the left motor gets throttle + steering and the right motor gets throttle − steering, each capped at full power.

| Throttle | Steering | Result |
|---|---|---|
| Center | Full right | Spins clockwise in place: left forward, right reverse |
| Full forward | Full right | Pivots right: left motor full forward, right motor stopped |
| Full reverse | Full right | Pivots in reverse: left motor stopped, right motor full reverse |

Steering left mirrors each case. In reverse the nose still turns toward the stick, like a tank rather than a car.

## Safety features

- **Arming:** the motors stay off at power-up until both sticks have been seen centered.
- **Signal loss:** if either channel stops sending pulses for 100 ms, the motors stop immediately. The robot disarms, so you must re-center the sticks before driving again.
- **Low-battery cutoff:** if the battery voltage stays below 9.9 V for 1 second, the motors are shut off until the ESP32 is reset or power-cycled. Readings are averaged, so brief sag under heavy throttle won't trip it. If the board has an onboard RGB LED, it blinks red while cut off.
- **Slew limiting:** changes in motor power are rate-limited to soften current spikes, for example when going from full forward straight to full reverse.

## Building and uploading

This sketch needs [arduino-cli](https://arduino.github.io/arduino-cli/) and the Espressif ESP32 Arduino core **3.x**. Older 2.x cores don't support the ESP32-C6.

```
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

Then, from this folder:

```
arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc .
arduino-cli upload -p <port> --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc .
arduino-cli monitor -p <port> -c baudrate=115200
```

`CDCOnBoot=cdc` sends Serial output over the C6's native USB port. Without it, Serial goes to the UART0 pins (GPIO 16/17) and the USB serial monitor stays blank. In the Arduino IDE, the same setting is **Tools → USB CDC On Boot → Enabled**. On Linux the native USB port usually shows up as `/dev/ttyACM0`.

## Configuration

Every setting is a constant at the top of `YarnBot.ino`:

| Setting | Purpose |
|---|---|
| `PIN_*` | Pin assignments |
| `RC_MIN`, `RC_MID`, `RC_MAX`, `RC_DEADBAND` | RC pulse calibration and center deadband |
| `REVERSE_THROTTLE`, `REVERSE_STEER` | Flip a channel's direction |
| `INVERT_LEFT`, `INVERT_RIGHT` | Flip a motor's direction instead of rewiring it |
| `SLEW_PER_SEC` | How fast motor power may change (0 disables limiting) |
| `CUTOFF_V`, `CUTOFF_HOLD_MS` | Low-battery threshold and how long the voltage must stay below it |
| `BAT_CAL` | Battery reading correction factor |
| `DEBUG` | Turn the Serial debug output on (1) or off (0) |
| `RC_STATS` | Print each channel's pulse min/max/average once per second (1) |
| `RC_LOOPBACK`, `PIN_LOOPBACK`, `LOOPBACK_US` | Bench test of the RC measurement; see [Bench testing](#bench-testing) |

## Bench testing

Lift the wheels off the ground and open the Serial monitor at 115200 baud. Five times a second the sketch prints a line like this:

```
thr=1500 str=1500 | T=+0.00 S=+0.00 | L=+0.00 R=+0.00 | 12.31V
```

From left to right: the raw throttle and steering pulse widths, the normalized throttle and steering values, the left and right motor outputs, and the battery voltage. `NOSIG`, `DISARMED` or `CUTOFF` is appended when one of those states is active.

1. Check that the pulse widths read about 1000 µs, 1500 µs and 2000 µs at the stick ends and center. If they don't, adjust `RC_MIN`, `RC_MID` and `RC_MAX`.
2. Check each stick corner against the [Controls](#controls) table. Use `INVERT_LEFT`, `INVERT_RIGHT` or the `REVERSE_*` flags to fix any direction that's wrong.
3. Switch off the transmitter. The motors should stop right away.

### Checking RC signal quality

Set `RC_STATS` to 1 to get a line like this once per second:

```
STATS thr n=50 min=1500 max=1500 avg=1500.0 spread=0 | str n=58 min=1500 max=1500 avg=1500.0 spread=0
```

`n` is the number of pulses received that second, and `spread` is max − min. With the sticks centered, the spread should be a few µs at most. `RC_DEADBAND` only has to cover that spread plus any slop in the stick centering. If the average isn't near `RC_MID`, adjust the subtrim on your transmitter or change `RC_MID`.

To check the ESP32's measurement without the receiver, set `RC_LOOPBACK` to 1. The sketch then outputs a fixed `LOOPBACK_US` pulse (1500 µs) on `PIN_LOOPBACK` (GPIO 1) and keeps the motors off. Unplug the receiver from the throttle input, jumper `PIN_LOOPBACK` to GPIO 18, and the throttle stats should show exactly that width. Set `RC_LOOPBACK` back to 0 when you're done.

## Battery calibration

Resistor tolerance and ADC error make the printed battery voltage a little off. To correct it:

1. Power up with the motors idle and let the printed voltage settle for a couple of seconds.
2. Measure the battery with a multimeter between battery + (at the 330 kΩ resistor) and the shared ground.
3. Set `BAT_CAL` to its current value × (multimeter reading ÷ printed voltage). For example, if the meter reads 12.20 V and Serial prints 11.95 V, 1.00 × 12.20 ÷ 11.95 gives `BAT_CAL = 1.021f`.
4. Re-upload and check again.

Accuracy matters most near the 9.9 V cutoff. If you have an adjustable bench supply, calibrate at about 10 V.

## License

MIT. See [LICENSE](LICENSE).
