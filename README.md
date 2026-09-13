# Dino Game Auto Player — ESP32

ESP32 hardware auto-player for Chrome Dino using one analog light sensor and one servo.

- **Servo:** GPIO 18
- **Light sensor:** analog GPIO 15
- **White:** ADC > 200
- **Black:** ADC < 200
- **Servo rest:** 35°
- **Servo press:** 38°
- **Serial:** 115200 baud

The runtime uses FreeRTOS tasks only. `loop()` does no application work.

## How it works

Three tasks run independently:

1. **Sensor task** — samples the light sensor every 5 ms, filters the samples, detects obstacle envelopes, and plans jumps.
2. **Servo task** — waits for precisely scheduled click commands, presses the key, then retracts the servo.
3. **Serial task** — handles commands and configuration without disturbing sensor timing.

The sensor task uses one ADC conversion per 5 ms period (200 Hz). A rolling median of three samples plus a small hysteresis removes single-sample noise without continuously hammering the ADC.

## Important sensor-width correction

The physical sensor spot is approximately **2× the width of a normal cactus**.

That matters because the sensor starts seeing a cactus before the cactus center reaches the sensor and stops seeing it after the cactus has passed. The firmware therefore does not use the raw optical pulse edges as cactus edges.

With `ratio 2.0`, it removes about **one third of the measured pulse duration from each optical edge** before calculating the cactus position. This is also why the default clear gap is only **40 ms**: the wide sensor already merges very close cactus shapes naturally.

`ratio` is configurable and saved in flash.

## FreeRTOS layout

| Task | Core | Priority | Behavior |
|---|---:|---:|---|
| Servo | 1 | 5 | Blocks until a command is due; no polling loop |
| Sensor / planner | 0 | 4 | Periodic with `vTaskDelayUntil()` |
| Serial console | 0 | 2 | Checks input every 10 ms |

The Arduino `loop()` simply blocks indefinitely.

## Serial Monitor guide

### PlatformIO / VS Code

1. Connect the ESP32 by USB.
2. Open this project in VS Code with PlatformIO.
3. Upload the firmware.
4. Open **PlatformIO → Project Tasks → upesy_wroom → Monitor**, or use the Serial Monitor button in the PlatformIO toolbar.
5. The project already sets the monitor speed to **115200** in `platformio.ini`.
6. Click inside the monitor terminal, type a command, and press **Enter**.

Example:

```text
show
```

The ESP32 responds and then prints the small command manual again. It does this after every command, so you do not have to remember the syntax.

### Arduino IDE

If you use Arduino IDE instead:

1. Open **Tools → Serial Monitor**.
2. Set baud rate to **115200**.
3. Select **New Line** or **Both NL & CR** as the line ending.
4. Type a command and press **Enter**.

## First setup

### 1. Check the sensor

Put the sensor over normal white game background and run:

```text
sensor
```

Then put a dark cactus / dark game object under the same sensor area and run it again.

The intended setup is:

```text
white > 200
black < 200
```

If your readings are different, choose a threshold roughly halfway between the two values:

```text
threshold 200
```

For example, if white is around 350 and black is around 90, a threshold around 220 is reasonable.

### 2. Check the servo

Run:

```text
click
```

The servo should press the key and return immediately.

Your current defaults are:

```text
rest 35
press 38
hold 80
```

### 3. Check the saved setup

Run:

```text
show
```

Configuration changes are written to ESP32 **NVS flash** immediately. Powering the board off does not erase them.

### 4. Start the game

If Dino is stopped:

```text
start
```

This arms autoplay and sends one click to start/restart the game.

If Dino is already running:

```text
arm
```

To stop autoplay:

```text
stop
```

## Commands

The firmware prints this compact reminder after every command:

```text
start | arm | stop | click | show | sensor | reset | defaults
theme auto|light|dark
<name> <value>  (auto-saved)
threshold rest press hold actuator travel mintravel air landing
clearance gap sample cooldown rearm ratio adapt adaptstep debug
```

Examples:

```text
press 38
rest 35
threshold 200
travel 1550
ratio 2.0
sample 5
gap 40
adapt on
debug on
```

The older syntax still works too:

```text
set travel_ms 1550
set click_angle 38
```

## Recommended starting configuration

```text
threshold 200
rest 35
press 38
hold 80
actuator 160
travel 1550
air 450
landing 50
clearance 100
gap 40
sample 5
ratio 2.0
adapt on
adaptstep 3
```

These values are starting points, not a replacement for final physical calibration. The most hardware-dependent value is `travel` because it depends on exactly where the sensor is placed relative to the Dino.

## Calibration guide

Start with:

```text
debug on
start
```

A planned jump looks like:

```text
[PLAN] #4 pulse=165 travel=1390 cmd-in=310 late=0 exit-aligned
```

Use these rules:

- **Dino jumps too late:** decrease `travel` in small steps, for example 20–40 ms.
- **Dino jumps too early:** increase `travel` in small steps.
- **Servo physically reaches the key later than expected:** increase `actuator`.
- **Dino lands on the end of a cactus:** increase `landing` by 10–20 ms.
- **One cactus is being split into several detections:** increase `gap` slightly, for example 40 → 50 ms.
- **Separate obstacles are being treated as one group:** decrease `gap`.
- **`late` is repeatedly above 0:** the complete obstacle envelope was learned too late for the requested timing. Prefer moving the sensor farther ahead; also check `gap`, `actuator`, and `travel`.
- **Sensor reading flickers around 200:** first improve physical alignment and lighting. The firmware already uses a 3-sample median and ADC hysteresis; avoid making `sample` extremely small.

After moving the sensor or making a large timing change, run:

```text
reset
```

This clears learned speed/envelope history but keeps all saved settings.

## Speed adaptation

With one light sensor, exact game speed cannot be measured independently because cactus widths vary. However, your sensor footprint is wider than a cactus, which makes optical pulse duration more stable than with a tiny point sensor.

The firmware therefore uses a conservative low-percentile estimate from the last seven obstacle pulses. It only shortens the estimated sensor-to-Dino travel time gradually as the game accelerates.

Defaults:

```text
adapt on
adaptstep 3
mintravel 350
```

The extra safety bias intentionally avoids adapting too aggressively from one unusually narrow cactus.

## Persistent configuration

Settings are stored using ESP32 `Preferences` / NVS.

- Changing a value saves it immediately.
- Resetting or unplugging the ESP32 keeps it.
- `reset` keeps saved configuration and only clears learned run timing.
- `defaults` restores the firmware defaults and saves them.
- A full ESP32 flash/NVS erase will erase the saved values.

Firmware version 2 migrates version-1 saved settings. Your custom values are retained; old untouched timing defaults are upgraded to the safer task-based defaults.

## Notes

- GPIO 15 is an ESP32 ADC2/strapping pin. This project does not use Wi-Fi, so the ADC2/Wi-Fi conflict is not relevant during normal operation. Make sure the sensor circuit does not force an invalid boot strap level during reset.
- Power anything larger than a tiny servo from a suitable external supply and connect its ground to ESP32 ground.
