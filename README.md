# Dino Game Auto Player — ESP32

ESP32 hardware auto-player for Chrome Dino using one analog light sensor and one servo.

- **Servo:** GPIO 18
- **Light sensor:** analog GPIO 15
- **White:** ADC > 200
- **Black:** ADC < 200
- **Servo rest / press:** 35° / 38°
- **Serial:** 115200 baud

The runtime uses FreeRTOS tasks. `loop()` stays idle.

## Current behavior

The firmware now handles three things automatically:

1. **SHORT jump** for a normal/small obstacle.
2. **LONG jump** for a wide cactus group.
3. **Game acceleration** by scaling horizontal timing as Chrome gets faster.

Your existing jump calibration remains the base calibration:

```text
SHORT: hold 80 ms   air 450 ms
LONG:  hold 160 ms  air 520 ms
```

The servo angles, hold times and vertical jump airtimes are **not** scaled with game speed. Chrome's acceleration changes horizontal scrolling speed; it does not make the servo or the Dino's vertical jump physics run proportionally faster.

The values that do scale are:

```text
travel   sensor -> Dino travel time
gap      optical clear-gap used to finish an obstacle envelope
```

Example:

```text
speed x1.00 -> travel 1550 ms, gap 40 ms
speed x1.50 -> travel ~1033 ms, gap ~27 ms
speed x2.00 -> travel ~775 ms,  gap ~20 ms
```

## Acceleration tracking

Chrome normal mode starts around speed `6`, accelerates continuously, and caps around speed `13`.

The ESP32 does not simply assume elapsed time is perfect. It uses the light sensor:

1. Collect recent optical obstacle pulses.
2. Use the small-pulse reference as a speed signal.
3. Learn a baseline after the first few obstacles.
4. Compare the current reference pulse with that baseline.
5. A shorter pulse means the screen is moving faster.
6. Scale `travel` and `gap` by the resulting speed factor.

When the run was started with `start`, the firmware also compensates for the small amount of Chrome acceleration that happened while the initial optical baseline was being learned. After that, pulse timing drives the scale.

`adaptstep` limits how much the estimate can increase from one obstacle to the next so one noisy pulse cannot suddenly change the timing.

Defaults:

```text
adapt on
adaptstep 3
```

For a normal run, prefer:

```text
start
```

rather than `arm`, because `start` gives the ESP32 a known game start point. `arm` still adapts, but its speed scale is relative to the speed at which it was armed.

## Short and long jumps

The sensor footprint is approximately **2× one normal cactus width**.

The firmware estimates the fixed optical footprint and then estimates the actual obstacle/group width.

Default:

```text
ratio 2.0
longat 1.60
```

Approximately:

```text
width ~1.0 -> SHORT
width >=1.60 -> LONG
```

Manual tests:

```text
click
longclick
```

## FreeRTOS tasks

| Task | Core | Priority | Job |
|---|---:|---:|---|
| Servo | 1 | 5 | Executes scheduled key presses |
| Sensor / planner | 0 | 4 | ADC sampling, filtering, width/speed estimation and planning |
| Serial | 0 | 2 | Commands, configuration and runtime logs |

The sensor and servo tasks do not print directly to Serial. Runtime logs are queued for the serial task so UART output cannot block obstacle detection.

## Serial Monitor

### PlatformIO / VS Code

1. Connect the ESP32 over USB.
2. Open the project in VS Code with PlatformIO.
3. Upload the firmware.
4. Open **PlatformIO → Project Tasks → upesy_wroom → Monitor**.
5. Baud is already configured to **115200**.
6. Click the terminal, type a command and press **Enter**.

### Arduino IDE

Open **Tools → Serial Monitor** and select:

```text
Baud: 115200
Line ending: New Line
```

`Both NL & CR` also works.

The firmware prints the short command manual again after every command.

## Main commands

```text
start
arm
stop

click
longclick

show
sensor
reset
defaults

theme auto
theme light
theme dark
```

Change settings directly:

```text
travel 1550
ratio 2.0
longat 1.60
adapt on
adaptstep 3
```

Old syntax such as this still works:

```text
set travel_ms 1550
```

## Recommended configuration

```text
threshold 200
rest 35
press 38

hold 80
longhold 160
air 450
longair 520
longat 1.60

actuator 160
travel 1550
mintravel 350
landing 50
clearance 100

gap 40
sample 5
ratio 2.0

adapt on
adaptstep 3
```

All configuration changes are saved to ESP32 NVS immediately.

## Watching acceleration

Use:

```text
debug on
start
```

A planner line now includes the measured speed factor and scaled timing:

```text
[PLAN] #12 SHORT width=1.04 speed=x1.37 pulse=105 travel=1131 gap=29 hold=80 cmd-in=245 late=0 exit-aligned
```

Later in the same run you might see:

```text
[PLAN] #31 LONG width=2.08 speed=x1.82 pulse=86 travel=852 gap=22 hold=160 cmd-in=126 late=0 exit-aligned
```

Important fields:

- `SHORT` / `LONG` — selected jump.
- `width` — estimated obstacle width.
- `speed=x...` — measured relative game speed.
- `travel` — current scaled sensor-to-Dino time.
- `gap` — current scaled envelope finalization gap.
- `hold` — actual servo hold used for that jump.
- `late` — whether the ideal command time was already missed.

Run:

```text
show
```

to see the current speed estimate and base/effective timing.

## Tuning acceleration

Normally leave:

```text
adapt on
adaptstep 3
```

If acceleration correction visibly lags behind the game, try:

```text
adaptstep 4
```

or:

```text
adaptstep 5
```

Do not immediately use very large values. The pulse history already filters cactus-width variation; `adaptstep` is a second safety limit.

If you want to temporarily compare against the old fixed-speed behavior:

```text
adapt off
reset
```

Then turn it back on:

```text
adapt on
reset
```

`reset` clears learned width/speed history but keeps saved settings.

## General timing calibration

Your base `travel` is still the timing at approximately the beginning of a run.

- All jumps too late near the **start**: decrease `travel` in 20–40 ms steps.
- All jumps too early near the **start**: increase `travel`.
- Start is good but later game becomes late: increase `adaptstep` slightly.
- Start is good but later game becomes too early: decrease `adaptstep`.
- Servo physically reaches the key later than modeled: increase `actuator`.
- A wide group gets SHORT: lower `longat` slightly.
- Too many single cacti get LONG: raise `longat`.
- A long jump is physically too short: increase `longhold`.
- Repeated `late > 0`: check sensor position, `travel`, `actuator`, and whether speed scaling is learning correctly.

## Persistent configuration

Settings are stored using ESP32 `Preferences` / NVS.

- Power loss keeps configuration.
- `reset` only clears runtime learning.
- `defaults` restores firmware defaults and saves them.
- Existing v3 calibration is preserved by this acceleration update.

## Rollback point

The last confirmed working firmware before acceleration scaling is preserved at:

```text
working-without-acceleration
```

That branch points to commit:

```text
d976f2a79c45c9dc18495b80f0875f7248376cbf
```

## Hardware notes

- GPIO 15 is an ESP32 ADC2/strapping pin. Wi-Fi is not used here.
- Make sure the sensor circuit does not force an invalid boot strap level during reset.
- Use a suitable external supply for the servo when necessary and connect grounds together.
