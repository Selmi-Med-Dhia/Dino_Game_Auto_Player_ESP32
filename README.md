# Dino Game Auto Player — ESP32

ESP32 hardware auto-player for Chrome Dino using one analog light sensor and one servo.

- Servo: GPIO 18
- Light sensor: analog GPIO 15
- Serial: 115200 baud
- Servo rest / press: 35° / 38°
- Runtime: FreeRTOS tasks only; `loop()` stays idle

## Default behavior

The firmware is armed automatically after every ESP32 boot/reset.

It waits for the next cactus. When that first cactus finishes passing the sensor, that obstacle becomes the timing baseline:

```text
round time = 0
scale      = x1.000
```

The first cactus is still jumped normally.

## Why the previous acceleration model drifted

The base `travel` value is calibrated at the first cactus seen by the sensor, not at the instant the browser game starts.

Chromium-style Dino code starts at:

```text
SPEED        = 6
ACCELERATION = 0.001 per animation-frame update
MAX_SPEED    = 13
CLEAR_TIME   = 3000 ms
```

The game already spends about 3000 ms accelerating before it is allowed to create the first obstacle. Therefore the first cactus is already moving faster than speed 6.

At 60 Hz:

```text
first cactus baseline ≈ frame 180
baseline game speed   ≈ 6.180
```

At 144 Hz:

```text
first cactus baseline ≈ frame 432
baseline game speed   ≈ 6.432
```

Your configured:

```text
travel 1550
gap 40
```

must remain exactly those values at the first-cactus baseline. Scaling starts after that.

The firmware now uses:

```text
base_frame = firstdelay * gamefps / 1000
base_speed = 6 + 0.001 * base_frame

current_frame = base_frame + round_elapsed_frames
current_speed = 6 + 0.001 * current_frame

scale = current_speed / base_speed

effective_travel = travel / scale
effective_gap    = gap / scale
```

Speed is capped at 13.

## chromedino.com

`chromedino.com` is a hosted Chromium-derived runner, not the browser's built-in `chrome://dino` page.

Public reverse-engineering of the site shows the familiar global `Runner.instance_` API and Chromium-style update rule:

```text
currentSpeed += config.ACCELERATION
```

Its historical game script is the Chromium-derived `game.js?v=2`, so the main speed constants are compatible with this firmware. The important mismatch was the first-cactus baseline described above.

For `chromedino.com`, start with:

```text
firstdelay 3000
```

If long-run timing is still slightly early/late, adjust `firstdelay` in small steps:

```text
firstdelay 3200
firstdelay 2800
```

This value is persisted in NVS.

## Best game targets

### Exact browser target

Use Chrome's built-in page:

```text
chrome://dino
```

This is the safest target because it is the actual Chromium game.

### HTTPS Chromium-derived target

A good online alternative is:

```text
https://litetex.github.io/t-rex-runner/
```

That project is maintained as a standalone extraction of Chromium's T-Rex runner.

## Browser refresh rate

The game update uses `requestAnimationFrame()`, so the rate of `+0.001` steps follows the animation callback rate.

Set:

```text
gamefps 60
```

or your display refresh rate, for example:

```text
gamefps 144
```

`gamefps` is persisted in NVS.

## Short and long jumps

The dual-jump behavior is unchanged:

```text
SHORT: hold 80 ms, modeled air 450 ms
LONG:  hold 160 ms, modeled air 520 ms
```

The optical pulse is still used to estimate obstacle/group width and choose SHORT vs LONG.

It is **not** used to estimate game speed.

Defaults:

```text
ratio 2.0
longat 1.60
```

## Serial commands

```text
start | arm | stop | click | longclick | show | sensor | reset | defaults
theme auto|light|dark

threshold rest press hold longhold air longair longat
actuator travel mintravel landing clearance gap sample cooldown rearm
ratio gamefps firstdelay adapt debug
```

Important settings:

```text
gamefps 60
firstdelay 3000
adapt on
debug on
```

## Debug output

With:

```text
debug on
```

planner output includes both the estimated absolute game speed and the scale relative to the first cactus:

```text
[PLAN] #12 SHORT width=1.05 round=30.0s frame=1980 game=7.980 scale=x1.291 travel=1200 gap=31 hold=80 ...
```

At 60 Hz with `firstdelay 3000`:

```text
round 0 s:
base frame = 180
game speed = 6.180
scale      = x1.000
travel     = 1550 ms

round 30 s:
frame      = 1980
game speed = 7.980
scale      = x1.291
travel     ≈ 1200 ms
```

This is different from incorrectly scaling against speed 6.000.

## Round controls

`reset`

Clears the round clock and width history, keeps autoplay armed, and makes the next cactus the new x1.000 timing baseline.

`start`

Resets the round and sends one short click to start/restart Dino.

`arm`

Resets the round without pressing the key.

`stop`

Stops automatic play.

`adapt on`

Enables frame-based acceleration scaling.

`adapt off`

Uses fixed `travel` and `gap`.

## Current base configuration

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

gamefps 60
firstdelay 3000
adapt on
```

All settings are persisted in NVS. `firstdelay`, `gamefps`, and `ratio` are stored separately so the firmware-v3 `Config` layout remains unchanged.

## FreeRTOS tasks

| Task | Core | Priority | Job |
|---|---:|---:|---|
| Servo | 1 | 5 | Executes scheduled short/long key presses |
| Sensor / planner | 0 | 4 | Samples sensor, tracks round timing, classifies obstacles and plans jumps |
| Serial | 0 | 2 | Commands and queued runtime logs |

Sensor and servo tasks do not print directly to Serial.

## Rollback

The known-good pre-acceleration version is preserved on branch:

```text
working-without-acceleration
```

pointing to:

```text
d976f2a79c45c9dc18495b80f0875f7248376cbf
```

## Hardware notes

GPIO 15 is an ESP32 ADC2/strapping pin. Wi-Fi is not used here. Make sure the sensor circuit does not force an invalid boot strap level during reset.

Power anything larger than a tiny servo from a suitable external supply and connect the grounds together.
