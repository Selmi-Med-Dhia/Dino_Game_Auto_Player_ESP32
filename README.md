# Dino Game Auto Player — ESP32

ESP32 hardware auto-player for Chrome Dino using one analog light sensor and one servo.

- Servo: GPIO 18
- Light sensor: analog GPIO 15
- Serial: 115200 baud
- Servo rest / press: 35° / 38°
- Runtime: FreeRTOS tasks only; `loop()` stays idle

## Default behavior

The firmware is **armed automatically after every ESP32 boot/reset**.

After boot it waits for the next cactus. When that first cactus finishes passing the sensor, the ESP defines:

```text
round frame = 0
round time  = 0
```

That cactus is still handled normally. If the ESP32 is reset during a game, the next cactus becomes the new frame-0 reference.

## Exact Chromium acceleration rule

The Chromium Dino source does **not** increase speed every N cacti.

Normal mode uses:

```text
start speed   = 6
acceleration  = 0.001 per animation-frame update
maximum speed = 13
```

The game update loop does the equivalent of:

```text
if (currentSpeed < 13)
    currentSpeed += 0.001;
```

So speed changes in discrete but very small steps: one `0.001` step for each browser animation frame.

The ESP mirrors that rule from the first-cactus reference:

```text
frame = floor(round_time_ms * gamefps / 1000)
frame = min(frame, 7000)

speed = 6 + 0.001 * frame
speed = min(speed, 13)
```

Only horizontal-motion timings scale:

```text
effective travel = configured travel / (speed / 6)
effective gap    = configured gap    / (speed / 6)
```

Servo angles, short/long key holds and vertical jump airtime remain unchanged.

## Important: browser refresh rate

Chromium schedules the game update with `requestAnimationFrame()`. Browser animation callbacks normally follow the display refresh rate.

Therefore the exact time required to accumulate 7000 speed steps depends on the monitor running Chrome.

Default:

```text
gamefps 60
```

Examples:

| Display | +0.001 steps/sec | Time from 6 to 13 |
|---:|---:|---:|
| 60 Hz | 60 | ~116.7 s |
| 75 Hz | 75 | ~93.3 s |
| 120 Hz | 120 | ~58.3 s |
| 144 Hz | 144 | ~48.6 s |
| 165 Hz | 165 | ~42.4 s |

Set the value to the refresh rate of the display where Chrome Dino is running:

```text
gamefps 144
```

On Windows this is normally visible under **Settings → System → Display → Advanced display → Choose a refresh rate**.

`gamefps` is stored separately in NVS, so it survives power loss without changing the existing firmware-v3 configuration layout.

## Why the game can look like it speeds up in bigger steps

Some obstacle behavior unlocks only after speed thresholds. That can make progression look step-like even though speed itself is updated every animation frame.

Examples from the Chromium game logic include:

```text
large cactus multiples: allowed from speed 7
pterodactyl:            allowed from speed 8.5
```

So the appearance of new obstacle patterns is stepped; the speed update itself is not based on cactus count.

## Default 60 Hz timing example

With:

```text
travel 1550
gap 40
gamefps 60
```

the frame-stepped model is approximately:

| Round time | Emulated frame | Game speed | Speed factor | Travel | Gap |
|---:|---:|---:|---:|---:|---:|
| 0 s | 0 | 6.000 | x1.000 | 1550 ms | 40 ms |
| 15 s | 900 | 6.900 | x1.150 | 1348 ms | 35 ms |
| 30 s | 1800 | 7.800 | x1.300 | 1192 ms | 31 ms |
| 60 s | 3600 | 9.600 | x1.600 | 969 ms | 25 ms |
| 90 s | 5400 | 11.400 | x1.900 | 816 ms | 21 ms |
| ~116.7 s+ | 7000 | 13.000 | x2.167 | ~715 ms | ~18 ms |

Between those rows, the value changes one frame step at a time rather than as a continuous formula.

## Short and long jumps

The existing dual-jump behavior is unchanged:

```text
SHORT: hold 80 ms, modeled air 450 ms
LONG:  hold 160 ms, modeled air 520 ms
```

The optical sensor footprint is approximately 2× a normal cactus width. Pulse duration remains useful for estimating obstacle/group width and selecting SHORT vs LONG.

**Pulse duration is not used for acceleration.**

Defaults:

```text
ratio 2.0
longat 1.60
```

## Serial Monitor

PlatformIO / VS Code:

1. Connect the ESP32.
2. Upload the firmware.
3. Open **PlatformIO → Project Tasks → upesy_wroom → Monitor**.
4. Use 115200 baud.
5. Type a command and press Enter.

Arduino IDE:

1. Open **Tools → Serial Monitor**.
2. Set **115200 baud**.
3. Select **New Line** or **Both NL & CR**.

The compact command reminder is printed after every command.

## Commands

```text
start | arm | stop | click | longclick | show | sensor | reset | defaults
theme auto|light|dark

threshold rest press hold longhold air longair longat
actuator travel mintravel landing clearance gap sample cooldown rearm
ratio gamefps adapt debug
```

Examples:

```text
show
gamefps 60
gamefps 144
adapt on
debug on
```

Old `set ...` syntax still works.

## Round-control commands

`reset`

Clears the round/frame counter and width history while leaving autoplay armed. The next cactus becomes frame 0.

`start`

Resets the round and sends one short click to start/restart Dino. The first cactus after that becomes frame 0.

`arm`

Resets the round without pressing the key. Usually unnecessary because boot already auto-arms.

`stop`

Stops automatic play.

`adapt on`

Uses Chromium frame-step acceleration.

`adapt off`

Keeps configured `travel` and `gap` fixed.

## Debug output

Enable:

```text
debug on
```

A planner line now includes the exact emulated Chromium frame number:

```text
[PLAN] #12 SHORT width=1.05 round=30.0s frame=1800 speed=x1.300 travel=1192 gap=31 hold=80 cmd-in=220 late=0 exit-aligned
```

Useful fields:

- `round` — elapsed time since the first cactus reference.
- `frame` — emulated browser animation-frame count.
- `speed` — current speed factor relative to speed 6.
- `travel` / `gap` — current scaled horizontal timings.
- `SHORT` / `LONG` — selected jump type.
- `width` — estimated obstacle/group width.
- `late` — scheduling lateness.

`show` prints the same frame/speed state plus the configured `gamefps`.

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

adapt on
```

All normal configuration changes are saved in ESP32 NVS. `gamefps` and `ratio` are also persisted in NVS.

## Notes on `adaptstep`

`adaptstep` remains in the old saved firmware-v3 structure only for binary/NVS compatibility. It is no longer used by acceleration logic.

## FreeRTOS tasks

| Task | Core | Priority | Job |
|---|---:|---:|---|
| Servo | 1 | 5 | Executes scheduled short/long key presses |
| Sensor / planner | 0 | 4 | Samples sensor, tracks round/frame time, classifies obstacles and plans jumps |
| Serial | 0 | 2 | Commands and queued runtime logs |

Sensor and servo tasks never print directly to Serial.

## Rollback

The known-good pre-acceleration version is still preserved on branch:

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
