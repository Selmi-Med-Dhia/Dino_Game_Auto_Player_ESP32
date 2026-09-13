# Dino Game Auto Player — ESP32

ESP32 hardware auto-player for Chrome Dino using one analog light sensor and one servo.

- Servo: GPIO 18
- Light sensor: analog GPIO 15
- Serial: 115200 baud
- Servo rest / press: 35° / 38°
- Runtime: FreeRTOS tasks only; `loop()` stays idle

## Default behavior

The firmware is now **armed automatically after every ESP32 boot/reset**.

You do not need to type `arm`.

After boot, it waits for the next cactus. When that first cactus finishes passing the sensor:

```text
round time = 0
```

That same first cactus is still handled normally. From then on, acceleration is calculated only from elapsed round time.

If you reset the ESP32 in the middle of a Dino run, the round clock is cleared. The next cactus becomes the new `t = 0`.

## Acceleration model

Acceleration no longer depends on how long a cactus stays under the sensor.

That approach is unsuitable because single, double and triple cactus groups have different optical pulse widths.

The firmware now uses a deterministic Chrome-Dino-style speed curve:

```text
speed = 6.0 + 0.060 * round_seconds
maximum speed = 13.0
```

The first cactus after boot/reset is the reference point (`speed x1.00`).

Only timings related to horizontal screen motion are scaled:

```text
effective travel = configured travel / speed factor
effective gap    = configured gap / speed factor
```

The physical jump calibration does **not** scale:

```text
SHORT hold = 80 ms
LONG  hold = 160 ms
SHORT air  = 450 ms
LONG  air  = 520 ms
servo angles stay unchanged
```

With the default `travel 1550` and `gap 40`:

| Round time | Speed factor | Travel | Gap |
|---:|---:|---:|---:|
| 0 s | x1.00 | 1550 ms | 40 ms |
| 15 s | x1.15 | 1348 ms | 35 ms |
| 30 s | x1.30 | 1192 ms | 31 ms |
| 60 s | x1.60 | 969 ms | 25 ms |
| 90 s | x1.90 | 816 ms | 21 ms |
| ~117 s+ | x2.17 | ~715 ms | ~18 ms |

`mintravel` still limits how low travel can go.

## Short and long jumps

The existing dual-jump behavior is unchanged.

```text
SHORT: hold 80 ms, modeled air 450 ms
LONG:  hold 160 ms, modeled air 520 ms
```

The optical sensor footprint is approximately 2× a normal cactus width. The firmware keeps a recent pulse history only for estimating obstacle/group width and selecting SHORT vs LONG.

Pulse duration is **not used for speed anymore**.

Default:

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

The firmware prints the command reminder after every command.

## Commands

```text
start | arm | stop | click | longclick | show | sensor | reset | defaults
theme auto|light|dark

threshold rest press hold longhold air longair longat
actuator travel mintravel landing clearance gap sample cooldown rearm
ratio adapt debug
```

Old `set ...` syntax still works.

### Important commands

`show`

Shows current round state, elapsed time, speed factor and effective timing.

`reset`

Resets the round clock and learned width history while keeping autoplay armed and preserving all saved configuration.

```text
[RESET] Round reset. Autoplay remains armed; the next cactus becomes t=0.
```

`start`

Resets the round and sends one short servo click to start/restart Dino. The first cactus after that click becomes round `t = 0`.

`arm`

Resets the round without pressing the key. Usually unnecessary because the firmware auto-arms on boot.

`stop`

Stops automatic play.

`adapt on`

Enables round-time acceleration scaling.

`adapt off`

Keeps `travel` and `gap` fixed.

## Debug output

Recommended:

```text
debug on
```

A planned jump now looks like:

```text
[PLAN] #12 SHORT width=1.05 round=43.2s speed=x1.43 travel=1084 gap=28 hold=80 cmd-in=220 late=0 exit-aligned
```

Useful fields:

- `round` — seconds since the first cactus passed.
- `speed` — deterministic round-time speed factor.
- `travel` — current scaled sensor-to-Dino timing.
- `gap` — current scaled obstacle-envelope gap.
- `SHORT` / `LONG` — selected jump type.
- `width` — estimated obstacle/group width.
- `late` — scheduling lateness.

For a normal run, `speed` should rise steadily with time even if every obstacle is a different cactus group.

## Current starting configuration

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
```

All changed settings are saved immediately in ESP32 NVS.

## Notes on `adaptstep`

`adaptstep` remains in the saved v3 configuration for backward compatibility, but timed acceleration does not use per-obstacle adaptation anymore. Existing NVS data therefore remains compatible without being erased.

## FreeRTOS tasks

| Task | Core | Priority | Job |
|---|---:|---:|---|
| Servo | 1 | 5 | Executes scheduled short/long key presses |
| Sensor / planner | 0 | 4 | Samples sensor, tracks round clock, classifies obstacles, plans jumps |
| Serial | 0 | 2 | Commands and queued runtime logs |

Sensor and servo tasks never print directly to Serial.

## Rollback

The previously working pre-acceleration version remains preserved on branch:

```text
working-without-acceleration
```

That branch points to commit:

```text
d976f2a79c45c9dc18495b80f0875f7248376cbf
```

## Hardware notes

GPIO 15 is an ESP32 ADC2/strapping pin. This project does not use Wi-Fi. Make sure the sensor circuit does not force an invalid boot strap level during reset.

Power anything larger than a tiny servo from a suitable external supply and connect the grounds together.
