# Dino Game Auto Player — ESP32 + Light Sensor + Servo

Hardware version of the software-only [`Dino-Game-Auto-Player`](https://github.com/Selmi-Med-Dhia/Dino-Game-Auto-Player).

The ESP32 watches a point **ahead of the Dino** with an analog light sensor. Instead of jumping immediately when one dark sample appears, it measures the full obstacle envelope, merges short gaps, predicts when the obstacle reaches the Dino, and schedules a servo command so the Dino lands just after the obstacle's trailing edge.

Servo movement runs in its own high-priority **FreeRTOS task**, so sensor sampling and serial commands keep running while a click is waiting or being executed.

## Wiring

| Device | ESP32 |
|---|---|
| Servo signal | GPIO 18 |
| Light sensor analog output | GPIO 15 |
| Servo / sensor GND | ESP32 GND |

Default servo positions:

- Rest: **20°**
- Key press: **25°**

Default light threshold:

- ADC value **> 200** = white
- ADC value **<= 200** = black

For anything larger than a tiny servo, power the servo from a separate suitable supply and connect its ground to ESP32 ground.

> GPIO 15 is an ESP32 strapping pin. The project uses it because that is the requested sensor pin. If a particular sensor circuit prevents the board from booting, make sure it is not forcing an invalid level during reset.

## Build

This is a PlatformIO Arduino project. `platformio.ini` already includes `ESP32Servo`.

Upload it normally with PlatformIO, then open the serial monitor at **115200 baud**.

## Quick start

1. Put the light sensor on an empty background point ahead of the Dino, at cactus-body height.
2. Upload the firmware.
3. Open Serial Monitor at 115200 baud.
4. Run:

```text
sensor
```

Verify the reading is above 200 on the white background and below 200 on a black obstacle.

5. Run:

```text
start
```

`start` arms the auto-player and queues one servo click to start/restart the Dino game.

If the game is already running, use:

```text
arm
```

To stop immediately:

```text
stop
```

## Serial commands

```text
help
status
sensor

start
arm
stop
click

theme auto
theme light
theme dark

reset
defaults
```

Change a parameter with:

```text
set <parameter> <value>
```

Examples:

```text
set threshold 200
set rest 20
set click_angle 25
set hold_ms 80
set travel_ms 1550
set actuator_ms 160
set air_ms 450
set landing_ms 30
set clearance_ms 90
set gap_ms 120
set adapt on
set debug on
```

### Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `threshold` | 200 | Black/white ADC threshold |
| `rest` | 20 | Servo rest angle |
| `click_angle` | 25 | Servo key-down angle |
| `hold_ms` | 80 | Time the servo holds the key |
| `actuator_ms` | 160 | Servo command → physical key contact delay |
| `travel_ms` | 1550 | Initial time from sensor position to Dino |
| `min_travel_ms` | 350 | Minimum auto-adapted travel time |
| `air_ms` | 450 | Estimated Dino airtime |
| `landing_ms` | 30 | Desired landing time after obstacle trailing edge |
| `clearance_ms` | 90 | Minimum leading-edge safety clearance |
| `gap_ms` | 120 | Clear gap required before an obstacle envelope is finalized |
| `cooldown_ms` | 70 | Minimum separation between servo commands |
| `rearm_ms` | 12 | How soon before the previous landing a new jump may be planned |
| `theme_flip_ms` | 1500 | Sustained color reversal required for automatic day/night rebasing |
| `sample_ms` | 2 | Light-sensor polling period |
| `adapt` | on | Use obstacle-envelope history to shorten travel time as the game accelerates |
| `adapt_step` | 6 | Maximum travel-time reduction per obstacle, percent |
| `auto_theme` | on | Automatically follow Chrome Dino light/dark inversion |
| `debug` | off | Extra sensor and timing logs |

## Timing logic

The firmware follows the same hardware-oriented logic as the software simulator:

1. The sensor sees the obstacle reach the far-ahead measurement point.
2. Black/white samples are treated as an **envelope**, not independent pixels.
3. Short clear gaps are merged, which helps avoid treating different parts of one cactus/group as separate obstacles.
4. The trailing-edge time is projected from the sensor to the Dino using `travel_ms`.
5. The desired key-contact time is chosen so landing happens just after the trailing edge.
6. A leading-edge safety limit prevents waiting too long for a wide group.
7. `actuator_ms` is subtracted to determine when the ESP32 must command the servo.
8. The command is placed in a FreeRTOS queue; the dedicated servo task executes it at the requested time, presses to 25°, holds, then returns to 20°.

The software version can measure scrolling speed directly from screen frames. A single physical light sensor cannot uniquely determine absolute game speed because obstacle widths vary. This firmware therefore uses a conservative rolling short-envelope history as a speed proxy and only reduces the sensor-to-Dino travel estimate gradually. For maximum repeatability, tune `travel_ms` for your physical sensor position and use `adapt_step` to control how aggressively it follows acceleration.

If you want exact speed measurement in a later hardware revision, two horizontally separated light sensors can measure obstacle transit time directly.

## Tuning

Start with:

```text
set debug on
status
```

Watch lines such as:

```text
[PLAN] jump=4 envelope=184 ms travel=1457 ms command-in=422 ms late=0 ms reason=exit-aligned
```

Important cases:

- `reason=exit-aligned` — normal planned jump.
- `reason=entry-safety` — a wide obstacle/group forced an earlier contact.
- `reason=post-landing-rejump` — the next click was delayed until the previous jump should be ending.
- `reason=sensor-too-close` — the ideal command time had already passed. Move the sensor farther ahead or reduce latency.
- `task-late=...` — actual FreeRTOS servo-task scheduling lateness.

If jumps are consistently too early or too late, `travel_ms` is the first parameter to tune.

If the servo itself physically takes longer than expected to hit the key, tune `actuator_ms`.

If cactus forks are being split into multiple obstacles, increase `gap_ms` slightly.

## FreeRTOS behavior

The servo task is separate from `loop()` and has its own command queue. Each planned jump records the exact future execution time.

Stopping the player invalidates pending automatic commands. Old scheduled clicks cannot fire after a later restart because each run has a generation ID.

Manual `click` commands also go through the servo task.
