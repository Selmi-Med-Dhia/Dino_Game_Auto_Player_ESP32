# Dino Game Auto Player — ESP32

ESP32 hardware version of the Chrome Dino auto-player. A light sensor watches the screen ahead of the Dino and a servo presses the jump key.

The timing logic is intentionally hardware-friendly: the ESP32 measures the whole obstacle envelope, merges short gaps, estimates when the obstacle reaches the Dino, then schedules the servo through a dedicated FreeRTOS task.

## Wiring

| Device | ESP32 |
|---|---|
| Servo signal | GPIO 18 |
| Light sensor analog output | GPIO 15 |
| Grounds | Common GND |

Current servo defaults:

- Rest: **35°**
- Press: **38°**

Light sensor:

- ADC **> 200** = white
- ADC **<= 200** = black

Use a suitable external supply for the servo if needed, with its ground connected to ESP32 ground.

## Build

PlatformIO project using the Arduino framework and `ESP32Servo`.

Open the serial monitor at **115200 baud** after uploading.

## Serial interface

The console is intentionally simple. After **every command**, the ESP32 prints the short command manual again.

Main commands:

```text
start
arm
stop
click
show
sensor
reset
defaults
theme auto
theme light
theme dark
```

Change a value directly:

```text
press 38
rest 35
threshold 200
travel 1550
adapt on
debug on
```

You can still use the old syntax, for example:

```text
set click_angle 38
set travel_ms 1550
set adapt on
```

Available settings:

```text
threshold
rest
press
hold
actuator
travel
mintravel
air
landing
clearance
gap
cooldown
rearm
sample
themeflip
adapt
adaptstep
debug
```

## Configuration is saved

Every successful setting change and `theme` change is automatically saved in the ESP32's **NVS flash memory** using `Preferences`.

That means values survive:

- restart/reset,
- power loss,
- unplugging the ESP32.

`defaults` restores the firmware defaults and saves them.

`reset` is different: it only clears the learned timing history for the current run and does **not** erase your saved configuration.

## Starting the game

Use:

```text
start
```

This resets the timing model, arms autoplay, and queues one initial servo click to start/restart Chrome Dino.

If the game is already moving:

```text
arm
```

To stop and cancel pending automatic clicks:

```text
stop
```

## Important timing values

| Setting | Default | Meaning |
|---|---:|---|
| `travel` | 1550 ms | Sensor position → Dino travel time at the beginning |
| `actuator` | 160 ms | Servo command → physical key contact |
| `air` | 450 ms | Estimated Dino jump airtime |
| `landing` | 30 ms | Desired landing after the obstacle trailing edge |
| `clearance` | 90 ms | Leading-edge safety margin |
| `gap` | 120 ms | Clear time used to finish/merge an obstacle envelope |
| `mintravel` | 350 ms | Lower limit for automatic speed adaptation |
| `adaptstep` | 6% | Maximum travel-time reduction per obstacle |

The software-only version can estimate speed directly from screen frames. With one physical light sensor, the ESP32 instead uses recent short obstacle-envelope durations as a conservative speed proxy.

## FreeRTOS servo task

Obstacle sensing and serial input run independently from servo movement.

Planned jumps are placed in a queue with an execution timestamp. The dedicated FreeRTOS servo task waits for that timestamp, moves to the press angle, holds the key for the configured duration, and returns to the rest angle.

Stopping autoplay invalidates old automatic commands so they cannot fire after a later restart.

## First tuning steps

Start with:

```text
sensor
show
debug on
start
```

If jumps are consistently early or late, tune `travel` first.

If the servo takes longer than expected to physically hit the key, tune `actuator`.

If one cactus/group is incorrectly split into several detections, increase `gap` slightly.
