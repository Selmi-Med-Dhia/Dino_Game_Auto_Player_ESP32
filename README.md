# Dino Game Auto Player — ESP32

ESP32 hardware auto-player for Chrome Dino using one analog light sensor and one servo.

- **Servo:** GPIO 18
- **Light sensor:** analog GPIO 15
- **White:** ADC > 200
- **Black:** ADC < 200
- **Servo rest / press:** 35° / 38°
- **Serial:** 115200 baud

The runtime is FreeRTOS-task based. `loop()` does no application work.

## What changed: short and long jumps

Chrome Dino has a variable jump: releasing Space early shortens the jump, while holding it longer lets the Dino continue to the full jump arc.

The firmware now has two profiles:

```text
SHORT: hold 80 ms   modeled airtime 450 ms
LONG:  hold 160 ms  modeled airtime 520 ms
```

The existing short-jump settings are preserved. A long jump is used automatically for wide cactus groups.

You can test both mechanically:

```text
click
longclick
```

## How wide groups are detected

Your optical sensor footprint is approximately **2× the width of a normal cactus**. A raw dark pulse therefore contains both the cactus width and the sensor footprint.

The firmware keeps a recent pulse history and estimates a normal single-cactus pulse. From that it separates:

```text
raw pulse ≈ fixed sensor footprint + obstacle/group width
```

This is better than applying the old one-third edge correction to every obstacle: the optical footprint stays the same when the cactus group becomes wider.

The estimated group width is reported in approximate **normal-cactus widths**. Default:

```text
ratio 2.0
longat 1.60
```

So a detected group around 2 cactus widths receives the long profile, while a normal single cactus remains on the short profile. During the first few detections after `start`/`reset`, the firmware conservatively uses long jumps while it learns the single-cactus pulse reference.

Wide groups are not fed into the speed estimator, preventing a triple cactus from being mistaken for a sudden slowdown.

## FreeRTOS tasks

| Task | Core | Priority | Job |
|---|---:|---:|---|
| Servo | 1 | 5 | Executes scheduled key presses |
| Sensor / planner | 0 | 4 | 5 ms ADC sampling, filtering, detection, planning |
| Serial | 0 | 2 | Commands, configuration, queued runtime logs |

The sensor and servo tasks do not print directly to Serial. Runtime messages go through a non-blocking log queue so UART output cannot stall obstacle detection.

## Serial Monitor guide

### PlatformIO / VS Code

1. Connect the ESP32 by USB.
2. Open the project in VS Code with PlatformIO.
3. Upload the firmware.
4. Open **PlatformIO → Project Tasks → upesy_wroom → Monitor**.
5. The baud rate is already **115200** in `platformio.ini`.
6. Click the monitor terminal, type a command, and press **Enter**.

### Arduino IDE

Open **Tools → Serial Monitor**, set **115200 baud**, and use **New Line** or **Both NL & CR**.

The ESP32 prints the compact manual again after every command.

## First test

Check the sensor:

```text
sensor
```

Target:

```text
white > 200
black < 200
```

If needed:

```text
threshold 200
```

Check both servo actions:

```text
click
longclick
```

Then show the saved configuration:

```text
show
```

Start/restart Dino:

```text
start
```

If Dino is already running:

```text
arm
```

Stop:

```text
stop
```

## Commands

```text
start | arm | stop | click | longclick | show | sensor | reset | defaults
theme auto|light|dark
<name> <value>  (auto-saved)
threshold rest press hold longhold air longair longat
actuator travel mintravel landing clearance gap sample cooldown rearm
ratio adapt adaptstep debug
```

Old `set ...` syntax still works.

## Recommended starting configuration

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
landing 50
clearance 100

gap 40
sample 5
ratio 2.0

adapt on
adaptstep 3
mintravel 350
```

All changed settings are stored immediately in ESP32 NVS flash.

## Debugging the two jump types

Enable:

```text
debug on
start
```

Typical output:

```text
[PLAN] #8 SHORT width=1.06 pulse=132 hold=80 air=450 cmd-in=281 late=0 exit-aligned
[PLAN] #9 LONG width=2.14 pulse=177 hold=160 air=520 cmd-in=196 late=0 exit-aligned
```

Important fields:

- `SHORT` / `LONG` — selected jump profile.
- `width` — estimated obstacle width in normal-cactus widths.
- `pulse` — measured optical pulse duration.
- `hold` — servo key-down command duration.
- `air` — planner's expected jump airtime.
- `late` — how late the planner was when the full obstacle envelope became known.

## Long-jump tuning

The defaults are intended as a safe starting point.

### A wide group still gets SHORT

Lower the width threshold slightly:

```text
longat 1.55
```

Do not make large jumps in this setting. `1.50–1.75` is the useful tuning area for the stated sensor geometry.

### Too many single cacti get LONG

Raise it:

```text
longat 1.70
```

### Dino starts the long jump but still lands too soon

First increase the physical hold a little:

```text
longhold 170
```

or:

```text
longhold 180
```

Chrome Dino reaches a maximum jump arc, so excessively large hold values do not keep increasing the useful jump indefinitely.

If the physical long jump is correct but the planner predicts its landing too early/late, tune the model separately:

```text
longair 530
```

Keep `hold` / `air` for the already-working short jump and `longhold` / `longair` for wide groups.

## General timing calibration

- **All jumps too late:** decrease `travel` by about 20–40 ms per test.
- **All jumps too early:** increase `travel` similarly.
- **Servo reaches the key later than expected:** increase `actuator`.
- **Dino lands on the trailing edge:** increase `landing` by 10–20 ms.
- **One obstacle is split into multiple detections:** slightly increase `gap`.
- **Different obstacles are merged:** decrease `gap`.
- **Repeated `late > 0`:** the sensor is physically too close or timing latency is too large.

After moving the sensor or changing major timing values:

```text
reset
```

This clears learned pulse/speed history but keeps saved configuration.

## Sensor filtering and speed adaptation

The sensor task runs every **5 ms (200 Hz)** and uses:

- a rolling 3-sample median;
- ±8 ADC hysteresis around the threshold;
- a 40 ms envelope-finalization gap.

Two histories are kept separately:

- a longer pulse history estimates normal cactus width for SHORT/LONG classification;
- a short-only history estimates speed, so wide groups cannot corrupt acceleration tracking.

## Persistent configuration migration

Firmware v3 preserves the existing v1/v2 configuration layout when upgrading. Your already-calibrated threshold, servo angles, short hold, travel timing, and other settings are copied into the new configuration. Only the new long-jump fields receive their defaults.

- Power loss/reset keeps settings.
- `reset` keeps settings and clears learned runtime history only.
- `defaults` restores and saves firmware defaults.
- A full flash/NVS erase clears saved settings.

## Hardware notes

- GPIO 15 is an ESP32 ADC2/strapping pin. This project does not use Wi-Fi, but make sure the sensor circuit does not force an invalid boot strap level during reset.
- Power anything larger than a tiny servo from a suitable external supply and connect grounds together.
