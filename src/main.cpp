#include <Arduino.h>
#include <ESP32Servo.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {

constexpr int SERVO_PIN = 18;
constexpr int LIGHT_SENSOR_PIN = 15;
constexpr size_t CLICK_QUEUE_LENGTH = 12;
constexpr size_t ENVELOPE_HISTORY_SIZE = 5;
constexpr size_t ADAPT_WARMUP_ENVELOPES = 3;

struct Config {
  int threshold = 200;
  int restAngle = 20;
  int clickAngle = 25;

  uint32_t servoHoldMs = 80;
  uint32_t actuatorDelayMs = 160;
  uint32_t sensorTravelMs = 1550;
  uint32_t minSensorTravelMs = 350;
  uint32_t jumpAirTimeMs = 450;
  uint32_t landingMarginMs = 30;
  uint32_t entryClearanceMs = 90;
  uint32_t envelopeFinalizeGapMs = 120;
  uint32_t cooldownMs = 70;
  uint32_t rearmBeforeLandingMs = 12;

  uint32_t themeFlipMs = 1500;
  uint32_t samplePeriodMs = 2;

  bool autoAdapt = true;
  bool autoTheme = true;
  float maxAdaptDropPct = 6.0f;
  bool debug = false;
};

struct ClickCommand {
  uint64_t executeAtMs;
  uint32_t generation;
  int clickAngle;
  int restAngle;
  uint32_t holdMs;
  bool manual;
};

Servo dinoServo;
Config config;
QueueHandle_t clickQueue = nullptr;
TaskHandle_t clickTaskHandle = nullptr;

volatile bool playing = false;
volatile uint32_t playGeneration = 1;

String serialLine;

int lastSensorValue = 0;
bool backgroundIsWhite = true;
bool envelopeActive = false;
uint64_t envelopeStartMs = 0;
uint64_t envelopeLastSeenMs = 0;
uint64_t oppositeSinceMs = 0;
uint64_t lastSensorSampleMs = 0;

uint64_t lastPlannedLandingMs = 0;
uint64_t lastScheduledCommandMs = 0;
bool hasPreviousPlan = false;

uint32_t effectiveTravelMs = config.sensorTravelMs;
uint32_t envelopeHistory[ENVELOPE_HISTORY_SIZE] = {};
size_t envelopeHistoryCount = 0;
size_t envelopeHistoryIndex = 0;
uint32_t baselineShortEnvelopeMs = 0;

uint32_t scheduledJumpCount = 0;
uint32_t executedClickCount = 0;

uint64_t nowMs() {
  return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
}

const char *boolText(bool value) {
  return value ? "on" : "off";
}

const char *colorText(bool isWhite) {
  return isWhite ? "white" : "black";
}

bool parseBool(const String &value, bool &out) {
  if (value == "on" || value == "true" || value == "1" || value == "yes") {
    out = true;
    return true;
  }
  if (value == "off" || value == "false" || value == "0" || value == "no") {
    out = false;
    return true;
  }
  return false;
}

void clearClickQueue() {
  if (!clickQueue) {
    return;
  }

  ClickCommand ignored;
  while (xQueueReceive(clickQueue, &ignored, 0) == pdTRUE) {
  }
}

void resetEnvelope() {
  envelopeActive = false;
  envelopeStartMs = 0;
  envelopeLastSeenMs = 0;
  oppositeSinceMs = 0;
}

void resetTimingModel() {
  effectiveTravelMs = config.sensorTravelMs;
  envelopeHistoryCount = 0;
  envelopeHistoryIndex = 0;
  baselineShortEnvelopeMs = 0;

  lastPlannedLandingMs = 0;
  lastScheduledCommandMs = 0;
  hasPreviousPlan = false;
  scheduledJumpCount = 0;

  resetEnvelope();
}

uint32_t rollingMinEnvelopeMs() {
  if (envelopeHistoryCount == 0) {
    return 0;
  }

  uint32_t result = envelopeHistory[0];
  for (size_t i = 1; i < envelopeHistoryCount; ++i) {
    if (envelopeHistory[i] < result) {
      result = envelopeHistory[i];
    }
  }
  return result;
}

void updateAdaptiveTravel(uint32_t envelopeDurationMs) {
  envelopeHistory[envelopeHistoryIndex] = envelopeDurationMs;
  envelopeHistoryIndex = (envelopeHistoryIndex + 1) % ENVELOPE_HISTORY_SIZE;
  if (envelopeHistoryCount < ENVELOPE_HISTORY_SIZE) {
    ++envelopeHistoryCount;
  }

  const uint32_t rollingMin = rollingMinEnvelopeMs();

  if (baselineShortEnvelopeMs == 0 &&
      envelopeHistoryCount >= ADAPT_WARMUP_ENVELOPES) {
    baselineShortEnvelopeMs = rollingMin;
    if (config.debug) {
      Serial.printf("[ADAPT] baseline short envelope = %lu ms\n",
                    static_cast<unsigned long>(baselineShortEnvelopeMs));
    }
  }

  if (!config.autoAdapt || baselineShortEnvelopeMs == 0 ||
      rollingMin >= baselineShortEnvelopeMs) {
    return;
  }

  uint32_t targetTravel = static_cast<uint32_t>(
      (static_cast<uint64_t>(config.sensorTravelMs) * rollingMin) /
      baselineShortEnvelopeMs);

  if (targetTravel < config.minSensorTravelMs) {
    targetTravel = config.minSensorTravelMs;
  }

  if (targetTravel >= effectiveTravelMs) {
    return;
  }

  uint32_t maxDrop = static_cast<uint32_t>(
      effectiveTravelMs * (config.maxAdaptDropPct / 100.0f));
  if (maxDrop < 1) {
    maxDrop = 1;
  }

  const uint32_t limitedTarget =
      effectiveTravelMs > maxDrop ? effectiveTravelMs - maxDrop
                                  : config.minSensorTravelMs;

  effectiveTravelMs =
      targetTravel > limitedTarget ? targetTravel : limitedTarget;

  if (effectiveTravelMs < config.minSensorTravelMs) {
    effectiveTravelMs = config.minSensorTravelMs;
  }

  if (config.debug) {
    Serial.printf("[ADAPT] envelope=%lu ms rolling-min=%lu ms travel=%lu ms\n",
                  static_cast<unsigned long>(envelopeDurationMs),
                  static_cast<unsigned long>(rollingMin),
                  static_cast<unsigned long>(effectiveTravelMs));
  }
}

bool enqueueClick(uint64_t executeAtMs, bool manual) {
  ClickCommand cmd{};
  cmd.executeAtMs = executeAtMs;
  cmd.generation = playGeneration;
  cmd.clickAngle = config.clickAngle;
  cmd.restAngle = config.restAngle;
  cmd.holdMs = config.servoHoldMs;
  cmd.manual = manual;

  BaseType_t result;
  if (manual) {
    result = xQueueSendToFront(clickQueue, &cmd, 0);
  } else {
    result = xQueueSend(clickQueue, &cmd, 0);
  }

  if (result != pdTRUE) {
    Serial.println("[WARN] Servo click queue is full; click was dropped.");
    return false;
  }

  return true;
}

void servoClickTask(void *parameter) {
  (void)parameter;

  for (;;) {
    ClickCommand cmd{};
    if (xQueueReceive(clickQueue, &cmd, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    bool cancelled = false;

    for (;;) {
      if (!cmd.manual &&
          (!playing || cmd.generation != playGeneration)) {
        cancelled = true;
        break;
      }

      const uint64_t now = nowMs();
      if (now >= cmd.executeAtMs) {
        break;
      }

      uint64_t remaining = cmd.executeAtMs - now;
      if (remaining > 5) {
        remaining = 5;
      }
      if (remaining < 1) {
        remaining = 1;
      }
      vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(remaining)));
    }

    if (cancelled) {
      continue;
    }

    const uint64_t actualMs = nowMs();
    const uint64_t taskLateMs =
        actualMs > cmd.executeAtMs ? actualMs - cmd.executeAtMs : 0;

    dinoServo.write(cmd.clickAngle);

    TickType_t holdTicks = pdMS_TO_TICKS(cmd.holdMs);
    if (holdTicks < 1) {
      holdTicks = 1;
    }
    vTaskDelay(holdTicks);

    dinoServo.write(cmd.restAngle);
    ++executedClickCount;

    Serial.printf("[CLICK] #%lu at %llu ms, task-late=%llu ms\n",
                  static_cast<unsigned long>(executedClickCount),
                  static_cast<unsigned long long>(actualMs),
                  static_cast<unsigned long long>(taskLateMs));
  }
}

void finalizeEnvelope(uint64_t now) {
  if (!envelopeActive || envelopeLastSeenMs < envelopeStartMs) {
    resetEnvelope();
    return;
  }

  uint32_t durationMs =
      static_cast<uint32_t>(envelopeLastSeenMs - envelopeStartMs);
  if (durationMs == 0) {
    durationMs = 1;
  }

  updateAdaptiveTravel(durationMs);

  const int64_t entryAtDino =
      static_cast<int64_t>(envelopeStartMs) + effectiveTravelMs;
  const int64_t exitAtDino =
      static_cast<int64_t>(envelopeLastSeenMs) + effectiveTravelMs;

  const int64_t exitAlignedContact =
      exitAtDino + static_cast<int64_t>(config.landingMarginMs) -
      static_cast<int64_t>(config.jumpAirTimeMs);

  const int64_t latestSafeContact =
      entryAtDino - static_cast<int64_t>(config.entryClearanceMs);

  int64_t desiredContact =
      exitAlignedContact < latestSafeContact ? exitAlignedContact
                                             : latestSafeContact;

  const bool usedEntrySafety = latestSafeContact < exitAlignedContact;

  int64_t commandAt =
      desiredContact - static_cast<int64_t>(config.actuatorDelayMs);

  bool delayedForPreviousJump = false;
  bool delayedForCooldown = false;

  if (hasPreviousPlan) {
    const int64_t earliestForRejump =
        static_cast<int64_t>(lastPlannedLandingMs) +
        static_cast<int64_t>(config.rearmBeforeLandingMs) -
        static_cast<int64_t>(config.actuatorDelayMs);

    if (commandAt < earliestForRejump) {
      commandAt = earliestForRejump;
      delayedForPreviousJump = true;
    }

    const int64_t earliestForCooldown =
        static_cast<int64_t>(lastScheduledCommandMs) +
        static_cast<int64_t>(config.cooldownMs);

    if (commandAt < earliestForCooldown) {
      commandAt = earliestForCooldown;
      delayedForCooldown = true;
    }
  }

  uint64_t lateByMs = 0;
  if (commandAt < static_cast<int64_t>(now)) {
    lateByMs =
        static_cast<uint64_t>(static_cast<int64_t>(now) - commandAt);
    commandAt = static_cast<int64_t>(now);
  }

  const uint64_t executeAtMs = static_cast<uint64_t>(commandAt);

  if (enqueueClick(executeAtMs, false)) {
    ++scheduledJumpCount;

    lastScheduledCommandMs = executeAtMs;
    lastPlannedLandingMs =
        executeAtMs + config.actuatorDelayMs + config.jumpAirTimeMs;
    hasPreviousPlan = true;

    const int64_t commandInMs =
        static_cast<int64_t>(executeAtMs) - static_cast<int64_t>(now);

    const char *reason = "exit-aligned";
    if (lateByMs > 0) {
      reason = "sensor-too-close";
    } else if (delayedForPreviousJump) {
      reason = "post-landing-rejump";
    } else if (delayedForCooldown) {
      reason = "cooldown";
    } else if (usedEntrySafety) {
      reason = "entry-safety";
    }

    Serial.printf(
        "[PLAN] jump=%lu envelope=%lu ms travel=%lu ms command-in=%lld ms "
        "late=%llu ms reason=%s\n",
        static_cast<unsigned long>(scheduledJumpCount),
        static_cast<unsigned long>(durationMs),
        static_cast<unsigned long>(effectiveTravelMs),
        static_cast<long long>(commandInMs),
        static_cast<unsigned long long>(lateByMs),
        reason);
  }

  resetEnvelope();
}

void processSensor() {
  const uint64_t now = nowMs();

  if (now - lastSensorSampleMs < config.samplePeriodMs) {
    return;
  }
  lastSensorSampleMs = now;

  lastSensorValue = analogRead(LIGHT_SENSOR_PIN);

  if (!playing) {
    return;
  }

  const bool isWhite = lastSensorValue > config.threshold;
  bool obstacle = isWhite != backgroundIsWhite;

  if (config.autoTheme) {
    if (obstacle) {
      if (oppositeSinceMs == 0) {
        oppositeSinceMs = now;
      }

      if (now - oppositeSinceMs >= config.themeFlipMs) {
        backgroundIsWhite = isWhite;
        resetEnvelope();

        Serial.printf("[THEME] Background changed to %s; detector rebased.\n",
                      colorText(backgroundIsWhite));
        return;
      }
    } else {
      oppositeSinceMs = 0;
    }
  }

  obstacle = isWhite != backgroundIsWhite;

  if (obstacle) {
    if (!envelopeActive) {
      envelopeActive = true;
      envelopeStartMs = now;
      if (config.debug) {
        Serial.printf("[SENSOR] obstacle started, value=%d (%s)\n",
                      lastSensorValue, colorText(isWhite));
      }
    }

    envelopeLastSeenMs = now;
    return;
  }

  if (envelopeActive &&
      now - envelopeLastSeenMs >= config.envelopeFinalizeGapMs) {
    finalizeEnvelope(now);
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Dino ESP32 Auto-Player commands:");
  Serial.println("  start                 Start autoplay + click once to start/restart Dino");
  Serial.println("  arm                   Start autoplay without an initial click");
  Serial.println("  stop                  Stop autoplay and cancel pending automatic clicks");
  Serial.println("  click                 Manual immediate servo click");
  Serial.println("  status                Show current state and all tuning parameters");
  Serial.println("  sensor                Read the light sensor once");
  Serial.println("  theme auto|light|dark Select automatic or fixed background polarity");
  Serial.println("  reset                 Reset learned envelope/speed timing");
  Serial.println("  defaults              Restore default parameters");
  Serial.println();
  Serial.println("Tuning: set <name> <value>");
  Serial.println("  threshold              ADC split between black/white (default 200)");
  Serial.println("  rest                   Servo rest angle (default 20)");
  Serial.println("  click_angle            Servo key-down angle (default 25)");
  Serial.println("  hold_ms                How long key stays pressed (default 80)");
  Serial.println("  actuator_ms            Servo command -> physical key contact (default 160)");
  Serial.println("  travel_ms              Sensor -> Dino travel time at run start (default 1550)");
  Serial.println("  min_travel_ms          Lower bound for auto-adaptation (default 350)");
  Serial.println("  air_ms                 Dino jump airtime (default 450)");
  Serial.println("  landing_ms             Desired landing after obstacle exit (default 30)");
  Serial.println("  clearance_ms           Min entry clearance before cactus (default 90)");
  Serial.println("  gap_ms                 Merge/finalize clear gap (default 120)");
  Serial.println("  cooldown_ms            Minimum gap between servo commands (default 70)");
  Serial.println("  rearm_ms               Re-arm before previous landing (default 12)");
  Serial.println("  theme_flip_ms          Sustained polarity change before day/night rebase");
  Serial.println("  sample_ms              Sensor sample period (default 2)");
  Serial.println("  adapt                  on/off envelope-based speed adaptation");
  Serial.println("  adapt_step              Max travel-time drop per obstacle, percent");
  Serial.println("  auto_theme             on/off automatic day/night polarity detection");
  Serial.println("  debug                   on/off extra sensor/adaptation logging");
  Serial.println();
}

void printStatus() {
  const int sensor = analogRead(LIGHT_SENSOR_PIN);
  const bool isWhite = sensor > config.threshold;

  Serial.println();
  Serial.println("=== Dino ESP32 Auto-Player ===");
  Serial.printf("playing: %s\n", playing ? "YES" : "NO");
  Serial.printf("sensor: %d (%s), threshold=%d, GPIO=%d\n",
                sensor, colorText(isWhite), config.threshold, LIGHT_SENSOR_PIN);
  Serial.printf("background: %s, auto-theme=%s\n",
                colorText(backgroundIsWhite), boolText(config.autoTheme));
  Serial.printf("servo: GPIO=%d rest=%d click=%d hold=%lu ms\n",
                SERVO_PIN, config.restAngle, config.clickAngle,
                static_cast<unsigned long>(config.servoHoldMs));
  Serial.printf(
      "timing: actuator=%lu travel-base=%lu travel-effective=%lu "
      "min-travel=%lu air=%lu landing=%lu clearance=%lu\n",
      static_cast<unsigned long>(config.actuatorDelayMs),
      static_cast<unsigned long>(config.sensorTravelMs),
      static_cast<unsigned long>(effectiveTravelMs),
      static_cast<unsigned long>(config.minSensorTravelMs),
      static_cast<unsigned long>(config.jumpAirTimeMs),
      static_cast<unsigned long>(config.landingMarginMs),
      static_cast<unsigned long>(config.entryClearanceMs));
  Serial.printf("envelope: gap=%lu cooldown=%lu rearm=%lu sample=%lu\n",
                static_cast<unsigned long>(config.envelopeFinalizeGapMs),
                static_cast<unsigned long>(config.cooldownMs),
                static_cast<unsigned long>(config.rearmBeforeLandingMs),
                static_cast<unsigned long>(config.samplePeriodMs));
  Serial.printf("adapt: %s step=%.1f%% baseline-envelope=%lu ms rolling-min=%lu ms\n",
                boolText(config.autoAdapt), config.maxAdaptDropPct,
                static_cast<unsigned long>(baselineShortEnvelopeMs),
                static_cast<unsigned long>(rollingMinEnvelopeMs()));
  Serial.printf("theme-flip=%lu ms debug=%s queued=%u\n",
                static_cast<unsigned long>(config.themeFlipMs),
                boolText(config.debug),
                static_cast<unsigned>(uxQueueMessagesWaiting(clickQueue)));
  Serial.println();
}

void startAutoplay(bool clickToStart) {
  ++playGeneration;
  clearClickQueue();
  resetTimingModel();

  lastSensorValue = analogRead(LIGHT_SENSOR_PIN);
  backgroundIsWhite = lastSensorValue > config.threshold;
  oppositeSinceMs = 0;

  playing = true;

  Serial.printf("[START] Autoplay armed. Background=%s sensor=%d.\n",
                colorText(backgroundIsWhite), lastSensorValue);

  if (clickToStart) {
    enqueueClick(nowMs(), true);
    Serial.println("[START] Initial click queued.");
  }
}

void stopAutoplay() {
  playing = false;
  ++playGeneration;
  clearClickQueue();
  resetEnvelope();
  dinoServo.write(config.restAngle);
  Serial.println("[STOP] Autoplay stopped; pending automatic clicks cancelled.");
}

bool setUIntParameter(const String &name, const String &value) {
  const long parsed = value.toInt();

  if (name == "threshold") {
    if (parsed < 0 || parsed > 4095) return false;
    config.threshold = static_cast<int>(parsed);
  } else if (name == "rest") {
    if (parsed < 0 || parsed > 180) return false;
    config.restAngle = static_cast<int>(parsed);
    if (!playing) dinoServo.write(config.restAngle);
  } else if (name == "click_angle") {
    if (parsed < 0 || parsed > 180) return false;
    config.clickAngle = static_cast<int>(parsed);
  } else if (name == "hold_ms") {
    if (parsed < 1 || parsed > 2000) return false;
    config.servoHoldMs = static_cast<uint32_t>(parsed);
  } else if (name == "actuator_ms") {
    if (parsed < 0 || parsed > 5000) return false;
    config.actuatorDelayMs = static_cast<uint32_t>(parsed);
  } else if (name == "travel_ms") {
    if (parsed < 1 || parsed > 10000) return false;
    config.sensorTravelMs = static_cast<uint32_t>(parsed);
    resetTimingModel();
  } else if (name == "min_travel_ms") {
    if (parsed < 1 || parsed > 10000) return false;
    config.minSensorTravelMs = static_cast<uint32_t>(parsed);
    if (effectiveTravelMs < config.minSensorTravelMs) {
      effectiveTravelMs = config.minSensorTravelMs;
    }
  } else if (name == "air_ms") {
    if (parsed < 1 || parsed > 3000) return false;
    config.jumpAirTimeMs = static_cast<uint32_t>(parsed);
  } else if (name == "landing_ms") {
    if (parsed < 0 || parsed > 2000) return false;
    config.landingMarginMs = static_cast<uint32_t>(parsed);
  } else if (name == "clearance_ms") {
    if (parsed < 0 || parsed > 3000) return false;
    config.entryClearanceMs = static_cast<uint32_t>(parsed);
  } else if (name == "gap_ms") {
    if (parsed < 1 || parsed > 3000) return false;
    config.envelopeFinalizeGapMs = static_cast<uint32_t>(parsed);
  } else if (name == "cooldown_ms") {
    if (parsed < 0 || parsed > 3000) return false;
    config.cooldownMs = static_cast<uint32_t>(parsed);
  } else if (name == "rearm_ms") {
    if (parsed < 0 || parsed > 3000) return false;
    config.rearmBeforeLandingMs = static_cast<uint32_t>(parsed);
  } else if (name == "theme_flip_ms") {
    if (parsed < 100 || parsed > 10000) return false;
    config.themeFlipMs = static_cast<uint32_t>(parsed);
  } else if (name == "sample_ms") {
    if (parsed < 1 || parsed > 1000) return false;
    config.samplePeriodMs = static_cast<uint32_t>(parsed);
  } else {
    return false;
  }

  return true;
}

void setParameter(const String &name, const String &value) {
  bool boolValue = false;

  if (name == "adapt") {
    if (!parseBool(value, boolValue)) {
      Serial.println("[ERR] adapt expects on/off.");
      return;
    }
    config.autoAdapt = boolValue;
    if (!config.autoAdapt) {
      effectiveTravelMs = config.sensorTravelMs;
    }
  } else if (name == "auto_theme") {
    if (!parseBool(value, boolValue)) {
      Serial.println("[ERR] auto_theme expects on/off.");
      return;
    }
    config.autoTheme = boolValue;
  } else if (name == "debug") {
    if (!parseBool(value, boolValue)) {
      Serial.println("[ERR] debug expects on/off.");
      return;
    }
    config.debug = boolValue;
  } else if (name == "adapt_step") {
    const float parsed = value.toFloat();
    if (parsed <= 0.0f || parsed > 50.0f) {
      Serial.println("[ERR] adapt_step must be >0 and <=50 percent.");
      return;
    }
    config.maxAdaptDropPct = parsed;
  } else if (!setUIntParameter(name, value)) {
    Serial.println("[ERR] Unknown parameter or value outside allowed range.");
    return;
  }

  Serial.printf("[SET] %s = %s\n", name.c_str(), value.c_str());
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();

  if (line.length() == 0) {
    return;
  }

  if (line == "help" || line == "?") {
    printHelp();
    return;
  }

  if (line == "status") {
    printStatus();
    return;
  }

  if (line == "sensor") {
    const int value = analogRead(LIGHT_SENSOR_PIN);
    Serial.printf("[SENSOR] %d -> %s (threshold=%d)\n",
                  value, colorText(value > config.threshold), config.threshold);
    return;
  }

  if (line == "start") {
    startAutoplay(true);
    return;
  }

  if (line == "arm") {
    startAutoplay(false);
    return;
  }

  if (line == "stop" || line == "pause") {
    stopAutoplay();
    return;
  }

  if (line == "click" || line == "jump") {
    if (enqueueClick(nowMs(), true)) {
      Serial.println("[MANUAL] Click queued.");
    }
    return;
  }

  if (line == "reset") {
    resetTimingModel();
    Serial.println("[RESET] Learned timing/envelope history cleared.");
    return;
  }

  if (line == "defaults") {
    config = Config{};
    resetTimingModel();
    if (!playing) {
      dinoServo.write(config.restAngle);
    }
    Serial.println("[DEFAULTS] Parameters restored.");
    printStatus();
    return;
  }

  if (line.startsWith("theme ")) {
    const String mode = line.substring(6);
    if (mode == "auto") {
      config.autoTheme = true;
      const int value = analogRead(LIGHT_SENSOR_PIN);
      backgroundIsWhite = value > config.threshold;
      resetEnvelope();
      Serial.printf("[THEME] auto; current background=%s\n",
                    colorText(backgroundIsWhite));
    } else if (mode == "light") {
      config.autoTheme = false;
      backgroundIsWhite = true;
      resetEnvelope();
      Serial.println("[THEME] fixed light theme (white background).");
    } else if (mode == "dark") {
      config.autoTheme = false;
      backgroundIsWhite = false;
      resetEnvelope();
      Serial.println("[THEME] fixed dark theme (black background).");
    } else {
      Serial.println("[ERR] Use: theme auto | theme light | theme dark");
    }
    return;
  }

  if (line.startsWith("set ")) {
    const int separator = line.indexOf(' ', 4);
    if (separator < 0) {
      Serial.println("[ERR] Use: set <name> <value>");
      return;
    }

    String name = line.substring(4, separator);
    String value = line.substring(separator + 1);
    name.trim();
    value.trim();

    if (name.length() == 0 || value.length() == 0) {
      Serial.println("[ERR] Use: set <name> <value>");
      return;
    }

    setParameter(name, value);
    return;
  }

  Serial.println("[ERR] Unknown command. Type 'help'.");
}

void processSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        handleCommand(serialLine);
        serialLine = "";
      }
      continue;
    }

    if (serialLine.length() < 120) {
      serialLine += c;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(250);

  pinMode(LIGHT_SENSOR_PIN, INPUT);
  analogReadResolution(12);

  dinoServo.setPeriodHertz(50);
  dinoServo.attach(SERVO_PIN, 500, 2400);
  dinoServo.write(config.restAngle);

  clickQueue = xQueueCreate(CLICK_QUEUE_LENGTH, sizeof(ClickCommand));
  if (!clickQueue) {
    Serial.println("[FATAL] Could not create servo click queue.");
    while (true) {
      delay(1000);
    }
  }

  const BaseType_t taskCreated = xTaskCreatePinnedToCore(
      servoClickTask,
      "dino-servo",
      4096,
      nullptr,
      4,
      &clickTaskHandle,
      1);

  if (taskCreated != pdPASS) {
    Serial.println("[FATAL] Could not create servo FreeRTOS task.");
    while (true) {
      delay(1000);
    }
  }

  lastSensorValue = analogRead(LIGHT_SENSOR_PIN);
  backgroundIsWhite = lastSensorValue > config.threshold;

  Serial.println();
  Serial.println("Dino ESP32 Auto-Player ready.");
  Serial.printf("Servo GPIO %d | Light sensor GPIO %d | threshold %d\n",
                SERVO_PIN, LIGHT_SENSOR_PIN, config.threshold);
  Serial.println("Type 'help' for commands. Type 'start' to play.");
  printStatus();
}

void loop() {
  processSerial();
  processSensor();
  delay(1);
}
