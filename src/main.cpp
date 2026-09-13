#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {

constexpr int SERVO_PIN = 18;
constexpr int SENSOR_PIN = 15;
constexpr size_t CLICK_QUEUE_SIZE = 12;
constexpr size_t HISTORY_SIZE = 5;
constexpr size_t ADAPT_WARMUP = 3;
constexpr uint32_t CONFIG_MAGIC = 0xD1A02026;
constexpr uint16_t CONFIG_VERSION = 1;

enum class ThemeMode : uint8_t { Auto, Light, Dark };

struct Config {
  uint32_t magic = CONFIG_MAGIC;
  uint16_t version = CONFIG_VERSION;

  int threshold = 200;
  int restAngle = 35;
  int pressAngle = 38;

  uint32_t holdMs = 80;
  uint32_t actuatorMs = 160;
  uint32_t travelMs = 1550;
  uint32_t minTravelMs = 350;
  uint32_t airMs = 450;
  uint32_t landingMs = 30;
  uint32_t clearanceMs = 90;
  uint32_t gapMs = 120;
  uint32_t cooldownMs = 70;
  uint32_t rearmMs = 12;
  uint32_t themeFlipMs = 1500;
  uint32_t sampleMs = 2;

  bool adapt = true;
  float adaptStepPct = 6.0f;
  ThemeMode theme = ThemeMode::Auto;
  bool debug = false;
};

struct ClickCommand {
  uint64_t atMs;
  uint32_t generation;
  int pressAngle;
  int restAngle;
  uint32_t holdMs;
  bool manual;
};

struct DetectorState {
  bool active = false;
  uint64_t startMs = 0;
  uint64_t lastObstacleMs = 0;
  uint64_t oppositeSinceMs = 0;
  uint64_t lastSampleMs = 0;

  uint64_t lastLandingMs = 0;
  uint64_t lastCommandMs = 0;
  bool hasPlan = false;

  uint32_t effectiveTravelMs = 0;
  uint32_t history[HISTORY_SIZE] = {};
  size_t historyCount = 0;
  size_t historyIndex = 0;
  uint32_t baselineEnvelopeMs = 0;
  uint32_t plannedJumps = 0;
};

struct UIntSetting {
  const char *name;
  const char *legacyName;
  uint32_t Config::*field;
  uint32_t minValue;
  uint32_t maxValue;
};

constexpr UIntSetting UINT_SETTINGS[] = {
    {"hold", "holdms", &Config::holdMs, 1, 2000},
    {"actuator", "actuatorms", &Config::actuatorMs, 0, 5000},
    {"travel", "travelms", &Config::travelMs, 1, 10000},
    {"mintravel", "mintravelms", &Config::minTravelMs, 1, 10000},
    {"air", "airms", &Config::airMs, 1, 3000},
    {"landing", "landingms", &Config::landingMs, 0, 2000},
    {"clearance", "clearancems", &Config::clearanceMs, 0, 3000},
    {"gap", "gapms", &Config::gapMs, 1, 3000},
    {"cooldown", "cooldownms", &Config::cooldownMs, 0, 3000},
    {"rearm", "rearmms", &Config::rearmMs, 0, 3000},
    {"sample", "samplems", &Config::sampleMs, 1, 1000},
    {"themeflip", "themeflipms", &Config::themeFlipMs, 100, 10000},
};

Servo servo;
Preferences prefs;
Config config;
DetectorState detector;

QueueHandle_t clickQueue = nullptr;
TaskHandle_t clickTaskHandle = nullptr;

volatile bool playing = false;
volatile uint32_t playGeneration = 1;

String serialLine;
bool backgroundIsWhite = true;
int lastSensorValue = 0;
uint32_t executedClicks = 0;
bool prefsReady = false;

// ---------- Helpers ----------

uint64_t nowMs() {
  return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
}

const char *onOff(bool value) { return value ? "on" : "off"; }
const char *colorName(bool white) { return white ? "white" : "black"; }

const char *themeName(ThemeMode mode) {
  if (mode == ThemeMode::Light) return "light";
  if (mode == ThemeMode::Dark) return "dark";
  return "auto";
}

bool sensorIsWhite(int value) { return value > config.threshold; }

String normalize(String text) {
  text.trim();
  text.toLowerCase();
  text.replace("_", "");
  text.replace("-", "");
  return text;
}

bool parseLong(const String &text, long &value) {
  char *end = nullptr;
  value = strtol(text.c_str(), &end, 10);
  while (end && *end == ' ') ++end;
  return end && end != text.c_str() && *end == '\0';
}

bool parseFloat(const String &text, float &value) {
  char *end = nullptr;
  value = strtof(text.c_str(), &end);
  while (end && *end == ' ') ++end;
  return end && end != text.c_str() && *end == '\0';
}

bool parseBool(String text, bool &value) {
  text = normalize(text);
  if (text == "on" || text == "true" || text == "1" || text == "yes") {
    value = true;
    return true;
  }
  if (text == "off" || text == "false" || text == "0" || text == "no") {
    value = false;
    return true;
  }
  return false;
}

// ---------- Configuration in flash (ESP32 NVS) ----------

bool configValid() {
  return config.magic == CONFIG_MAGIC &&
         config.version == CONFIG_VERSION &&
         config.threshold >= 0 && config.threshold <= 4095 &&
         config.restAngle >= 0 && config.restAngle <= 180 &&
         config.pressAngle >= 0 && config.pressAngle <= 180 &&
         config.holdMs >= 1 && config.holdMs <= 2000 &&
         config.actuatorMs <= 5000 &&
         config.travelMs >= 1 && config.travelMs <= 10000 &&
         config.minTravelMs >= 1 && config.minTravelMs <= 10000 &&
         config.airMs >= 1 && config.airMs <= 3000 &&
         config.landingMs <= 2000 &&
         config.clearanceMs <= 3000 &&
         config.gapMs >= 1 && config.gapMs <= 3000 &&
         config.cooldownMs <= 3000 &&
         config.rearmMs <= 3000 &&
         config.themeFlipMs >= 100 && config.themeFlipMs <= 10000 &&
         config.sampleMs >= 1 && config.sampleMs <= 1000 &&
         config.adaptStepPct > 0.0f && config.adaptStepPct <= 50.0f &&
         static_cast<uint8_t>(config.theme) <= 2;
}

bool saveConfig(bool announce = true) {
  if (!prefsReady) {
    if (announce) Serial.println("[SAVE] Flash storage unavailable.");
    return false;
  }

  const bool ok =
      prefs.putBytes("config", &config, sizeof(config)) == sizeof(config);
  if (announce) {
    Serial.println(ok ? "[SAVE] Configuration saved."
                      : "[SAVE] Could not save configuration.");
  }
  return ok;
}

void loadConfig() {
  prefsReady = prefs.begin("dino", false);
  if (!prefsReady) {
    Serial.println("[NVS] Flash storage unavailable; using defaults.");
    return;
  }

  const bool hasSavedConfig =
      prefs.getBytesLength("config") == sizeof(config);

  if (hasSavedConfig) {
    prefs.getBytes("config", &config, sizeof(config));
  }

  if (!hasSavedConfig || !configValid()) {
    config = Config{};
    saveConfig(false);
    Serial.println("[NVS] Defaults loaded and saved.");
  } else {
    Serial.println("[NVS] Saved configuration loaded.");
  }
}

// ---------- Timing model ----------

void resetEnvelope() {
  detector.active = false;
  detector.startMs = 0;
  detector.lastObstacleMs = 0;
  detector.oppositeSinceMs = 0;
}

void resetTiming() {
  detector = DetectorState{};
  detector.effectiveTravelMs = config.travelMs;
}

uint32_t shortestEnvelope() {
  if (!detector.historyCount) return 0;

  uint32_t shortest = detector.history[0];
  for (size_t i = 1; i < detector.historyCount; ++i) {
    if (detector.history[i] < shortest) shortest = detector.history[i];
  }
  return shortest;
}

void updateTravel(uint32_t envelopeMs) {
  detector.history[detector.historyIndex] = envelopeMs;
  detector.historyIndex = (detector.historyIndex + 1) % HISTORY_SIZE;
  if (detector.historyCount < HISTORY_SIZE) ++detector.historyCount;

  const uint32_t shortest = shortestEnvelope();

  if (!detector.baselineEnvelopeMs &&
      detector.historyCount >= ADAPT_WARMUP) {
    detector.baselineEnvelopeMs = shortest;
  }

  if (!config.adapt || !detector.baselineEnvelopeMs ||
      shortest >= detector.baselineEnvelopeMs) {
    return;
  }

  uint32_t target = static_cast<uint32_t>(
      static_cast<uint64_t>(config.travelMs) * shortest /
      detector.baselineEnvelopeMs);
  if (target < config.minTravelMs) target = config.minTravelMs;
  if (target >= detector.effectiveTravelMs) return;

  uint32_t maxDrop = static_cast<uint32_t>(
      detector.effectiveTravelMs * config.adaptStepPct / 100.0f);
  if (!maxDrop) maxDrop = 1;

  uint32_t limited =
      detector.effectiveTravelMs > maxDrop
          ? detector.effectiveTravelMs - maxDrop
          : config.minTravelMs;

  detector.effectiveTravelMs = target > limited ? target : limited;
  if (detector.effectiveTravelMs < config.minTravelMs) {
    detector.effectiveTravelMs = config.minTravelMs;
  }

  if (config.debug) {
    Serial.printf("[ADAPT] envelope=%lu shortest=%lu travel=%lu\n",
                  static_cast<unsigned long>(envelopeMs),
                  static_cast<unsigned long>(shortest),
                  static_cast<unsigned long>(detector.effectiveTravelMs));
  }
}

// ---------- Servo FreeRTOS task ----------

void clearClickQueue() {
  if (!clickQueue) return;
  ClickCommand ignored{};
  while (xQueueReceive(clickQueue, &ignored, 0) == pdTRUE) {}
}

bool queueClick(uint64_t atMs, bool manual) {
  ClickCommand command{
      atMs, playGeneration, config.pressAngle,
      config.restAngle, config.holdMs, manual};

  BaseType_t result =
      manual ? xQueueSendToFront(clickQueue, &command, 0)
             : xQueueSend(clickQueue, &command, 0);

  if (result != pdTRUE) {
    Serial.println("[WARN] Servo queue full; click dropped.");
    return false;
  }
  return true;
}

void servoTask(void *) {
  for (;;) {
    ClickCommand command{};
    if (xQueueReceive(clickQueue, &command, portMAX_DELAY) != pdTRUE) continue;

    bool cancelled = false;
    for (;;) {
      if (!command.manual &&
          (!playing || command.generation != playGeneration)) {
        cancelled = true;
        break;
      }

      const uint64_t now = nowMs();
      if (now >= command.atMs) break;

      const uint64_t remaining = command.atMs - now;
      uint32_t sleepMs = remaining > 5 ? 5 : static_cast<uint32_t>(remaining);
      if (!sleepMs) sleepMs = 1;
      vTaskDelay(pdMS_TO_TICKS(sleepMs));
    }
    if (cancelled) continue;

    const uint64_t actual = nowMs();
    const uint64_t late = actual > command.atMs ? actual - command.atMs : 0;

    servo.write(command.pressAngle);
    TickType_t holdTicks = pdMS_TO_TICKS(command.holdMs);
    vTaskDelay(holdTicks ? holdTicks : 1);
    servo.write(command.restAngle);

    ++executedClicks;
    if (config.debug) {
      Serial.printf("[CLICK] #%lu task-late=%llu ms\n",
                    static_cast<unsigned long>(executedClicks),
                    static_cast<unsigned long long>(late));
    }
  }
}

// ---------- Obstacle detection and jump planning ----------

void finalizeEnvelope(uint64_t now) {
  if (!detector.active || detector.lastObstacleMs < detector.startMs) {
    resetEnvelope();
    return;
  }

  uint32_t envelopeMs =
      static_cast<uint32_t>(detector.lastObstacleMs - detector.startMs);
  if (!envelopeMs) envelopeMs = 1;
  updateTravel(envelopeMs);

  const int64_t entry =
      static_cast<int64_t>(detector.startMs) + detector.effectiveTravelMs;
  const int64_t exit =
      static_cast<int64_t>(detector.lastObstacleMs) + detector.effectiveTravelMs;

  const int64_t exitContact = exit + config.landingMs - config.airMs;
  const int64_t safeContact = entry - config.clearanceMs;
  int64_t commandAt =
      (exitContact < safeContact ? exitContact : safeContact) -
      config.actuatorMs;

  const bool entrySafety = safeContact < exitContact;
  bool delayedForLanding = false;
  bool delayedForCooldown = false;

  if (detector.hasPlan) {
    const int64_t afterLanding =
        static_cast<int64_t>(detector.lastLandingMs) +
        config.rearmMs - config.actuatorMs;
    const int64_t afterCooldown =
        static_cast<int64_t>(detector.lastCommandMs) + config.cooldownMs;

    if (commandAt < afterLanding) {
      commandAt = afterLanding;
      delayedForLanding = true;
    }
    if (commandAt < afterCooldown) {
      commandAt = afterCooldown;
      delayedForCooldown = true;
    }
  }

  uint64_t lateMs = 0;
  if (commandAt < static_cast<int64_t>(now)) {
    lateMs = static_cast<uint64_t>(static_cast<int64_t>(now) - commandAt);
    commandAt = static_cast<int64_t>(now);
  }

  const char *reason = "exit-aligned";
  if (lateMs) reason = "sensor-too-close";
  else if (delayedForLanding) reason = "post-landing-rejump";
  else if (delayedForCooldown) reason = "cooldown";
  else if (entrySafety) reason = "entry-safety";

  const uint64_t executeAt = static_cast<uint64_t>(commandAt);
  if (queueClick(executeAt, false)) {
    ++detector.plannedJumps;
    detector.lastCommandMs = executeAt;
    detector.lastLandingMs = executeAt + config.actuatorMs + config.airMs;
    detector.hasPlan = true;

    Serial.printf(
        "[PLAN] #%lu envelope=%lu travel=%lu command-in=%lld late=%llu %s\n",
        static_cast<unsigned long>(detector.plannedJumps),
        static_cast<unsigned long>(envelopeMs),
        static_cast<unsigned long>(detector.effectiveTravelMs),
        static_cast<long long>(
            static_cast<int64_t>(executeAt) - static_cast<int64_t>(now)),
        static_cast<unsigned long long>(lateMs), reason);
  }

  resetEnvelope();
}

void chooseBackground() {
  lastSensorValue = analogRead(SENSOR_PIN);

  if (config.theme == ThemeMode::Light) backgroundIsWhite = true;
  else if (config.theme == ThemeMode::Dark) backgroundIsWhite = false;
  else backgroundIsWhite = sensorIsWhite(lastSensorValue);
}

void processSensor() {
  const uint64_t now = nowMs();
  if (now - detector.lastSampleMs < config.sampleMs) return;
  detector.lastSampleMs = now;

  lastSensorValue = analogRead(SENSOR_PIN);
  if (!playing) return;

  const bool white = sensorIsWhite(lastSensorValue);
  bool obstacle = white != backgroundIsWhite;

  if (config.theme == ThemeMode::Auto) {
    if (obstacle) {
      if (!detector.oppositeSinceMs) detector.oppositeSinceMs = now;

      if (now - detector.oppositeSinceMs >= config.themeFlipMs) {
        backgroundIsWhite = white;
        resetEnvelope();
        Serial.printf("[THEME] Background rebased to %s.\n",
                      colorName(backgroundIsWhite));
        return;
      }
    } else {
      detector.oppositeSinceMs = 0;
    }
  }

  obstacle = white != backgroundIsWhite;
  if (obstacle) {
    if (!detector.active) {
      detector.active = true;
      detector.startMs = now;
      if (config.debug) {
        Serial.printf("[SENSOR] obstacle start, ADC=%d\n", lastSensorValue);
      }
    }
    detector.lastObstacleMs = now;
  } else if (detector.active &&
             now - detector.lastObstacleMs >= config.gapMs) {
    finalizeEnvelope(now);
  }
}

// ---------- Serial console ----------

void printManual() {
  Serial.println();
  Serial.println("start | arm | stop | click | show | sensor | reset | defaults");
  Serial.println("theme auto|light|dark");
  Serial.println("<name> <value>  (auto-saved)");
  Serial.println("threshold rest press hold actuator travel mintravel air landing");
  Serial.println("clearance gap cooldown rearm sample themeflip adapt adaptstep debug");
  Serial.println("Example: press 38   travel 1550   adapt on");
  Serial.println();
}

void printStatus() {
  const int sensor = analogRead(SENSOR_PIN);

  Serial.println("\n=== Dino Auto-Player ===");
  Serial.printf("Run=%s | sensor=%d (%s) | threshold=%d | theme=%s\n",
                playing ? "ON" : "OFF", sensor,
                colorName(sensorIsWhite(sensor)),
                config.threshold, themeName(config.theme));
  Serial.printf("Servo: rest=%d press=%d hold=%lu ms\n",
                config.restAngle, config.pressAngle,
                static_cast<unsigned long>(config.holdMs));
  Serial.printf(
      "Timing: actuator=%lu travel=%lu effective=%lu air=%lu landing=%lu clearance=%lu\n",
      static_cast<unsigned long>(config.actuatorMs),
      static_cast<unsigned long>(config.travelMs),
      static_cast<unsigned long>(detector.effectiveTravelMs),
      static_cast<unsigned long>(config.airMs),
      static_cast<unsigned long>(config.landingMs),
      static_cast<unsigned long>(config.clearanceMs));
  Serial.printf(
      "Detector: gap=%lu cooldown=%lu rearm=%lu sample=%lu themeflip=%lu\n",
      static_cast<unsigned long>(config.gapMs),
      static_cast<unsigned long>(config.cooldownMs),
      static_cast<unsigned long>(config.rearmMs),
      static_cast<unsigned long>(config.sampleMs),
      static_cast<unsigned long>(config.themeFlipMs));
  Serial.printf("Adapt=%s mintravel=%lu step=%.1f%% | debug=%s | flash=%s\n",
                onOff(config.adapt),
                static_cast<unsigned long>(config.minTravelMs),
                config.adaptStepPct, onOff(config.debug),
                prefsReady ? "ready" : "unavailable");
}

void startAutoplay(bool clickToStart) {
  ++playGeneration;
  clearClickQueue();
  resetTiming();
  chooseBackground();
  playing = true;

  Serial.printf("[START] Armed. Background=%s, sensor=%d.\n",
                colorName(backgroundIsWhite), lastSensorValue);

  if (clickToStart && queueClick(nowMs(), true)) {
    Serial.println("[START] Initial click queued.");
  }
}

void stopAutoplay() {
  playing = false;
  ++playGeneration;
  clearClickQueue();
  resetEnvelope();
  servo.write(config.restAngle);
  Serial.println("[STOP] Autoplay stopped.");
}

bool setTheme(String value) {
  value = normalize(value);

  if (value == "auto") config.theme = ThemeMode::Auto;
  else if (value == "light") config.theme = ThemeMode::Light;
  else if (value == "dark") config.theme = ThemeMode::Dark;
  else return false;

  chooseBackground();
  resetEnvelope();
  saveConfig();
  Serial.printf("[THEME] %s.\n", themeName(config.theme));
  return true;
}

enum class SetResult { Changed, Invalid, Unknown };

SetResult setParameter(String rawName, String value) {
  const String name = normalize(rawName);
  long number = 0;
  bool changed = false;

  if (name == "threshold" || name == "rest" ||
      name == "press" || name == "clickangle") {
    if (!parseLong(value, number)) return SetResult::Invalid;

    if (name == "threshold" && number >= 0 && number <= 4095)
      config.threshold = static_cast<int>(number);
    else if (name == "rest" && number >= 0 && number <= 180)
      config.restAngle = static_cast<int>(number);
    else if ((name == "press" || name == "clickangle") &&
             number >= 0 && number <= 180)
      config.pressAngle = static_cast<int>(number);
    else
      return SetResult::Invalid;

    changed = true;
  } else if (name == "adapt" || name == "debug" || name == "autotheme") {
    bool flag = false;
    if (!parseBool(value, flag)) return SetResult::Invalid;

    if (name == "adapt") config.adapt = flag;
    else if (name == "debug") config.debug = flag;
    else {
      config.theme = flag
          ? ThemeMode::Auto
          : (backgroundIsWhite ? ThemeMode::Light : ThemeMode::Dark);
    }
    changed = true;
  } else if (name == "adaptstep") {
    float percent = 0.0f;
    if (!parseFloat(value, percent) || percent <= 0.0f || percent > 50.0f)
      return SetResult::Invalid;
    config.adaptStepPct = percent;
    changed = true;
  } else {
    for (const auto &setting : UINT_SETTINGS) {
      if (name != setting.name && name != setting.legacyName) continue;
      if (!parseLong(value, number) || number < 0 ||
          static_cast<uint32_t>(number) < setting.minValue ||
          static_cast<uint32_t>(number) > setting.maxValue) {
        return SetResult::Invalid;
      }
      config.*(setting.field) = static_cast<uint32_t>(number);
      changed = true;
      break;
    }
  }

  if (!changed) return SetResult::Unknown;

  if (name == "rest" && !playing) servo.write(config.restAngle);
  if (name == "travel" || name == "travelms" ||
      name == "mintravel" || name == "mintravelms" || name == "adapt") {
    resetTiming();
  }
  if (name == "threshold" || name == "autotheme") chooseBackground();

  saveConfig();
  Serial.printf("[SET] %s = %s\n", rawName.c_str(), value.c_str());
  return SetResult::Changed;
}

void restoreDefaults() {
  config = Config{};
  saveConfig();
  resetTiming();
  chooseBackground();
  if (!playing) servo.write(config.restAngle);
  Serial.println("[DEFAULTS] Defaults restored.");
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  // Old commands such as "set travel_ms 1550" still work.
  if (line.startsWith("set ")) {
    line.remove(0, 4);
    line.trim();
  }

  const int split = line.indexOf(' ');
  const String command = split < 0 ? line : line.substring(0, split);
  String value = split < 0 ? "" : line.substring(split + 1);
  value.trim();

  if (command == "help" || command == "manual" || command == "?") {
    return;
  } else if (command == "start" || command == "run") {
    startAutoplay(true);
  } else if (command == "arm") {
    startAutoplay(false);
  } else if (command == "stop" || command == "pause") {
    stopAutoplay();
  } else if (command == "click" || command == "jump") {
    if (queueClick(nowMs(), true)) Serial.println("[MANUAL] Click queued.");
  } else if (command == "show" || command == "status") {
    printStatus();
  } else if (command == "sensor") {
    const int sensor = analogRead(SENSOR_PIN);
    Serial.printf("[SENSOR] %d -> %s (threshold=%d)\n",
                  sensor, colorName(sensorIsWhite(sensor)), config.threshold);
  } else if (command == "reset") {
    resetTiming();
    Serial.println("[RESET] Learned timing cleared; settings kept.");
  } else if (command == "defaults") {
    restoreDefaults();
  } else if (command == "theme") {
    if (!setTheme(value))
      Serial.println("[ERR] Use: theme auto | theme light | theme dark");
  } else if (value.length() > 0) {
    const SetResult result = setParameter(command, value);
    if (result == SetResult::Invalid) Serial.println("[ERR] Invalid value.");
    else if (result == SetResult::Unknown) Serial.println("[ERR] Unknown setting.");
  } else {
    Serial.println("[ERR] Unknown command.");
  }
}

void processSerial() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        handleCommand(serialLine);
        serialLine = "";
        printManual();  // Always remind the user what is available.
      }
    } else if (serialLine.length() < 120) {
      serialLine += c;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(250);

  pinMode(SENSOR_PIN, INPUT);
  analogReadResolution(12);

  loadConfig();
  resetTiming();

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(config.restAngle);

  clickQueue = xQueueCreate(CLICK_QUEUE_SIZE, sizeof(ClickCommand));
  if (!clickQueue) {
    Serial.println("[FATAL] Could not create servo queue.");
    while (true) delay(1000);
  }

  if (xTaskCreatePinnedToCore(
          servoTask, "dino-servo", 4096, nullptr, 4,
          &clickTaskHandle, 1) != pdPASS) {
    Serial.println("[FATAL] Could not create servo task.");
    while (true) delay(1000);
  }

  chooseBackground();

  Serial.println("\nDino ESP32 Auto-Player ready.");
  Serial.printf("Servo GPIO %d | sensor GPIO %d\n", SERVO_PIN, SENSOR_PIN);
  printStatus();
  printManual();
}

void loop() {
  processSerial();
  processSensor();
  delay(1);
}
