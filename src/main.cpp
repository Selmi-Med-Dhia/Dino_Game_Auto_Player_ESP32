#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {

constexpr int SERVO_PIN = 18;
constexpr int SENSOR_PIN = 15;
constexpr int SENSOR_HYSTERESIS = 8;
constexpr size_t CLICK_QUEUE_SIZE = 12;
constexpr size_t HISTORY_SIZE = 7;
constexpr size_t ADAPT_WARMUP = 5;
constexpr float DEFAULT_SENSOR_RATIO = 2.0f;
constexpr float ADAPT_SAFETY_BIAS = 1.05f;
constexpr uint32_t CONFIG_MAGIC = 0xD1A02026;
constexpr uint16_t CONFIG_VERSION = 2;

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
  uint32_t landingMs = 50;
  uint32_t clearanceMs = 100;
  uint32_t gapMs = 40;
  uint32_t cooldownMs = 70;
  uint32_t rearmMs = 12;
  uint32_t themeFlipMs = 1500;
  uint32_t sampleMs = 5;
  bool adapt = true;
  float adaptStepPct = 3.0f;
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
  bool debug;
};

struct Detector {
  bool active = false;
  uint64_t startMs = 0;
  uint64_t lastObstacleMs = 0;
  uint64_t oppositeSinceMs = 0;
  uint64_t lastLandingMs = 0;
  uint64_t lastCommandMs = 0;
  bool hasPlan = false;
  uint32_t travelMs = 0;
  uint32_t history[HISTORY_SIZE] = {};
  size_t historyCount = 0;
  size_t historyIndex = 0;
  uint32_t baselinePulseMs = 0;
  uint32_t plannedJumps = 0;
};

struct UIntSetting {
  const char *name;
  const char *legacy;
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
    {"gap", "gapms", &Config::gapMs, 1, 1000},
    {"cooldown", "cooldownms", &Config::cooldownMs, 0, 3000},
    {"rearm", "rearmms", &Config::rearmMs, 0, 3000},
    {"sample", "samplems", &Config::sampleMs, 2, 50},
    {"themeflip", "themeflipms", &Config::themeFlipMs, 100, 10000},
};

Servo servo;
Preferences prefs;
Config config;
Detector detector;
float sensorRatio = DEFAULT_SENSOR_RATIO;
QueueHandle_t clickQueue = nullptr;
SemaphoreHandle_t stateMutex = nullptr;
TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t servoTaskHandle = nullptr;
TaskHandle_t serialTaskHandle = nullptr;
volatile bool playing = false;
volatile uint32_t playGeneration = 1;
bool prefsReady = false;
bool backgroundIsWhite = true;
bool filteredWhite = true;
int lastSensorValue = 0;
String serialLine;

uint64_t nowMs() { return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL; }
const char *onOff(bool v) { return v ? "on" : "off"; }
const char *colorName(bool v) { return v ? "white" : "black"; }
const char *themeName(ThemeMode m) {
  return m == ThemeMode::Light ? "light" : m == ThemeMode::Dark ? "dark" : "auto";
}

Config getConfig() {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  Config copy = config;
  xSemaphoreGive(stateMutex);
  return copy;
}

String normalize(String s) {
  s.trim();
  s.toLowerCase();
  s.replace("_", "");
  s.replace("-", "");
  return s;
}

bool parseLong(const String &s, long &out) {
  char *end = nullptr;
  out = strtol(s.c_str(), &end, 10);
  while (end && *end == ' ') ++end;
  return end && end != s.c_str() && *end == '\0';
}

bool parseFloatValue(const String &s, float &out) {
  char *end = nullptr;
  out = strtof(s.c_str(), &end);
  while (end && *end == ' ') ++end;
  return end && end != s.c_str() && *end == '\0';
}

bool parseBool(String s, bool &out) {
  s = normalize(s);
  if (s == "on" || s == "true" || s == "1" || s == "yes") {
    out = true;
    return true;
  }
  if (s == "off" || s == "false" || s == "0" || s == "no") {
    out = false;
    return true;
  }
  return false;
}

// Persistent configuration -------------------------------------------------

bool configValid(const Config &c) {
  return c.magic == CONFIG_MAGIC && c.version == CONFIG_VERSION &&
         c.threshold >= 0 && c.threshold <= 4095 &&
         c.restAngle >= 0 && c.restAngle <= 180 &&
         c.pressAngle >= 0 && c.pressAngle <= 180 &&
         c.holdMs >= 1 && c.holdMs <= 2000 && c.actuatorMs <= 5000 &&
         c.travelMs >= 1 && c.travelMs <= 10000 &&
         c.minTravelMs >= 1 && c.minTravelMs <= 10000 &&
         c.airMs >= 1 && c.airMs <= 3000 && c.landingMs <= 2000 &&
         c.clearanceMs <= 3000 && c.gapMs >= 1 && c.gapMs <= 1000 &&
         c.cooldownMs <= 3000 && c.rearmMs <= 3000 &&
         c.themeFlipMs >= 100 && c.themeFlipMs <= 10000 &&
         c.sampleMs >= 2 && c.sampleMs <= 50 &&
         c.adaptStepPct > 0 && c.adaptStepPct <= 20 &&
         static_cast<uint8_t>(c.theme) <= 2;
}

void migrateV1(Config &c) {
  if (c.version != 1) return;
  c.version = CONFIG_VERSION;
  if (c.sampleMs == 2) c.sampleMs = 5;
  if (c.gapMs == 120) c.gapMs = 40;
  if (c.landingMs == 30) c.landingMs = 50;
  if (c.clearanceMs == 90) c.clearanceMs = 100;
  if (c.adaptStepPct == 6.0f) c.adaptStepPct = 3.0f;
}

bool saveConfig(bool announce = true) {
  if (!prefsReady) return false;
  Config copy = getConfig();
  bool ok = prefs.putBytes("config", &copy, sizeof(copy)) == sizeof(copy);
  if (announce) Serial.println(ok ? "[SAVE] Saved to flash." : "[SAVE] Flash write failed.");
  return ok;
}

void loadConfig() {
  prefsReady = prefs.begin("dino", false);
  if (!prefsReady) {
    Serial.println("[NVS] Flash unavailable; using defaults.");
    return;
  }

  bool hasSaved = prefs.getBytesLength("config") == sizeof(config);
  bool migrated = false;
  if (hasSaved) prefs.getBytes("config", &config, sizeof(config));
  if (hasSaved && config.magic == CONFIG_MAGIC && config.version == 1) {
    migrateV1(config);
    migrated = true;
  }

  if (!hasSaved || !configValid(config)) {
    config = Config{};
    prefs.putBytes("config", &config, sizeof(config));
    Serial.println("[NVS] Defaults loaded and saved.");
  } else {
    if (migrated) prefs.putBytes("config", &config, sizeof(config));
    Serial.println(migrated ? "[NVS] Configuration migrated."
                            : "[NVS] Saved configuration loaded.");
  }

  sensorRatio = prefs.getFloat("ratio", DEFAULT_SENSOR_RATIO);
  if (sensorRatio < 0.5f || sensorRatio > 6.0f) sensorRatio = DEFAULT_SENSOR_RATIO;
}

// Sensor and timing model --------------------------------------------------

int readSensor() { return analogRead(SENSOR_PIN); }

int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) b = a;
  return b;
}

bool classifyWhite(int adc, int threshold, bool previous) {
  return previous ? adc > threshold - SENSOR_HYSTERESIS
                  : adc > threshold + SENSOR_HYSTERESIS;
}

void resetEnvelopeLocked() {
  detector.active = false;
  detector.startMs = 0;
  detector.lastObstacleMs = 0;
  detector.oppositeSinceMs = 0;
}

void resetTimingLocked() {
  detector = Detector{};
  detector.travelMs = config.travelMs;
}

uint32_t speedPulseLocked() {
  uint32_t values[HISTORY_SIZE];
  for (size_t i = 0; i < detector.historyCount; ++i) values[i] = detector.history[i];
  for (size_t i = 1; i < detector.historyCount; ++i) {
    uint32_t v = values[i];
    size_t j = i;
    while (j && values[j - 1] > v) {
      values[j] = values[j - 1];
      --j;
    }
    values[j] = v;
  }
  size_t index = (detector.historyCount * 35) / 100;
  return values[index < detector.historyCount ? index : detector.historyCount - 1];
}

void updateTravelLocked(uint32_t pulseMs) {
  detector.history[detector.historyIndex] = pulseMs;
  detector.historyIndex = (detector.historyIndex + 1) % HISTORY_SIZE;
  if (detector.historyCount < HISTORY_SIZE) ++detector.historyCount;

  uint32_t pulse = speedPulseLocked();
  if (!detector.baselinePulseMs && detector.historyCount >= ADAPT_WARMUP)
    detector.baselinePulseMs = pulse;
  if (!config.adapt || !detector.baselinePulseMs || pulse >= detector.baselinePulseMs) return;

  float target = static_cast<float>(config.travelMs) * pulse /
                 detector.baselinePulseMs * ADAPT_SAFETY_BIAS;
  if (target < config.minTravelMs) target = config.minTravelMs;
  if (target >= detector.travelMs) return;

  uint32_t drop = static_cast<uint32_t>(detector.travelMs * config.adaptStepPct / 100.0f);
  if (!drop) drop = 1;
  uint32_t floor = detector.travelMs > drop ? detector.travelMs - drop : config.minTravelMs;
  detector.travelMs = static_cast<uint32_t>(target > floor ? target : floor);
  if (detector.travelMs < config.minTravelMs) detector.travelMs = config.minTravelMs;
}

void clearClickQueue() {
  ClickCommand ignored{};
  while (clickQueue && xQueueReceive(clickQueue, &ignored, 0) == pdTRUE) {}
}

bool queueClick(uint64_t atMs, bool manual, const Config &cfg) {
  ClickCommand cmd{atMs, playGeneration, cfg.pressAngle, cfg.restAngle,
                   cfg.holdMs, manual, cfg.debug};
  if (xQueueSend(clickQueue, &cmd, 0) == pdTRUE) return true;
  Serial.println("[WARN] Servo queue full; click dropped.");
  return false;
}

bool queueClick(uint64_t atMs, bool manual) {
  return queueClick(atMs, manual, getConfig());
}

void insertPending(ClickCommand *pending, size_t &count, const ClickCommand &cmd) {
  if (count >= CLICK_QUEUE_SIZE) return;
  size_t i = count;
  while (i && pending[i - 1].atMs > cmd.atMs) {
    pending[i] = pending[i - 1];
    --i;
  }
  pending[i] = cmd;
  ++count;
}

void prunePending(ClickCommand *pending, size_t &count) {
  size_t out = 0;
  for (size_t i = 0; i < count; ++i) {
    if (pending[i].manual ||
        (playing && pending[i].generation == playGeneration)) {
      pending[out++] = pending[i];
    }
  }
  count = out;
}

// Servo task ---------------------------------------------------------------

void servoTask(void *) {
  ClickCommand pending[CLICK_QUEUE_SIZE]{};
  size_t count = 0;

  for (;;) {
    prunePending(pending, count);
    TickType_t wait = portMAX_DELAY;
    if (count) {
      uint64_t now = nowMs();
      uint64_t ms = pending[0].atMs > now ? pending[0].atMs - now : 0;
      wait = ms ? pdMS_TO_TICKS(static_cast<uint32_t>(ms)) : 0;
    }

    ClickCommand incoming{};
    if (xQueueReceive(clickQueue, &incoming, wait) == pdTRUE) {
      insertPending(pending, count, incoming);
      continue;
    }
    if (!count) continue;

    ClickCommand cmd = pending[0];
    for (size_t i = 1; i < count; ++i) pending[i - 1] = pending[i];
    --count;
    if (!cmd.manual && (!playing || cmd.generation != playGeneration)) continue;

    uint64_t actual = nowMs();
    servo.write(cmd.pressAngle);
    TickType_t hold = pdMS_TO_TICKS(cmd.holdMs);
    vTaskDelay(hold ? hold : 1);
    servo.write(cmd.restAngle);

    if (cmd.debug) {
      uint64_t late = actual > cmd.atMs ? actual - cmd.atMs : 0;
      Serial.printf("[CLICK] task-late=%llu ms\n",
                    static_cast<unsigned long long>(late));
    }
  }
}

void finalizeEnvelopeLocked(uint64_t now) {
  if (!detector.active || detector.lastObstacleMs < detector.startMs) {
    resetEnvelopeLocked();
    return;
  }

  uint32_t pulseMs = static_cast<uint32_t>(detector.lastObstacleMs - detector.startMs);
  if (!pulseMs) pulseMs = 1;
  updateTravelLocked(pulseMs);

  // A wide optical spot makes the measured pulse wider than the cactus.
  // ratio=2 means the sensor spot is about twice a normal cactus width.
  float edgeFraction = sensorRatio / (2.0f * (sensorRatio + 1.0f));
  int64_t correction = static_cast<int64_t>(pulseMs * edgeFraction);
  int64_t entry = static_cast<int64_t>(detector.startMs) +
                  correction + detector.travelMs;
  int64_t exit = static_cast<int64_t>(detector.lastObstacleMs) -
                 correction + detector.travelMs;

  int64_t exitContact = exit + config.landingMs - config.airMs;
  int64_t safeContact = entry - config.clearanceMs;
  int64_t commandAt =
      (exitContact < safeContact ? exitContact : safeContact) - config.actuatorMs;

  bool entrySafety = safeContact < exitContact;
  bool delayedLanding = false;
  bool delayedCooldown = false;

  if (detector.hasPlan) {
    int64_t afterLanding = static_cast<int64_t>(detector.lastLandingMs) +
                           config.rearmMs - config.actuatorMs;
    int64_t afterCooldown =
        static_cast<int64_t>(detector.lastCommandMs) + config.cooldownMs;

    if (commandAt < afterLanding) {
      commandAt = afterLanding;
      delayedLanding = true;
    }
    if (commandAt < afterCooldown) {
      commandAt = afterCooldown;
      delayedCooldown = true;
    }
  }

  uint64_t lateMs = 0;
  if (commandAt < static_cast<int64_t>(now)) {
    lateMs = static_cast<uint64_t>(static_cast<int64_t>(now) - commandAt);
    commandAt = static_cast<int64_t>(now);
  }

  const char *reason = lateMs ? "sensor-too-close" :
                       delayedLanding ? "post-landing-rejump" :
                       delayedCooldown ? "cooldown" :
                       entrySafety ? "entry-safety" : "exit-aligned";
  uint64_t executeAt = static_cast<uint64_t>(commandAt);

  if (queueClick(executeAt, false, config)) {
    ++detector.plannedJumps;
    detector.lastCommandMs = executeAt;
    detector.lastLandingMs = executeAt + config.actuatorMs + config.airMs;
    detector.hasPlan = true;

    Serial.printf(
        "[PLAN] #%lu pulse=%lu travel=%lu cmd-in=%lld late=%llu %s\n",
        static_cast<unsigned long>(detector.plannedJumps),
        static_cast<unsigned long>(pulseMs),
        static_cast<unsigned long>(detector.travelMs),
        static_cast<long long>(
            static_cast<int64_t>(executeAt) - static_cast<int64_t>(now)),
        static_cast<unsigned long long>(lateMs), reason);
  }
  resetEnvelopeLocked();
}

// Sensor / planner task ----------------------------------------------------

void sensorTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  int first = readSensor();
  int samples[3] = {first, first, first};
  size_t sampleIndex = 0;
  filteredWhite = first > config.threshold;

  for (;;) {
    Config cfg = getConfig();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg.sampleMs));

    samples[sampleIndex] = readSensor();
    sampleIndex = (sampleIndex + 1) % 3;
    int adc = median3(samples[0], samples[1], samples[2]);
    uint64_t now = nowMs();
    bool themeChanged = false;
    bool newBackground = false;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    lastSensorValue = adc;
    filteredWhite = classifyWhite(adc, config.threshold, filteredWhite);

    if (playing) {
      bool obstacle = filteredWhite != backgroundIsWhite;

      if (config.theme == ThemeMode::Auto) {
        if (obstacle) {
          if (!detector.oppositeSinceMs) detector.oppositeSinceMs = now;
          if (now - detector.oppositeSinceMs >= config.themeFlipMs) {
            backgroundIsWhite = filteredWhite;
            newBackground = backgroundIsWhite;
            resetEnvelopeLocked();
            obstacle = false;
            themeChanged = true;
          }
        } else {
          detector.oppositeSinceMs = 0;
        }
      }

      if (obstacle) {
        if (!detector.active) {
          detector.active = true;
          detector.startMs = now;
        }
        detector.lastObstacleMs = now;
      } else if (detector.active &&
                 now - detector.lastObstacleMs >= config.gapMs) {
        finalizeEnvelopeLocked(now);
      }
    }
    xSemaphoreGive(stateMutex);

    if (themeChanged) {
      Serial.printf("[THEME] Background rebased to %s.\n",
                    colorName(newBackground));
    }
  }
}

void chooseBackground() {
  int adc = readSensor();
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  lastSensorValue = adc;
  filteredWhite = adc > config.threshold;
  backgroundIsWhite = config.theme == ThemeMode::Light ? true :
                      config.theme == ThemeMode::Dark ? false : filteredWhite;
  xSemaphoreGive(stateMutex);
}

// Serial console -----------------------------------------------------------

void printManual() {
  Serial.println("\nstart | arm | stop | click | show | sensor | reset | defaults");
  Serial.println("theme auto|light|dark");
  Serial.println("<name> <value>  (auto-saved)");
  Serial.println("threshold rest press hold actuator travel mintravel air landing");
  Serial.println("clearance gap sample cooldown rearm ratio adapt adaptstep debug");
  Serial.println("Example: press 38   travel 1550   ratio 2.0   adapt on\n");
}

void printStatus() {
  Config cfg = getConfig();
  int adc = readSensor();

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  float ratio = sensorRatio;
  uint32_t effective = detector.travelMs;
  bool bg = backgroundIsWhite;
  xSemaphoreGive(stateMutex);

  Serial.println("\n=== Dino Auto-Player ===");
  Serial.printf(
      "Run=%s | sensor=%d (%s) | threshold=%d | background=%s | theme=%s\n",
      playing ? "ON" : "OFF", adc, colorName(adc > cfg.threshold),
      cfg.threshold, colorName(bg), themeName(cfg.theme));
  Serial.printf("Servo: rest=%d press=%d hold=%lu ms\n",
                cfg.restAngle, cfg.pressAngle,
                static_cast<unsigned long>(cfg.holdMs));
  Serial.printf(
      "Timing: actuator=%lu travel=%lu effective=%lu air=%lu landing=%lu clearance=%lu\n",
      static_cast<unsigned long>(cfg.actuatorMs),
      static_cast<unsigned long>(cfg.travelMs),
      static_cast<unsigned long>(effective),
      static_cast<unsigned long>(cfg.airMs),
      static_cast<unsigned long>(cfg.landingMs),
      static_cast<unsigned long>(cfg.clearanceMs));
  Serial.printf("Sensor: sample=%lu ms gap=%lu ms ratio=%.2f:1 hysteresis=%d\n",
                static_cast<unsigned long>(cfg.sampleMs),
                static_cast<unsigned long>(cfg.gapMs), ratio,
                SENSOR_HYSTERESIS);
  Serial.printf("Adapt=%s mintravel=%lu step=%.1f%% | debug=%s | flash=%s\n",
                onOff(cfg.adapt),
                static_cast<unsigned long>(cfg.minTravelMs),
                cfg.adaptStepPct, onOff(cfg.debug),
                prefsReady ? "ready" : "unavailable");
}

void startAutoplay(bool clickToStart) {
  chooseBackground();

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  ++playGeneration;
  resetTimingLocked();
  playing = true;
  bool bg = backgroundIsWhite;
  int adc = lastSensorValue;
  xSemaphoreGive(stateMutex);
  clearClickQueue();

  Serial.printf("[START] Armed. Background=%s sensor=%d.\n",
                colorName(bg), adc);
  if (clickToStart && queueClick(nowMs(), true)) {
    Serial.println("[START] Initial click queued.");
  }
}

void stopAutoplay() {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  playing = false;
  ++playGeneration;
  resetEnvelopeLocked();
  int rest = config.restAngle;
  xSemaphoreGive(stateMutex);

  clearClickQueue();
  servo.write(rest);
  Serial.println("[STOP] Autoplay stopped.");
}

enum class SetResult { Changed, Invalid, Unknown };

SetResult setParameter(String rawName, String value) {
  String name = normalize(rawName);
  long number = 0;

  if (name == "ratio") {
    float ratio;
    if (!parseFloatValue(value, ratio) || ratio < 0.5f || ratio > 6.0f)
      return SetResult::Invalid;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    sensorRatio = ratio;
    xSemaphoreGive(stateMutex);
    if (prefsReady) prefs.putFloat("ratio", ratio);
    Serial.printf("[SET] ratio = %.2f\n", ratio);
    return SetResult::Changed;
  }

  bool changed = false;
  bool resetTiming = false;
  bool refreshBackground = false;
  bool moveRest = false;
  xSemaphoreTake(stateMutex, portMAX_DELAY);

  if (name == "threshold" || name == "rest" ||
      name == "press" || name == "clickangle") {
    if (!parseLong(value, number)) {
      xSemaphoreGive(stateMutex);
      return SetResult::Invalid;
    }

    if (name == "threshold" && number >= 0 && number <= 4095) {
      config.threshold = number;
    } else if (name == "rest" && number >= 0 && number <= 180) {
      config.restAngle = number;
    } else if ((name == "press" || name == "clickangle") &&
               number >= 0 && number <= 180) {
      config.pressAngle = number;
    } else {
      xSemaphoreGive(stateMutex);
      return SetResult::Invalid;
    }

    changed = true;
    refreshBackground = name == "threshold";
    moveRest = name == "rest" && !playing;
  } else if (name == "adapt" || name == "debug" || name == "autotheme") {
    bool flag;
    if (!parseBool(value, flag)) {
      xSemaphoreGive(stateMutex);
      return SetResult::Invalid;
    }

    if (name == "adapt") {
      config.adapt = flag;
      resetTiming = true;
    } else if (name == "debug") {
      config.debug = flag;
    } else {
      config.theme = flag ? ThemeMode::Auto
                          : (backgroundIsWhite ? ThemeMode::Light
                                               : ThemeMode::Dark);
    }
    changed = true;
    refreshBackground = name == "autotheme";
  } else if (name == "adaptstep") {
    float percent;
    if (!parseFloatValue(value, percent) || percent <= 0 || percent > 20) {
      xSemaphoreGive(stateMutex);
      return SetResult::Invalid;
    }
    config.adaptStepPct = percent;
    changed = true;
  } else {
    for (const auto &s : UINT_SETTINGS) {
      if (name != s.name && name != s.legacy) continue;
      if (!parseLong(value, number) || number < 0 ||
          static_cast<uint32_t>(number) < s.minValue ||
          static_cast<uint32_t>(number) > s.maxValue) {
        xSemaphoreGive(stateMutex);
        return SetResult::Invalid;
      }

      config.*(s.field) = static_cast<uint32_t>(number);
      changed = true;
      resetTiming = name == "travel" || name == "travelms" ||
                    name == "mintravel" || name == "mintravelms";
      break;
    }
  }

  if (resetTiming) resetTimingLocked();
  int rest = config.restAngle;
  xSemaphoreGive(stateMutex);

  if (!changed) return SetResult::Unknown;
  if (moveRest) servo.write(rest);
  if (refreshBackground) chooseBackground();
  saveConfig();
  Serial.printf("[SET] %s = %s\n", rawName.c_str(), value.c_str());
  return SetResult::Changed;
}

bool setTheme(String value) {
  value = normalize(value);
  ThemeMode mode;
  if (value == "auto") mode = ThemeMode::Auto;
  else if (value == "light") mode = ThemeMode::Light;
  else if (value == "dark") mode = ThemeMode::Dark;
  else return false;

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  config.theme = mode;
  resetEnvelopeLocked();
  xSemaphoreGive(stateMutex);

  chooseBackground();
  saveConfig();
  Serial.printf("[THEME] %s.\n", themeName(mode));
  return true;
}

void restoreDefaults() {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  config = Config{};
  sensorRatio = DEFAULT_SENSOR_RATIO;
  resetTimingLocked();
  bool moveRest = !playing;
  int rest = config.restAngle;
  xSemaphoreGive(stateMutex);

  if (moveRest) servo.write(rest);
  chooseBackground();
  saveConfig();
  if (prefsReady) prefs.putFloat("ratio", DEFAULT_SENSOR_RATIO);
  Serial.println("[DEFAULTS] Defaults restored and saved.");
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (!line.length()) return;

  // Keep compatibility with older commands such as "set travel_ms 1550".
  if (line.startsWith("set ")) {
    line.remove(0, 4);
    line.trim();
  }

  int split = line.indexOf(' ');
  String command = split < 0 ? line : line.substring(0, split);
  String value = split < 0 ? "" : line.substring(split + 1);
  value.trim();

  if (command == "help" || command == "manual" || command == "?") {
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
    Config cfg = getConfig();
    int adc = readSensor();
    Serial.printf("[SENSOR] ADC=%d -> %s (threshold=%d)\n",
                  adc, colorName(adc > cfg.threshold), cfg.threshold);
  } else if (command == "reset") {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    resetTimingLocked();
    xSemaphoreGive(stateMutex);
    Serial.println("[RESET] Learned timing cleared; saved settings kept.");
  } else if (command == "defaults") {
    restoreDefaults();
  } else if (command == "theme") {
    if (!setTheme(value)) {
      Serial.println("[ERR] Use: theme auto | theme light | theme dark");
    }
  } else if (value.length()) {
    SetResult result = setParameter(command, value);
    if (result == SetResult::Invalid) Serial.println("[ERR] Invalid value.");
    else if (result == SetResult::Unknown) Serial.println("[ERR] Unknown setting.");
  } else {
    Serial.println("[ERR] Unknown command.");
  }
}

void serialTask(void *) {
  for (;;) {
    while (Serial.available()) {
      char c = static_cast<char>(Serial.read());
      if (c == '\n' || c == '\r') {
        if (serialLine.length()) {
          handleCommand(serialLine);
          serialLine = "";
          printManual();
        }
      } else if (serialLine.length() < 120) {
        serialLine += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void fatal(const char *message) {
  Serial.println(message);
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(SENSOR_PIN, INPUT);
  analogReadResolution(12);

  stateMutex = xSemaphoreCreateMutex();
  if (!stateMutex) fatal("[FATAL] Could not create state mutex.");
  loadConfig();

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  resetTimingLocked();
  xSemaphoreGive(stateMutex);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(config.restAngle);

  clickQueue = xQueueCreate(CLICK_QUEUE_SIZE, sizeof(ClickCommand));
  if (!clickQueue) fatal("[FATAL] Could not create servo queue.");
  chooseBackground();

  if (xTaskCreatePinnedToCore(
          servoTask, "dino-servo", 4096, nullptr, 5,
          &servoTaskHandle, 1) != pdPASS ||
      xTaskCreatePinnedToCore(
          sensorTask, "dino-sensor", 4096, nullptr, 4,
          &sensorTaskHandle, 0) != pdPASS ||
      xTaskCreatePinnedToCore(
          serialTask, "dino-serial", 4096, nullptr, 2,
          &serialTaskHandle, 0) != pdPASS) {
    fatal("[FATAL] Could not create FreeRTOS tasks.");
  }

  Serial.println("\nDino ESP32 Auto-Player ready.");
  Serial.println("Runtime: sensor task + servo task + serial task. loop() stays idle.");
  printStatus();
  printManual();
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
