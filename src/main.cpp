#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <stdarg.h>
#include <stdio.h>
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
constexpr size_t LOG_QUEUE_SIZE = 16;
constexpr size_t WIDTH_HISTORY_SIZE = 15;
constexpr size_t CLASSIFY_WARMUP = 4;

constexpr float DEFAULT_SENSOR_RATIO = 2.0f;

// Chromium normal-mode speed model. The source increments currentSpeed by
// exactly 0.001 once per animation-frame update, starting at 6 and capped at 13.
// We emulate the browser frame rate, but intentionally re-anchor frame 0 to
// the trailing edge of the first cactus after boot/reset.
constexpr uint32_t DEFAULT_GAME_FPS = 60;
constexpr uint32_t MIN_GAME_FPS = 30;
constexpr uint32_t MAX_GAME_FPS = 360;
constexpr float GAME_START_SPEED = 6.0f;
constexpr float GAME_MAX_SPEED = 13.0f;
constexpr float GAME_ACCEL_PER_FRAME = 0.001f;
constexpr uint32_t GAME_MAX_ACCEL_FRAMES = 7000;

constexpr uint32_t CONFIG_MAGIC = 0xD1A02026;
constexpr uint16_t CONFIG_VERSION = 3;

enum class ThemeMode : uint8_t { Auto, Light, Dark };

// Exact v1/v2 layout, kept only for NVS migration.
struct LegacyConfig {
  uint32_t magic;
  uint16_t version;
  int threshold;
  int restAngle;
  int pressAngle;
  uint32_t holdMs;
  uint32_t actuatorMs;
  uint32_t travelMs;
  uint32_t minTravelMs;
  uint32_t airMs;
  uint32_t landingMs;
  uint32_t clearanceMs;
  uint32_t gapMs;
  uint32_t cooldownMs;
  uint32_t rearmMs;
  uint32_t themeFlipMs;
  uint32_t sampleMs;
  bool adapt;
  float adaptStepPct;
  ThemeMode theme;
  bool debug;
};

// Keep this layout unchanged from firmware v3 so existing calibration survives.
struct Config {
  uint32_t magic = CONFIG_MAGIC;
  uint16_t version = CONFIG_VERSION;
  int threshold = 200;
  int restAngle = 35;
  int pressAngle = 38;

  uint32_t holdMs = 80;
  uint32_t airMs = 450;

  uint32_t longHoldMs = 160;
  uint32_t longAirMs = 520;
  float longAtWidths = 1.60f;

  uint32_t actuatorMs = 160;
  uint32_t travelMs = 1550;
  uint32_t minTravelMs = 350;
  uint32_t landingMs = 50;
  uint32_t clearanceMs = 100;
  uint32_t gapMs = 40;
  uint32_t cooldownMs = 70;
  uint32_t rearmMs = 12;
  uint32_t themeFlipMs = 1500;
  uint32_t sampleMs = 5;

  bool adapt = true;          // enables Chromium frame-step acceleration scaling
  float adaptStepPct = 3.0f; // retained only for NVS compatibility
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

  uint32_t widthHistory[WIDTH_HISTORY_SIZE] = {};
  size_t widthCount = 0;
  size_t widthIndex = 0;

  bool roundStarted = false;
  uint64_t roundStartMs = 0;
  float speedScale = 1.0f;
  uint32_t speedFrame = 0;
  uint32_t travelMs = 0;
  uint32_t gapMs = 0;

  uint32_t plannedJumps = 0;
};

struct LogMessage {
  char text[176];
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
    {"longhold", "longholdms", &Config::longHoldMs, 1, 2000},
    {"air", "airms", &Config::airMs, 1, 3000},
    {"longair", "longairms", &Config::longAirMs, 1, 3000},
    {"actuator", "actuatorms", &Config::actuatorMs, 0, 5000},
    {"travel", "travelms", &Config::travelMs, 1, 10000},
    {"mintravel", "mintravelms", &Config::minTravelMs, 1, 10000},
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
uint32_t gameFps = DEFAULT_GAME_FPS;

QueueHandle_t clickQueue = nullptr;
QueueHandle_t logQueue = nullptr;
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

Config getConfig() {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  Config copy = config;
  xSemaphoreGive(stateMutex);
  return copy;
}

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

bool parseFloatValue(const String &text, float &value) {
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

void queueLog(const char *format, ...) {
  if (!logQueue) return;

  LogMessage message{};
  va_list args;
  va_start(args, format);
  vsnprintf(message.text, sizeof(message.text), format, args);
  va_end(args);
  xQueueSend(logQueue, &message, 0);
}

// --------------------------------------------------------------------------
// Persistent configuration
// --------------------------------------------------------------------------

bool configValid(const Config &c) {
  return c.magic == CONFIG_MAGIC &&
         c.version == CONFIG_VERSION &&
         c.threshold >= 0 && c.threshold <= 4095 &&
         c.restAngle >= 0 && c.restAngle <= 180 &&
         c.pressAngle >= 0 && c.pressAngle <= 180 &&
         c.holdMs >= 1 && c.holdMs <= 2000 &&
         c.longHoldMs >= 1 && c.longHoldMs <= 2000 &&
         c.airMs >= 1 && c.airMs <= 3000 &&
         c.longAirMs >= 1 && c.longAirMs <= 3000 &&
         c.longAtWidths >= 1.1f && c.longAtWidths <= 4.0f &&
         c.actuatorMs <= 5000 &&
         c.travelMs >= 1 && c.travelMs <= 10000 &&
         c.minTravelMs >= 1 && c.minTravelMs <= 10000 &&
         c.landingMs <= 2000 &&
         c.clearanceMs <= 3000 &&
         c.gapMs >= 1 && c.gapMs <= 1000 &&
         c.cooldownMs <= 3000 &&
         c.rearmMs <= 3000 &&
         c.themeFlipMs >= 100 && c.themeFlipMs <= 10000 &&
         c.sampleMs >= 2 && c.sampleMs <= 50 &&
         c.adaptStepPct > 0 && c.adaptStepPct <= 20 &&
         static_cast<uint8_t>(c.theme) <= 2;
}

bool legacyValid(const LegacyConfig &c) {
  return c.magic == CONFIG_MAGIC &&
         (c.version == 1 || c.version == 2) &&
         c.threshold >= 0 && c.threshold <= 4095 &&
         c.restAngle >= 0 && c.restAngle <= 180 &&
         c.pressAngle >= 0 && c.pressAngle <= 180 &&
         c.holdMs >= 1 && c.holdMs <= 2000 &&
         c.airMs >= 1 && c.airMs <= 3000 &&
         c.travelMs >= 1 && c.travelMs <= 10000 &&
         c.sampleMs >= 1 && c.sampleMs <= 1000;
}

void migrateLegacy(const LegacyConfig &old) {
  config = Config{};
  config.threshold = old.threshold;
  config.restAngle = old.restAngle;
  config.pressAngle = old.pressAngle;
  config.holdMs = old.holdMs;
  config.airMs = old.airMs;
  config.actuatorMs = old.actuatorMs;
  config.travelMs = old.travelMs;
  config.minTravelMs = old.minTravelMs;
  config.landingMs = old.landingMs;
  config.clearanceMs = old.clearanceMs;
  config.gapMs = old.gapMs;
  config.cooldownMs = old.cooldownMs;
  config.rearmMs = old.rearmMs;
  config.themeFlipMs = old.themeFlipMs;
  config.sampleMs = old.sampleMs;
  config.adapt = old.adapt;
  config.adaptStepPct = old.adaptStepPct;
  config.theme = old.theme;
  config.debug = old.debug;

  if (old.version == 1) {
    if (config.sampleMs == 2) config.sampleMs = 5;
    if (config.gapMs == 120) config.gapMs = 40;
    if (config.landingMs == 30) config.landingMs = 50;
    if (config.clearanceMs == 90) config.clearanceMs = 100;
    if (config.adaptStepPct == 6.0f) config.adaptStepPct = 3.0f;
  }
}

bool saveConfig(bool announce = true) {
  if (!prefsReady) return false;

  Config copy = getConfig();
  bool ok = prefs.putBytes("config", &copy, sizeof(copy)) == sizeof(copy);
  if (announce) {
    Serial.println(ok ? "[SAVE] Saved to flash."
                      : "[SAVE] Flash write failed.");
  }
  return ok;
}

void loadConfig() {
  prefsReady = prefs.begin("dino", false);
  if (!prefsReady) {
    Serial.println("[NVS] Flash unavailable; using defaults.");
    return;
  }

  size_t bytes = prefs.getBytesLength("config");
  bool migrated = false;

  if (bytes == sizeof(Config)) {
    prefs.getBytes("config", &config, sizeof(config));
  } else if (bytes == sizeof(LegacyConfig)) {
    LegacyConfig old{};
    prefs.getBytes("config", &old, sizeof(old));
    if (legacyValid(old)) {
      migrateLegacy(old);
      migrated = true;
    }
  }

  if (!configValid(config)) {
    config = Config{};
    Serial.println("[NVS] Defaults loaded.");
  } else if (migrated) {
    Serial.println("[NVS] Existing calibration migrated.");
  } else {
    Serial.println("[NVS] Saved configuration loaded.");
  }

  prefs.putBytes("config", &config, sizeof(config));

  sensorRatio = prefs.getFloat("ratio", DEFAULT_SENSOR_RATIO);
  if (sensorRatio < 0.5f || sensorRatio > 6.0f) {
    sensorRatio = DEFAULT_SENSOR_RATIO;
  }

  gameFps = prefs.getUInt("gamefps", DEFAULT_GAME_FPS);
  if (gameFps < MIN_GAME_FPS || gameFps > MAX_GAME_FPS) {
    gameFps = DEFAULT_GAME_FPS;
  }
}

// --------------------------------------------------------------------------
// Sensor + round-time model
// --------------------------------------------------------------------------

int readSensor() {
  return analogRead(SENSOR_PIN);
}

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

void resetRoundLocked() {
  detector = Detector{};
  detector.travelMs = config.travelMs;
  detector.gapMs = config.gapMs;
}

void updateRoundTimingLocked(uint64_t now) {
  float scale = 1.0f;
  uint32_t frame = 0;

  if (config.adapt && detector.roundStarted && now > detector.roundStartMs) {
    const uint64_t elapsedMs = now - detector.roundStartMs;

    // Chromium does not accelerate per cactus. In normal mode it runs
    // `currentSpeed += 0.001` once per requestAnimationFrame callback. The
    // callback rate normally follows the display refresh rate, so gameFps is
    // configurable to match the monitor running Chrome.
    uint64_t frames64 = elapsedMs * gameFps / 1000ULL;
    if (frames64 > GAME_MAX_ACCEL_FRAMES) frames64 = GAME_MAX_ACCEL_FRAMES;
    frame = static_cast<uint32_t>(frames64);

    float speed = GAME_START_SPEED + GAME_ACCEL_PER_FRAME * frame;
    if (speed > GAME_MAX_SPEED) speed = GAME_MAX_SPEED;
    scale = speed / GAME_START_SPEED;
  }

  detector.speedFrame = frame;
  detector.speedScale = scale;

  uint32_t travel =
      static_cast<uint32_t>(config.travelMs / scale + 0.5f);
  if (travel < config.minTravelMs) travel = config.minTravelMs;
  detector.travelMs = travel;

  uint32_t gap =
      static_cast<uint32_t>(config.gapMs / scale + 0.5f);
  uint32_t minGap = config.sampleMs * 3;
  detector.gapMs = gap > minGap ? gap : minGap;
}

void startRoundLocked(uint64_t firstCactusExitMs) {
  if (detector.roundStarted) return;
  detector.roundStarted = true;
  detector.roundStartMs = firstCactusExitMs;
  updateRoundTimingLocked(firstCactusExitMs);
}

uint32_t roundElapsedMsLocked(uint64_t now) {
  if (!detector.roundStarted || now <= detector.roundStartMs) return 0;
  uint64_t elapsed = now - detector.roundStartMs;
  return elapsed > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(elapsed);
}

void pushWidthPulseLocked(uint32_t pulseMs) {
  detector.widthHistory[detector.widthIndex] = pulseMs;
  detector.widthIndex = (detector.widthIndex + 1) % WIDTH_HISTORY_SIZE;
  if (detector.widthCount < WIDTH_HISTORY_SIZE) ++detector.widthCount;
}

uint32_t widthReferenceLocked() {
  if (!detector.widthCount) return 0;

  uint32_t values[WIDTH_HISTORY_SIZE];
  for (size_t i = 0; i < detector.widthCount; ++i) {
    values[i] = detector.widthHistory[i];
  }

  for (size_t i = 1; i < detector.widthCount; ++i) {
    uint32_t value = values[i];
    size_t j = i;
    while (j && values[j - 1] > value) {
      values[j] = values[j - 1];
      --j;
    }
    values[j] = value;
  }

  size_t index =
      detector.widthCount >= 3 ? 2 : detector.widthCount - 1;
  return values[index];
}

float estimateWidthLocked(uint32_t pulseMs, uint32_t referenceMs) {
  if (!referenceMs || detector.widthCount < CLASSIFY_WARMUP) return 1.0f;

  float cactusMs = referenceMs / (sensorRatio + 1.0f);
  float sensorMs = referenceMs - cactusMs;
  if (cactusMs < 1.0f) return 1.0f;

  float widths = (pulseMs - sensorMs) / cactusMs;
  return widths < 0.5f ? 0.5f : widths;
}

// --------------------------------------------------------------------------
// Servo scheduler
// --------------------------------------------------------------------------

void clearClickQueue() {
  ClickCommand ignored{};
  while (clickQueue && xQueueReceive(clickQueue, &ignored, 0) == pdTRUE) {}
}

bool queueClick(uint64_t atMs, bool manual,
                uint32_t holdMs, const Config &cfg) {
  ClickCommand cmd{
      atMs, playGeneration, cfg.pressAngle, cfg.restAngle,
      holdMs, manual, cfg.debug};

  if (xQueueSend(clickQueue, &cmd, 0) == pdTRUE) return true;
  queueLog("[WARN] Servo queue full; click dropped.");
  return false;
}

bool queueClick(uint64_t atMs, bool manual, bool longJump = false) {
  Config cfg = getConfig();
  return queueClick(
      atMs, manual, longJump ? cfg.longHoldMs : cfg.holdMs, cfg);
}

void insertPending(ClickCommand *pending, size_t &count,
                   const ClickCommand &cmd) {
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

void servoTask(void *) {
  ClickCommand pending[CLICK_QUEUE_SIZE]{};
  size_t count = 0;

  for (;;) {
    prunePending(pending, count);

    TickType_t wait = portMAX_DELAY;
    if (count) {
      uint64_t now = nowMs();
      uint64_t remaining =
          pending[0].atMs > now ? pending[0].atMs - now : 0;
      wait = remaining
          ? pdMS_TO_TICKS(static_cast<uint32_t>(remaining))
          : 0;
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

    if (!cmd.manual &&
        (!playing || cmd.generation != playGeneration)) {
      continue;
    }

    uint64_t actual = nowMs();
    servo.write(cmd.pressAngle);

    TickType_t hold = pdMS_TO_TICKS(cmd.holdMs);
    vTaskDelay(hold ? hold : 1);

    servo.write(cmd.restAngle);

    if (cmd.debug) {
      uint64_t late = actual > cmd.atMs ? actual - cmd.atMs : 0;
      queueLog("[CLICK] hold=%lu task-late=%llu ms",
               static_cast<unsigned long>(cmd.holdMs),
               static_cast<unsigned long long>(late));
    }
  }
}

// --------------------------------------------------------------------------
// Obstacle classification + planning
// --------------------------------------------------------------------------

void finalizeEnvelopeLocked(uint64_t now) {
  if (!detector.active ||
      detector.lastObstacleMs < detector.startMs) {
    resetEnvelopeLocked();
    return;
  }

  uint32_t pulseMs =
      static_cast<uint32_t>(
          detector.lastObstacleMs - detector.startMs);
  if (!pulseMs) pulseMs = 1;

  // The first fully-passed cactus after boot/reset/arm/start defines round t=0.
  startRoundLocked(detector.lastObstacleMs);
  updateRoundTimingLocked(now);

  pushWidthPulseLocked(pulseMs);
  uint32_t referenceMs = widthReferenceLocked();

  float widthEstimate = estimateWidthLocked(pulseMs, referenceMs);
  bool longJump =
      detector.widthCount < CLASSIFY_WARMUP ||
      widthEstimate >= config.longAtWidths;

  float sensorFootprintMs = referenceMs
      ? referenceMs * sensorRatio / (sensorRatio + 1.0f)
      : pulseMs * sensorRatio / (sensorRatio + 1.0f);
  int64_t edgeCorrection =
      static_cast<int64_t>(sensorFootprintMs * 0.5f);

  uint32_t holdMs =
      longJump ? config.longHoldMs : config.holdMs;
  uint32_t airMs =
      longJump ? config.longAirMs : config.airMs;

  int64_t entry =
      static_cast<int64_t>(detector.startMs) +
      edgeCorrection + detector.travelMs;
  int64_t exit =
      static_cast<int64_t>(detector.lastObstacleMs) -
      edgeCorrection + detector.travelMs;
  if (exit < entry) exit = entry;

  int64_t exitContact =
      exit + config.landingMs - airMs;
  int64_t safeContact =
      entry - config.clearanceMs;
  int64_t commandAt =
      (exitContact < safeContact ? exitContact : safeContact) -
      config.actuatorMs;

  bool entrySafety = safeContact < exitContact;
  bool delayedLanding = false;
  bool delayedCooldown = false;

  if (detector.hasPlan) {
    int64_t afterLanding =
        static_cast<int64_t>(detector.lastLandingMs) +
        config.rearmMs - config.actuatorMs;
    int64_t afterCooldown =
        static_cast<int64_t>(detector.lastCommandMs) +
        config.cooldownMs;

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
    lateMs =
        static_cast<uint64_t>(
            static_cast<int64_t>(now) - commandAt);
    commandAt = static_cast<int64_t>(now);
  }

  const char *reason =
      lateMs ? "sensor-too-close" :
      delayedLanding ? "post-landing-rejump" :
      delayedCooldown ? "cooldown" :
      entrySafety ? "entry-safety" :
      "exit-aligned";

  uint64_t executeAt =
      static_cast<uint64_t>(commandAt);

  if (queueClick(executeAt, false, holdMs, config)) {
    ++detector.plannedJumps;
    detector.lastCommandMs = executeAt;
    detector.lastLandingMs =
        executeAt + config.actuatorMs + airMs;
    detector.hasPlan = true;

    uint32_t roundMs = roundElapsedMsLocked(now);
    queueLog(
        "[PLAN] #%lu %s width=%.2f round=%.1fs frame=%lu speed=x%.3f "
        "travel=%lu gap=%lu hold=%lu cmd-in=%lld late=%llu %s",
        static_cast<unsigned long>(detector.plannedJumps),
        longJump ? "LONG" : "SHORT",
        widthEstimate,
        roundMs / 1000.0f,
        static_cast<unsigned long>(detector.speedFrame),
        detector.speedScale,
        static_cast<unsigned long>(detector.travelMs),
        static_cast<unsigned long>(detector.gapMs),
        static_cast<unsigned long>(holdMs),
        static_cast<long long>(
            static_cast<int64_t>(executeAt) -
            static_cast<int64_t>(now)),
        static_cast<unsigned long long>(lateMs),
        reason);
  }

  resetEnvelopeLocked();
}

// --------------------------------------------------------------------------
// Sensor task
// --------------------------------------------------------------------------

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
    filteredWhite =
        classifyWhite(adc, config.threshold, filteredWhite);

    // Frame-step scaling is deterministic and independent of obstacle width.
    updateRoundTimingLocked(now);

    if (playing) {
      bool obstacle =
          filteredWhite != backgroundIsWhite;

      if (config.theme == ThemeMode::Auto) {
        if (obstacle) {
          if (!detector.oppositeSinceMs) {
            detector.oppositeSinceMs = now;
          }

          if (now - detector.oppositeSinceMs >=
              config.themeFlipMs) {
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
      } else if (
          detector.active &&
          now - detector.lastObstacleMs >= detector.gapMs) {
        finalizeEnvelopeLocked(now);
      }
    }

    xSemaphoreGive(stateMutex);

    if (themeChanged) {
      queueLog("[THEME] Background rebased to %s.",
               colorName(newBackground));
    }
  }
}

void chooseBackground() {
  int adc = readSensor();

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  lastSensorValue = adc;
  filteredWhite = adc > config.threshold;
  backgroundIsWhite =
      config.theme == ThemeMode::Light ? true :
      config.theme == ThemeMode::Dark ? false :
      filteredWhite;
  xSemaphoreGive(stateMutex);
}

// --------------------------------------------------------------------------
// Serial console
// --------------------------------------------------------------------------

void printManual() {
  Serial.println(
      "\nstart | arm | stop | click | longclick | show | sensor | reset | defaults");
  Serial.println("theme auto|light|dark");
  Serial.println("<name> <value>  (auto-saved)");
  Serial.println("threshold rest press hold longhold air longair longat");
  Serial.println(
      "actuator travel mintravel landing clearance gap sample cooldown rearm");
  Serial.println("ratio gamefps adapt debug");
  Serial.println(
      "Round clock: first cactus = frame 0; speed += 0.001 each browser frame\n");
}

void printStatus() {
  Config cfg = getConfig();
  int adc = readSensor();
  uint64_t now = nowMs();

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  float ratio = sensorRatio;
  uint32_t fps = gameFps;
  float speed = detector.speedScale;
  uint32_t speedFrame = detector.speedFrame;
  uint32_t effectiveTravel = detector.travelMs;
  uint32_t effectiveGap = detector.gapMs;
  bool bg = backgroundIsWhite;
  bool roundStarted = detector.roundStarted;
  uint32_t roundMs = roundElapsedMsLocked(now);
  uint32_t ref = widthReferenceLocked();
  xSemaphoreGive(stateMutex);

  Serial.println("\n=== Dino Auto-Player ===");
  Serial.printf(
      "Run=%s | round=%s",
      playing ? "ON" : "OFF",
      roundStarted ? "RUNNING" : "WAITING FOR FIRST CACTUS");

  if (roundStarted) {
    Serial.printf(" (%.1f s)", roundMs / 1000.0f);
  }
  Serial.println();

  Serial.printf(
      "Sensor=%d (%s) | threshold=%d | background=%s | theme=%s\n",
      adc, colorName(adc > cfg.threshold),
      cfg.threshold, colorName(bg), themeName(cfg.theme));

  Serial.printf(
      "Servo: rest=%d press=%d | SHORT hold=%lu air=%lu | LONG hold=%lu air=%lu\n",
      cfg.restAngle, cfg.pressAngle,
      static_cast<unsigned long>(cfg.holdMs),
      static_cast<unsigned long>(cfg.airMs),
      static_cast<unsigned long>(cfg.longHoldMs),
      static_cast<unsigned long>(cfg.longAirMs));

  Serial.printf(
      "Long jump: width >= %.2f cactus-widths | pulse reference=%lu ms\n",
      cfg.longAtWidths,
      static_cast<unsigned long>(ref));

  Serial.printf(
      "Acceleration: frame=%lu speed=x%.3f | timed=%s | +0.001/frame @%luHz\n",
      static_cast<unsigned long>(speedFrame), speed, onOff(cfg.adapt),
      static_cast<unsigned long>(fps));

  Serial.printf(
      "Timing: travel=%lu->%lu gap=%lu->%lu actuator=%lu landing=%lu clearance=%lu\n",
      static_cast<unsigned long>(cfg.travelMs),
      static_cast<unsigned long>(effectiveTravel),
      static_cast<unsigned long>(cfg.gapMs),
      static_cast<unsigned long>(effectiveGap),
      static_cast<unsigned long>(cfg.actuatorMs),
      static_cast<unsigned long>(cfg.landingMs),
      static_cast<unsigned long>(cfg.clearanceMs));

  Serial.printf(
      "Sensor: sample=%lu ratio=%.2f:1 hysteresis=%d | debug=%s | flash=%s\n",
      static_cast<unsigned long>(cfg.sampleMs),
      ratio, SENSOR_HYSTERESIS,
      onOff(cfg.debug),
      prefsReady ? "ready" : "unavailable");
}

void armFreshRound(bool clickToStart) {
  chooseBackground();
  uint64_t now = nowMs();

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  ++playGeneration;
  resetRoundLocked();
  playing = true;
  bool bg = backgroundIsWhite;
  int adc = lastSensorValue;
  xSemaphoreGive(stateMutex);

  clearClickQueue();

  Serial.printf(
      "[ARM] Ready. Round clock will start when the next cactus passes. "
      "Background=%s sensor=%d.\n",
      colorName(bg), adc);

  if (clickToStart && queueClick(now, true, false)) {
    Serial.println("[START] Initial short click queued.");
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

enum class SetResult {
  Changed,
  Invalid,
  Unknown
};

SetResult setParameter(String rawName, String value) {
  String name = normalize(rawName);
  long number = 0;

  if (name == "gamefps" || name == "fps") {
    if (!parseLong(value, number) ||
        number < static_cast<long>(MIN_GAME_FPS) ||
        number > static_cast<long>(MAX_GAME_FPS)) {
      return SetResult::Invalid;
    }

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    gameFps = static_cast<uint32_t>(number);
    updateRoundTimingLocked(nowMs());
    xSemaphoreGive(stateMutex);

    if (prefsReady) prefs.putUInt("gamefps", gameFps);
    Serial.printf("[SET] gamefps = %lu\n",
                  static_cast<unsigned long>(gameFps));
    return SetResult::Changed;
  }

  if (name == "ratio") {
    float ratio;
    if (!parseFloatValue(value, ratio) ||
        ratio < 0.5f || ratio > 6.0f) {
      return SetResult::Invalid;
    }

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    sensorRatio = ratio;
    xSemaphoreGive(stateMutex);

    if (prefsReady) prefs.putFloat("ratio", ratio);
    Serial.printf("[SET] ratio = %.2f\n", ratio);
    return SetResult::Changed;
  }

  bool changed = false;
  bool resetRound = false;
  bool refreshBackground = false;
  bool moveRest = false;

  xSemaphoreTake(stateMutex, portMAX_DELAY);

  if (name == "threshold" ||
      name == "rest" ||
      name == "press" ||
      name == "clickangle") {
    if (!parseLong(value, number)) {
      xSemaphoreGive(stateMutex);
      return SetResult::Invalid;
    }

    if (name == "threshold" &&
        number >= 0 && number <= 4095) {
      config.threshold = static_cast<int>(number);
    } else if (name == "rest" &&
               number >= 0 && number <= 180) {
      config.restAngle = static_cast<int>(number);
    } else if (
        (name == "press" || name == "clickangle") &&
        number >= 0 && number <= 180) {
      config.pressAngle = static_cast<int>(number);
    } else {
      xSemaphoreGive(stateMutex);
      return SetResult::Invalid;
    }

    changed = true;
    refreshBackground = name == "threshold";
    moveRest = name == "rest" && !playing;

  } else if (
      name == "adapt" ||
      name == "debug" ||
      name == "autotheme") {
    bool flag;
    if (!parseBool(value, flag)) {
      xSemaphoreGive(stateMutex);
      return SetResult::Invalid;
    }

    if (name == "adapt") {
      config.adapt = flag;
      resetRound = true;
    } else if (name == "debug") {
      config.debug = flag;
    } else {
      config.theme =
          flag ? ThemeMode::Auto :
          (backgroundIsWhite ? ThemeMode::Light : ThemeMode::Dark);
      refreshBackground = true;
    }

    changed = true;

  } else if (name == "longat") {
    float width;
    if (!parseFloatValue(value, width) ||
        width < 1.1f || width > 4.0f) {
      xSemaphoreGive(stateMutex);
      return SetResult::Invalid;
    }
    config.longAtWidths = width;
    changed = true;

  } else if (name == "adaptstep") {
    // Kept only so old serial habits do not fail. Frame-step acceleration does
    // not need a per-obstacle step anymore, but the value remains in NVS v3.
    float valueFloat;
    if (!parseFloatValue(value, valueFloat) ||
        valueFloat <= 0 || valueFloat > 20) {
      xSemaphoreGive(stateMutex);
      return SetResult::Invalid;
    }
    config.adaptStepPct = valueFloat;
    changed = true;

  } else {
    for (const auto &setting : UINT_SETTINGS) {
      if (name != setting.name && name != setting.legacy) continue;

      if (!parseLong(value, number) ||
          number < 0 ||
          static_cast<uint32_t>(number) < setting.minValue ||
          static_cast<uint32_t>(number) > setting.maxValue) {
        xSemaphoreGive(stateMutex);
        return SetResult::Invalid;
      }

      config.*(setting.field) =
          static_cast<uint32_t>(number);
      changed = true;

      resetRound =
          name == "travel" ||
          name == "travelms" ||
          name == "mintravel" ||
          name == "mintravelms" ||
          name == "gap" ||
          name == "gapms";
      break;
    }
  }

  if (resetRound) resetRoundLocked();
  int rest = config.restAngle;
  xSemaphoreGive(stateMutex);

  if (!changed) return SetResult::Unknown;

  if (moveRest) servo.write(rest);
  if (refreshBackground) chooseBackground();

  saveConfig();
  Serial.printf("[SET] %s = %s\n",
                rawName.c_str(), value.c_str());
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
  gameFps = DEFAULT_GAME_FPS;
  resetRoundLocked();
  bool moveRest = !playing;
  int rest = config.restAngle;
  xSemaphoreGive(stateMutex);

  if (moveRest) servo.write(rest);
  chooseBackground();
  saveConfig();

  if (prefsReady) {
    prefs.putFloat("ratio", DEFAULT_SENSOR_RATIO);
    prefs.putUInt("gamefps", DEFAULT_GAME_FPS);
  }

  Serial.println("[DEFAULTS] Defaults restored and saved.");
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (!line.length()) return;

  if (line.startsWith("set ")) {
    line.remove(0, 4);
    line.trim();
  }

  int split = line.indexOf(' ');
  String command =
      split < 0 ? line : line.substring(0, split);
  String value =
      split < 0 ? "" : line.substring(split + 1);
  value.trim();

  if (command == "help" ||
      command == "manual" ||
      command == "?") {
    // Manual is printed after every command.

  } else if (command == "start" ||
             command == "run") {
    armFreshRound(true);

  } else if (command == "arm") {
    armFreshRound(false);

  } else if (command == "stop" ||
             command == "pause") {
    stopAutoplay();

  } else if (command == "click" ||
             command == "jump") {
    if (queueClick(nowMs(), true, false)) {
      Serial.println("[MANUAL] Short click queued.");
    }

  } else if (command == "longclick" ||
             command == "longjump") {
    if (queueClick(nowMs(), true, true)) {
      Serial.println("[MANUAL] Long click queued.");
    }

  } else if (command == "show" ||
             command == "status") {
    printStatus();

  } else if (command == "sensor") {
    Config cfg = getConfig();
    int adc = readSensor();
    Serial.printf(
        "[SENSOR] ADC=%d -> %s (threshold=%d)\n",
        adc, colorName(adc > cfg.threshold), cfg.threshold);

  } else if (command == "reset") {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    ++playGeneration;
    resetRoundLocked();
    playing = true;
    xSemaphoreGive(stateMutex);
    clearClickQueue();

    Serial.println(
        "[RESET] Round reset. Autoplay remains armed; "
        "the next cactus becomes t=0.");

  } else if (command == "defaults") {
    restoreDefaults();

  } else if (command == "theme") {
    if (!setTheme(value)) {
      Serial.println(
          "[ERR] Use: theme auto | theme light | theme dark");
    }

  } else if (value.length()) {
    SetResult result = setParameter(command, value);
    if (result == SetResult::Invalid) {
      Serial.println("[ERR] Invalid value.");
    } else if (result == SetResult::Unknown) {
      Serial.println("[ERR] Unknown setting.");
    }

  } else {
    Serial.println("[ERR] Unknown command.");
  }
}

void serialTask(void *) {
  for (;;) {
    LogMessage message{};
    while (xQueueReceive(logQueue, &message, 0) == pdTRUE) {
      Serial.println(message.text);
    }

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
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(SENSOR_PIN, INPUT);
  analogReadResolution(12);

  stateMutex = xSemaphoreCreateMutex();
  if (!stateMutex) {
    fatal("[FATAL] Could not create state mutex.");
  }

  loadConfig();

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  resetRoundLocked();
  playing = true;  // Always armed after boot/reset.
  ++playGeneration;
  xSemaphoreGive(stateMutex);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(config.restAngle);

  clickQueue =
      xQueueCreate(CLICK_QUEUE_SIZE, sizeof(ClickCommand));
  logQueue =
      xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogMessage));

  if (!clickQueue || !logQueue) {
    fatal("[FATAL] Could not create runtime queues.");
  }

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

  Serial.println(
      "\nDino ESP32 Auto-Player ready "
      "(auto-armed + Chromium frame-step acceleration).");
  Serial.println(
      "[AUTO] Waiting for the first cactus; "
      "its trailing edge becomes round t=0.");
  Serial.println(
      "Runtime: sensor task + servo task + serial task. "
      "loop() stays idle.");

  printStatus();
  printManual();
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
