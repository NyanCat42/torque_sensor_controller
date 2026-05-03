#include "odometer_manager.h"

#include "app_config.h"
#include "fram_storage.h"

#include <cmath>
#include <cstddef>
#include <limits>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr uint32_t kHallEdgeConfirmMs = 3;
constexpr uint32_t kDistancePersistMinMs = 500;

}  // namespace

OdometerManager::OdometerManager(gpio_num_t hallPin,
                                 uint8_t wheelMagnets,
                                 uint32_t speedStaleMs,
                                 float hardSpeedLimitKmh,
                                 FramStorage* framStorage,
                                 const Config& defaultConfig)
    : hallPin_(hallPin),
      wheelMagnets_(wheelMagnets == 0 ? 1 : wheelMagnets),
      speedStaleMs_(speedStaleMs),
      hardSpeedLimitKmh_(hardSpeedLimitKmh),
      framStorage_(framStorage),
      config_(defaultConfig),
      stateMutex_(nullptr),
      initialized_(false),
      rawHallActive_(false),
      hallActive_(false),
      lastHallActive_(false),
      dirty_(false),
      distanceDirty_(false),
      lastHallTransitionMs_(0),
      lastObservedEdgeMs_(0),
      lastAcceptedPulseMs_(0),
      lastPersistMs_(0),
      totalDistanceUm_(0),
      speedKmh_(0.0f),
      acceptedPulses_(0),
      rejectedPulses_(0) {}

bool OdometerManager::begin() {
  String error;
  if (!validateConfig_(config_, &error)) {
    Serial.printf("[ODO] Invalid default config: %s\n", error.c_str());
    return false;
  }

  if (!stateMutex_) {
    stateMutex_ = xSemaphoreCreateMutex();
    if (!stateMutex_) {
      Serial.println("[ODO] Failed to create mutex.");
      return false;
    }
  }

  pinMode(static_cast<uint8_t>(hallPin_), INPUT);

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
    rawHallActive_ = digitalRead(static_cast<uint8_t>(hallPin_)) == LOW;
    hallActive_ = rawHallActive_;
    lastHallActive_ = hallActive_;
    lastHallTransitionMs_ = millis();
    xSemaphoreGive(stateMutex_);
  } else {
    Serial.println("[ODO] Mutex timeout during begin.");
    return false;
  }

  bool loaded = loadPersistedState_();
  if (!loaded) {
    if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
      dirty_ = true;
      xSemaphoreGive(stateMutex_);
    }
  }

  float totalDistanceKm = 0.0f;
  Config configSnapshot;
  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
    initialized_ = true;
    configSnapshot = config_;
    totalDistanceKm = static_cast<float>(static_cast<double>(totalDistanceUm_) / 1000000000.0);
    xSemaphoreGive(stateMutex_);
  } else {
    Serial.println("[ODO] Mutex timeout finalizing begin.");
    return false;
  }

  Serial.printf("[ODO] Started. distance=%.3f km, wheel=%u mm, debounce=%u ms, persist=%lu ms, max=%.1f km/h\n",
                static_cast<double>(totalDistanceKm),
                static_cast<unsigned>(configSnapshot.wheelDiameterMm),
                static_cast<unsigned>(configSnapshot.debounceMs),
                static_cast<unsigned long>(configSnapshot.persistIntervalMs),
                static_cast<double>(configSnapshot.maxSpeedKmh));
  return true;
}

void OdometerManager::update() {
  if (!stateMutex_) {
    return;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }

  if (!initialized_) {
    xSemaphoreGive(stateMutex_);
    return;
  }

  const uint32_t nowMs = millis();
  const bool rawHallActive = digitalRead(static_cast<uint8_t>(hallPin_)) == LOW;
  if (rawHallActive != rawHallActive_) {
    rawHallActive_ = rawHallActive;
    lastHallTransitionMs_ = nowMs;
  }

  // Require the hall input to stay low for a short window to reject EMI spikes.
  const bool hallActiveFiltered =
      rawHallActive_ && elapsedMs_(nowMs, lastHallTransitionMs_) >= kHallEdgeConfirmMs;
  hallActive_ = hallActiveFiltered;

  if (hallActiveFiltered && !lastHallActive_) {
    processPulse_(nowMs);
  }
  lastHallActive_ = hallActiveFiltered;

  if (speedKmh_ > 0.0f && lastAcceptedPulseMs_ != 0 && elapsedMs_(nowMs, lastAcceptedPulseMs_) > speedStaleMs_) {
    speedKmh_ = 0.0f;
  }

    const bool periodicPersistDue =
      dirty_ && config_.persistIntervalMs > 0 && elapsedMs_(nowMs, lastPersistMs_) >= config_.persistIntervalMs;
    const bool distancePersistDue =
      dirty_ && distanceDirty_ && elapsedMs_(nowMs, lastPersistMs_) >= kDistancePersistMinMs;
    const bool shouldPersist = periodicPersistDue || distancePersistDue;

  xSemaphoreGive(stateMutex_);

  if (shouldPersist) {
    persistNow_();
  }
}

OdometerManager::Status OdometerManager::status() const {
  Status status;
  if (!stateMutex_) {
    return status;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return status;
  }

  status.initialized = initialized_;
  status.hallActive = hallActive_;
  status.speedFresh = speedKmh_ > 0.0f;
  status.framReady = framStorage_ && framStorage_->isReady();
  status.framError = framStorage_ && framStorage_->hasError();
  status.framErrorCount = framStorage_ ? framStorage_->errorCount() : 0;
  status.acceptedPulses = acceptedPulses_;
  status.rejectedPulses = rejectedPulses_;
  status.lastAcceptedPulseMs = lastAcceptedPulseMs_;
  status.lastPersistMs = lastPersistMs_;
  status.totalDistanceUm = totalDistanceUm_;
  status.totalDistanceKm = static_cast<float>(static_cast<double>(totalDistanceUm_) / 1000000000.0);
  status.speedKmh = speedKmh_;

  xSemaphoreGive(stateMutex_);
  return status;
}

OdometerManager::Config OdometerManager::config() const {
  Config copy;
  if (!stateMutex_) {
    return copy;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return copy;
  }

  copy = config_;
  xSemaphoreGive(stateMutex_);
  return copy;
}

bool OdometerManager::setConfig(const Config& config, String* errorMessage) {
  if (!validateConfig_(config, errorMessage)) {
    return false;
  }

  if (!stateMutex_) {
    if (errorMessage) {
      *errorMessage = "not_initialized";
    }
    return false;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    if (errorMessage) {
      *errorMessage = "mutex_timeout";
    }
    return false;
  }

  config_ = config;
  dirty_ = true;

  xSemaphoreGive(stateMutex_);

  if (framStorage_ && framStorage_->isReady()) {
    if (!persistNow_() && errorMessage) {
      *errorMessage = "config_applied_but_persist_failed";
    }
  }

  return true;
}

bool OdometerManager::validateConfig_(const Config& config, String* errorMessage) const {
  if (config.wheelDiameterMm < AppConfig::kOdometerMinWheelDiameterMm ||
      config.wheelDiameterMm > AppConfig::kOdometerMaxWheelDiameterMm) {
    if (errorMessage) {
      *errorMessage = "wheel_diameter_out_of_range";
    }
    return false;
  }

  if (config.debounceMs < AppConfig::kOdometerMinDebounceMs ||
      config.debounceMs > AppConfig::kOdometerMaxDebounceMs) {
    if (errorMessage) {
      *errorMessage = "debounce_out_of_range";
    }
    return false;
  }

  if (config.persistIntervalMs < AppConfig::kOdometerMinPersistIntervalMs ||
      config.persistIntervalMs > AppConfig::kOdometerMaxPersistIntervalMs) {
    if (errorMessage) {
      *errorMessage = "persist_interval_out_of_range";
    }
    return false;
  }

  if (!(config.maxSpeedKmh > 0.0f) || config.maxSpeedKmh > hardSpeedLimitKmh_) {
    if (errorMessage) {
      *errorMessage = "max_speed_out_of_range";
    }
    return false;
  }

  return true;
}

uint64_t OdometerManager::distancePerPulseUm_() const {
  const float circumferenceMm = kPi * static_cast<float>(config_.wheelDiameterMm);
  const float pulseDistanceMm = circumferenceMm / static_cast<float>(wheelMagnets_);
  return static_cast<uint64_t>(llround(static_cast<double>(pulseDistanceMm) * 1000.0));
}

float OdometerManager::speedFromDeltaMs_(uint32_t deltaMs) const {
  if (deltaMs == 0) {
    return std::numeric_limits<float>::infinity();
  }

  const double pulseDistanceM = static_cast<double>(distancePerPulseUm_()) / 1000000.0;
  const double speedKmh = (pulseDistanceM / (static_cast<double>(deltaMs) / 1000.0)) * 3.6;
  return static_cast<float>(speedKmh);
}

void OdometerManager::processPulse_(uint32_t nowMs) {
  if (lastObservedEdgeMs_ != 0 && elapsedMs_(nowMs, lastObservedEdgeMs_) < config_.debounceMs) {
    ++rejectedPulses_;
    lastObservedEdgeMs_ = nowMs;
    return;
  }
  lastObservedEdgeMs_ = nowMs;

  if (lastAcceptedPulseMs_ != 0) {
    const uint32_t deltaMs = elapsedMs_(nowMs, lastAcceptedPulseMs_);
    const float speedKmh = speedFromDeltaMs_(deltaMs);
    if (!std::isfinite(speedKmh) || speedKmh > config_.maxSpeedKmh) {
      ++rejectedPulses_;
      return;
    }
    speedKmh_ = speedKmh;
  }

  ++acceptedPulses_;
  totalDistanceUm_ += distancePerPulseUm_();
  lastAcceptedPulseMs_ = nowMs;
  dirty_ = true;
  distanceDirty_ = true;
}

bool OdometerManager::loadPersistedState_() {
  if (!framStorage_ || !framStorage_->isReady()) {
    return false;
  }

  PersistedState state = {};
  if (!framStorage_->read(kPersistAddress, reinterpret_cast<uint8_t*>(&state), sizeof(state))) {
    return false;
  }

  const uint32_t expectedChecksum =
      checksum_(reinterpret_cast<const uint8_t*>(&state), offsetof(PersistedState, checksum));
  if (state.magic != kPersistMagic || state.version != kPersistVersion ||
      state.wheelMagnets != wheelMagnets_ || state.checksum != expectedChecksum) {
    return false;
  }

  Config loadedConfig;
  loadedConfig.wheelDiameterMm = state.wheelDiameterMm;
  loadedConfig.debounceMs = state.debounceMs;
  loadedConfig.persistIntervalMs = state.persistIntervalMs;
  loadedConfig.maxSpeedKmh = state.maxSpeedKmh;

  String error;
  if (!validateConfig_(loadedConfig, &error)) {
    Serial.printf("[ODO] Ignoring invalid persisted config: %s\n", error.c_str());
    return false;
  }

  if (!stateMutex_) {
    return false;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return false;
  }

  config_ = loadedConfig;
  totalDistanceUm_ = state.totalDistanceUm;

  xSemaphoreGive(stateMutex_);
  return true;
}

bool OdometerManager::persistNow_() {
  if (!framStorage_ || !framStorage_->isReady()) {
    return false;
  }

  if (!stateMutex_) {
    return false;
  }

  PersistedState state = {};
  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return false;
  }

  state = makePersistedState_();
  xSemaphoreGive(stateMutex_);

  const bool written =
      framStorage_->write(kPersistAddress, reinterpret_cast<const uint8_t*>(&state), sizeof(state));

  if (written && xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
    lastPersistMs_ = millis();
    dirty_ = false;
    distanceDirty_ = false;
    xSemaphoreGive(stateMutex_);
  }

  return written;
}

OdometerManager::PersistedState OdometerManager::makePersistedState_() const {
  PersistedState state = {};
  state.magic = kPersistMagic;
  state.version = kPersistVersion;
  state.wheelDiameterMm = config_.wheelDiameterMm;
  state.debounceMs = config_.debounceMs;
  state.wheelMagnets = wheelMagnets_;
  state.persistIntervalMs = config_.persistIntervalMs;
  state.maxSpeedKmh = config_.maxSpeedKmh;
  state.totalDistanceUm = totalDistanceUm_;
  state.checksum = checksum_(reinterpret_cast<const uint8_t*>(&state), offsetof(PersistedState, checksum));
  return state;
}

uint32_t OdometerManager::checksum_(const uint8_t* data, size_t len) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < len; ++i) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

uint32_t OdometerManager::elapsedMs_(uint32_t nowMs, uint32_t startMs) {
  return nowMs - startMs;
}
