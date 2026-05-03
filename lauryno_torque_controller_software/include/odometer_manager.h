#pragma once

#include <Arduino.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <stdint.h>

class FramStorage;

class OdometerManager {
 public:
  struct Config {
    uint16_t wheelDiameterMm = 0;
    uint16_t debounceMs = 0;
    uint32_t persistIntervalMs = 0;
    float maxSpeedKmh = 0.0f;
  };

  struct Status {
    bool initialized = false;
    bool hallActive = false;
    bool speedFresh = false;
    bool framReady = false;
    bool framError = false;
    uint32_t framErrorCount = 0;
    uint32_t acceptedPulses = 0;
    uint32_t rejectedPulses = 0;
    uint32_t lastAcceptedPulseMs = 0;
    uint32_t lastPersistMs = 0;
    uint64_t totalDistanceUm = 0;
    float totalDistanceKm = 0.0f;
    float speedKmh = 0.0f;
  };

  OdometerManager(gpio_num_t hallPin,
                  uint8_t wheelMagnets,
                  uint32_t speedStaleMs,
                  float hardSpeedLimitKmh,
                  FramStorage* framStorage,
                  const Config& defaultConfig);

  bool begin();
  void update();

  Status status() const;
  Config config() const;

  bool setConfig(const Config& config, String* errorMessage = nullptr);

 private:
  struct PersistedState {
    uint32_t magic;
    uint16_t version;
    uint16_t wheelDiameterMm;
    uint16_t debounceMs;
    uint16_t wheelMagnets;
    uint32_t persistIntervalMs;
    float maxSpeedKmh;
    uint64_t totalDistanceUm;
    uint32_t checksum;
  };

  static constexpr uint32_t kPersistMagic = 0x4F444F31u;
  static constexpr uint16_t kPersistVersion = 1;
  static constexpr uint16_t kPersistAddress = 0;

  bool validateConfig_(const Config& config, String* errorMessage) const;
  uint64_t distancePerPulseUm_() const;
  float speedFromDeltaMs_(uint32_t deltaMs) const;

  void processPulse_(uint32_t nowMs);

  bool loadPersistedState_();
  bool persistNow_();
  PersistedState makePersistedState_() const;

  static uint32_t checksum_(const uint8_t* data, size_t len);
  static uint32_t elapsedMs_(uint32_t nowMs, uint32_t startMs);

  gpio_num_t hallPin_;
  uint8_t wheelMagnets_;
  uint32_t speedStaleMs_;
  float hardSpeedLimitKmh_;
  FramStorage* framStorage_;

  Config config_;
  mutable SemaphoreHandle_t stateMutex_;

  bool initialized_;
  bool rawHallActive_;
  bool hallActive_;
  bool lastHallActive_;
  bool dirty_;
  bool distanceDirty_;

  uint32_t lastHallTransitionMs_;
  uint32_t lastObservedEdgeMs_;
  uint32_t lastAcceptedPulseMs_;
  uint32_t lastPersistMs_;

  uint64_t totalDistanceUm_;
  float speedKmh_;

  uint32_t acceptedPulses_;
  uint32_t rejectedPulses_;
};
