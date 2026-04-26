#pragma once

#include <Arduino.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <stdint.h>

class FramStorage;
class VescCan;

class PedalAssistManager {
 public:
  struct Config {
    float assistRatio = 1.0f;
    float torqueRampStartNm = 20.0f;
    float torqueRampEndNm = 30.0f;
    float cadenceDisableRpm = 8.0f;
    float cadenceAverageRpm = 55.0f;
    float fastFilterAlpha = 0.70f;
    float maxCurrentA = 70.0f;
  };

  struct Status {
    bool initialized = false;
    bool cadenceFresh = false;
    bool cadenceActive = false;
    bool assistEnabled = false;
    bool highCadenceAveraging = false;
    bool failSafeActive = false;
    bool vescReady = false;
    bool controlLate = false;
    bool lastCanWriteOk = false;
    uint32_t updatedAtMs = 0;
    uint32_t lastCadencePulseMs = 0;
    uint32_t lastCommandMs = 0;
    uint32_t cadencePulsesTotal = 0;
    uint16_t torqueAdcRaw = 0;
    float cadenceRpm = 0.0f;
    float torqueRawNm = 0.0f;
    float torqueFilteredNm = 0.0f;
    float torqueRotationAvgNm = 0.0f;
    float torqueZeroOffsetV = 0.0f;
    float gpioVoltageV = 0.0f;
    float sensorVoltageV = 0.0f;
    float rampFactor = 0.0f;
    float assistRatio = 1.0f;
    float motorTorqueCmdNm = 0.0f;
    float motorCurrentCmdA = 0.0f;
  };

  PedalAssistManager(gpio_num_t torqueAdcPin,
                     gpio_num_t cadencePin,
                     uint8_t cadencePulsesPerRevolution,
                     uint32_t cadenceStaleMs,
                     uint32_t controlPeriodMs,
                     uint32_t controlLateFactor,
                     VescCan* vescCan,
                     FramStorage* framStorage,
                     const Config& defaultConfig);

  bool begin();
  void controlTick();

  Status status() const;
  Config config() const;

  bool setConfig(const Config& config, String* errorMessage = nullptr);
  bool calibrateZero(String* errorMessage = nullptr);

 private:
  struct PersistedState {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    float assistRatio;
    float torqueRampStartNm;
    float torqueRampEndNm;
    float cadenceDisableRpm;
    float cadenceAverageRpm;
    float fastFilterAlpha;
    float maxCurrentA;
    float torqueZeroOffsetV;
    uint32_t checksum;
  };

  static constexpr uint32_t kPersistMagic = 0x50444131u;
  static constexpr uint16_t kPersistVersion = 1;

  static void IRAM_ATTR onCadencePulseIsr_();

  bool validateConfig_(const Config& config, String* errorMessage) const;
  bool loadPersistedState_();
  bool persistNow_();

  PersistedState makePersistedState_() const;

  float sensorVoltageFromAdc_(uint16_t adcRaw) const;
  float torqueFromSensorVoltage_(float sensorVoltage, float zeroOffsetSensorVoltage) const;
  float torqueConstantOutputNmPerA_() const;
  float clamp01_(float value) const;

  static uint32_t checksum_(const uint8_t* data, size_t len);
  static uint32_t elapsedMs_(uint32_t nowMs, uint32_t startMs);

  gpio_num_t torqueAdcPin_;
  gpio_num_t cadencePin_;
  uint8_t cadencePulsesPerRevolution_;
  uint32_t cadenceStaleMs_;
  uint32_t controlPeriodMs_;
  uint32_t controlLateFactor_;
  VescCan* vescCan_;
  FramStorage* framStorage_;

  Config config_;

  mutable SemaphoreHandle_t stateMutex_;

  bool initialized_;
  bool dirty_;
  uint32_t lastPersistMs_;
  uint32_t lastControlMs_;
  uint32_t lastCadenceCalcMs_;
  uint32_t lastCadencePulseCount_;

  float torqueZeroOffsetV_;
  float fastFilteredTorqueNm_;
  bool fastFilterInitialized_;

  float rotationTorqueAccumulatorNm_;
  uint32_t rotationPulseAccumulator_;
  float rotationAverageTorqueNm_;
  bool rotationAverageValid_;

  Status lastStatus_;

  portMUX_TYPE cadenceMux_;
  volatile uint32_t cadencePulseCountIsr_;
  volatile uint32_t cadenceLastPulseUsIsr_;
  volatile uint32_t cadenceLastAcceptedPulseUsIsr_;

  static PedalAssistManager* isrOwner_;
};
