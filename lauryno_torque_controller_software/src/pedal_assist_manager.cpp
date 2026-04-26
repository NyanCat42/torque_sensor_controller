#include "pedal_assist_manager.h"

#include "app_config.h"
#include "fram_storage.h"
#include "vesc_can.h"

#include <esp32-hal-adc.h>

#include <algorithm>
#include <cmath>

PedalAssistManager* PedalAssistManager::isrOwner_ = nullptr;

PedalAssistManager::PedalAssistManager(gpio_num_t torqueAdcPin,
                                       gpio_num_t cadencePin,
                                       uint8_t cadencePulsesPerRevolution,
                                       uint32_t cadenceStaleMs,
                                       uint32_t controlPeriodMs,
                                       uint32_t controlLateFactor,
                                       VescCan* vescCan,
                                       FramStorage* framStorage,
                                       const Config& defaultConfig)
    : torqueAdcPin_(torqueAdcPin),
      cadencePin_(cadencePin),
      cadencePulsesPerRevolution_(cadencePulsesPerRevolution == 0 ? 1 : cadencePulsesPerRevolution),
      cadenceStaleMs_(cadenceStaleMs),
      controlPeriodMs_(controlPeriodMs == 0 ? 10 : controlPeriodMs),
      controlLateFactor_(controlLateFactor == 0 ? 4 : controlLateFactor),
      vescCan_(vescCan),
      framStorage_(framStorage),
      config_(defaultConfig),
      stateMutex_(nullptr),
      initialized_(false),
      dirty_(false),
      lastPersistMs_(0),
      lastControlMs_(0),
      lastCadenceCalcMs_(0),
      lastCadencePulseCount_(0),
      torqueZeroOffsetV_(0.0f),
      fastFilteredTorqueNm_(0.0f),
      fastFilterInitialized_(false),
      rotationTorqueAccumulatorNm_(0.0f),
      rotationPulseAccumulator_(0),
      rotationAverageTorqueNm_(0.0f),
      rotationAverageValid_(false),
      cadenceMux_(portMUX_INITIALIZER_UNLOCKED),
      cadencePulseCountIsr_(0),
      cadenceLastPulseUsIsr_(0),
      cadenceLastAcceptedPulseUsIsr_(0) {}

bool PedalAssistManager::begin() {
  String error;
  if (!validateConfig_(config_, &error)) {
    Serial.printf("[PEDAL] Invalid default config: %s\n", error.c_str());
    return false;
  }

  stateMutex_ = xSemaphoreCreateMutex();
  if (!stateMutex_) {
    Serial.println("[PEDAL] Failed to create mutex.");
    return false;
  }

  pinMode(static_cast<uint8_t>(torqueAdcPin_), INPUT);
  analogReadResolution(AppConfig::kPedalAdcResolutionBits);
  analogSetPinAttenuation(static_cast<uint8_t>(torqueAdcPin_), ADC_11db);

  pinMode(static_cast<uint8_t>(cadencePin_), INPUT_PULLUP);

  isrOwner_ = this;
  attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(cadencePin_)),
                  &PedalAssistManager::onCadencePulseIsr_,
                  RISING);

  if (!loadPersistedState_()) {
    dirty_ = true;
  }

  const int adcRawInit = analogRead(static_cast<uint8_t>(torqueAdcPin_));
  const uint16_t adcRaw = static_cast<uint16_t>(adcRawInit < 0 ? 0 : adcRawInit);
  const float sensorVoltage = sensorVoltageFromAdc_(adcRaw);

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
    lastStatus_.torqueAdcRaw = adcRaw;
    lastStatus_.gpioVoltageV = sensorVoltage / ((AppConfig::kPedalDividerR1Ohm + AppConfig::kPedalDividerR2Ohm) /
                                                 AppConfig::kPedalDividerR2Ohm);
    lastStatus_.sensorVoltageV = sensorVoltage;
    lastStatus_.torqueZeroOffsetV = torqueZeroOffsetV_;
    lastStatus_.initialized = true;
    xSemaphoreGive(stateMutex_);
  }

  initialized_ = true;
  Serial.printf("[PEDAL] Started. ratio=%.2f ramp=%.1f..%.1f Nm cadence disable>=%.1f rpm avg>=%.1f rpm\n",
                static_cast<double>(config_.assistRatio),
                static_cast<double>(config_.torqueRampStartNm),
                static_cast<double>(config_.torqueRampEndNm),
                static_cast<double>(config_.cadenceDisableRpm),
                static_cast<double>(config_.cadenceAverageRpm));
  return true;
}

void PedalAssistManager::controlTick() {
  if (!initialized_) {
    return;
  }

  Config configSnapshot;
  float zeroOffsetSnapshot = 0.0f;
  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    configSnapshot = config_;
    zeroOffsetSnapshot = torqueZeroOffsetV_;
    xSemaphoreGive(stateMutex_);
  } else {
    return;
  }

  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();

  bool controlLate = false;
  if (lastControlMs_ != 0) {
    const uint32_t lateThreshold = controlPeriodMs_ * controlLateFactor_;
    controlLate = elapsedMs_(nowMs, lastControlMs_) > lateThreshold;
  }
  lastControlMs_ = nowMs;

  uint32_t pulseCount = 0;
  uint32_t lastPulseUs = 0;
  portENTER_CRITICAL(&cadenceMux_);
  pulseCount = cadencePulseCountIsr_;
  lastPulseUs = cadenceLastPulseUsIsr_;
  portEXIT_CRITICAL(&cadenceMux_);

  const uint32_t deltaPulses = pulseCount - lastCadencePulseCount_;
  lastCadencePulseCount_ = pulseCount;

  uint32_t deltaMs = elapsedMs_(nowMs, lastCadenceCalcMs_);
  if (lastCadenceCalcMs_ == 0 || deltaMs == 0) {
    deltaMs = controlPeriodMs_;
  }
  lastCadenceCalcMs_ = nowMs;

  float cadenceRpm = 0.0f;
  if (deltaPulses > 0 && deltaMs > 0) {
    cadenceRpm = (static_cast<float>(deltaPulses) * 60000.0f) /
                 (static_cast<float>(cadencePulsesPerRevolution_) * static_cast<float>(deltaMs));
  }

  const bool cadenceFresh =
      (lastPulseUs != 0) && ((nowUs - lastPulseUs) <= (cadenceStaleMs_ * 1000u));

  if (cadenceFresh && cadenceRpm <= 0.0f) {
    const uint32_t pulseAgeUs = nowUs - lastPulseUs;
    if (pulseAgeUs > 0) {
      cadenceRpm = 60000000.0f /
                   (static_cast<float>(cadencePulsesPerRevolution_) * static_cast<float>(pulseAgeUs));
    }
  }

  if (!cadenceFresh || !std::isfinite(cadenceRpm) || cadenceRpm < 0.0f) {
    cadenceRpm = 0.0f;
  }

  const int adcRawSigned = analogRead(static_cast<uint8_t>(torqueAdcPin_));
  const uint16_t adcRaw = static_cast<uint16_t>(adcRawSigned < 0 ? 0 : adcRawSigned);
  const float sensorVoltage = sensorVoltageFromAdc_(adcRaw);
  const float dividerScale = (AppConfig::kPedalDividerR1Ohm + AppConfig::kPedalDividerR2Ohm) /
                             AppConfig::kPedalDividerR2Ohm;
  const float gpioVoltage = sensorVoltage / dividerScale;

  float rawTorqueNm = torqueFromSensorVoltage_(sensorVoltage, zeroOffsetSnapshot);
  if (!std::isfinite(rawTorqueNm) || rawTorqueNm < 0.0f) {
    rawTorqueNm = 0.0f;
  }

  if (!fastFilterInitialized_) {
    fastFilteredTorqueNm_ = rawTorqueNm;
    fastFilterInitialized_ = true;
  } else {
    const float alpha = clamp01_(configSnapshot.fastFilterAlpha);
    fastFilteredTorqueNm_ += alpha * (rawTorqueNm - fastFilteredTorqueNm_);
  }

  const bool highCadenceAveraging = cadenceFresh && cadenceRpm >= configSnapshot.cadenceAverageRpm;
  float filteredTorqueNm = fastFilteredTorqueNm_;

  if (highCadenceAveraging) {
    if (deltaPulses > 0) {
      rotationTorqueAccumulatorNm_ += rawTorqueNm * static_cast<float>(deltaPulses);
      rotationPulseAccumulator_ += deltaPulses;
    }

    if (rotationPulseAccumulator_ >= cadencePulsesPerRevolution_) {
      rotationAverageTorqueNm_ =
          rotationTorqueAccumulatorNm_ / static_cast<float>(rotationPulseAccumulator_);
      rotationAverageValid_ = true;
      rotationTorqueAccumulatorNm_ = 0.0f;
      rotationPulseAccumulator_ = 0;
    }

    if (rotationAverageValid_) {
      filteredTorqueNm = rotationAverageTorqueNm_;
    }
  } else {
    rotationTorqueAccumulatorNm_ = 0.0f;
    rotationPulseAccumulator_ = 0;
    rotationAverageValid_ = false;
  }

  bool failSafeActive = false;
  if (!std::isfinite(sensorVoltage) || sensorVoltage < 0.0f || sensorVoltage > 6.0f) {
    failSafeActive = true;
  }
  if (controlLate) {
    failSafeActive = true;
  }

  const bool cadenceActive = cadenceFresh && cadenceRpm >= configSnapshot.cadenceDisableRpm;
  const bool vescReady = vescCan_ && vescCan_->isReady();

  float rampFactor = 0.0f;
  if (configSnapshot.torqueRampEndNm > configSnapshot.torqueRampStartNm) {
    rampFactor = (filteredTorqueNm - configSnapshot.torqueRampStartNm) /
                 (configSnapshot.torqueRampEndNm - configSnapshot.torqueRampStartNm);
  } else {
    rampFactor = filteredTorqueNm > configSnapshot.torqueRampStartNm ? 1.0f : 0.0f;
  }
  rampFactor = clamp01_(rampFactor);

  const bool assistEnabled = cadenceActive && !failSafeActive && vescReady;

  float motorTorqueCmdNm = 0.0f;
  float motorCurrentCmdA = 0.0f;
  if (assistEnabled) {
    const float rampedHumanTorqueNm = filteredTorqueNm * rampFactor;
    motorTorqueCmdNm = rampedHumanTorqueNm * configSnapshot.assistRatio;

    const float torquePerAmp = torqueConstantOutputNmPerA_();
    if (!(torquePerAmp > 0.0f) || !std::isfinite(torquePerAmp)) {
      failSafeActive = true;
      motorTorqueCmdNm = 0.0f;
      motorCurrentCmdA = 0.0f;
    } else {
      motorCurrentCmdA = motorTorqueCmdNm / torquePerAmp;
      if (!std::isfinite(motorCurrentCmdA) || motorCurrentCmdA < 0.0f) {
        failSafeActive = true;
        motorTorqueCmdNm = 0.0f;
        motorCurrentCmdA = 0.0f;
      }
    }
  }

  if (failSafeActive || !cadenceActive) {
    motorTorqueCmdNm = 0.0f;
    motorCurrentCmdA = 0.0f;
  }

  if (motorCurrentCmdA > configSnapshot.maxCurrentA) {
    motorCurrentCmdA = configSnapshot.maxCurrentA;
  }

  bool canWriteOk = false;
  if (vescCan_) {
    canWriteOk = vescCan_->setCurrent(motorCurrentCmdA);
  }

  if (!canWriteOk && vescReady) {
    failSafeActive = true;
  }

  bool shouldPersist = false;
  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    lastStatus_.initialized = initialized_;
    lastStatus_.cadenceFresh = cadenceFresh;
    lastStatus_.cadenceActive = cadenceActive;
    lastStatus_.assistEnabled = assistEnabled && canWriteOk;
    lastStatus_.highCadenceAveraging = highCadenceAveraging;
    lastStatus_.failSafeActive = failSafeActive;
    lastStatus_.vescReady = vescReady;
    lastStatus_.controlLate = controlLate;
    lastStatus_.lastCanWriteOk = canWriteOk;
    lastStatus_.updatedAtMs = nowMs;
    lastStatus_.lastCadencePulseMs =
        lastPulseUs == 0 ? 0 : (nowMs - ((nowUs - lastPulseUs) / 1000u));
    lastStatus_.lastCommandMs = nowMs;
    lastStatus_.cadencePulsesTotal = pulseCount;
    lastStatus_.torqueAdcRaw = adcRaw;
    lastStatus_.cadenceRpm = cadenceRpm;
    lastStatus_.torqueRawNm = rawTorqueNm;
    lastStatus_.torqueFilteredNm = filteredTorqueNm;
    lastStatus_.torqueRotationAvgNm = rotationAverageTorqueNm_;
    lastStatus_.torqueZeroOffsetV = torqueZeroOffsetV_;
    lastStatus_.gpioVoltageV = gpioVoltage;
    lastStatus_.sensorVoltageV = sensorVoltage;
    lastStatus_.rampFactor = rampFactor;
    lastStatus_.assistRatio = configSnapshot.assistRatio;
    lastStatus_.motorTorqueCmdNm = motorTorqueCmdNm;
    lastStatus_.motorCurrentCmdA = motorCurrentCmdA;

    shouldPersist = dirty_ && AppConfig::kPedalPersistIntervalMs > 0 &&
                    elapsedMs_(nowMs, lastPersistMs_) >= AppConfig::kPedalPersistIntervalMs;
    xSemaphoreGive(stateMutex_);
  }

  if (shouldPersist) {
    persistNow_();
  }
}

PedalAssistManager::Status PedalAssistManager::status() const {
  Status copy;
  if (!stateMutex_) {
    return copy;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return copy;
  }

  copy = lastStatus_;
  copy.torqueZeroOffsetV = torqueZeroOffsetV_;

  xSemaphoreGive(stateMutex_);
  return copy;
}

PedalAssistManager::Config PedalAssistManager::config() const {
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

bool PedalAssistManager::setConfig(const Config& config, String* errorMessage) {
  if (!validateConfig_(config, errorMessage)) {
    return false;
  }

  if (!stateMutex_) {
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

bool PedalAssistManager::calibrateZero(String* errorMessage) {
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

  const float sensorVoltage = lastStatus_.sensorVoltageV;
  if (!std::isfinite(sensorVoltage) || sensorVoltage < 0.0f) {
    xSemaphoreGive(stateMutex_);
    if (errorMessage) {
      *errorMessage = "invalid_sensor_voltage";
    }
    return false;
  }

  torqueZeroOffsetV_ = sensorVoltage;
  dirty_ = true;

  xSemaphoreGive(stateMutex_);

  if (framStorage_ && framStorage_->isReady() && !persistNow_() && errorMessage) {
    *errorMessage = "calibrated_but_persist_failed";
  }

  return true;
}

void IRAM_ATTR PedalAssistManager::onCadencePulseIsr_() {
  PedalAssistManager* self = isrOwner_;
  if (!self) {
    return;
  }

  const uint32_t nowUs = micros();

  portENTER_CRITICAL_ISR(&self->cadenceMux_);
  const uint32_t sinceLast = nowUs - self->cadenceLastAcceptedPulseUsIsr_;
  if (self->cadenceLastAcceptedPulseUsIsr_ == 0 || sinceLast >= AppConfig::kPedalCadenceDebounceUs) {
    ++self->cadencePulseCountIsr_;
    self->cadenceLastPulseUsIsr_ = nowUs;
    self->cadenceLastAcceptedPulseUsIsr_ = nowUs;
  }
  portEXIT_CRITICAL_ISR(&self->cadenceMux_);
}

bool PedalAssistManager::validateConfig_(const Config& config, String* errorMessage) const {
  if (!(config.assistRatio >= AppConfig::kPedalAssistRatioMin &&
        config.assistRatio <= AppConfig::kPedalAssistRatioMax)) {
    if (errorMessage) {
      *errorMessage = "assist_ratio_out_of_range";
    }
    return false;
  }

  if (!(config.torqueRampStartNm >= AppConfig::kPedalTorqueRampMinNm &&
        config.torqueRampStartNm < AppConfig::kPedalTorqueRampMaxNm)) {
    if (errorMessage) {
      *errorMessage = "torque_ramp_start_out_of_range";
    }
    return false;
  }

  if (!(config.torqueRampEndNm > config.torqueRampStartNm &&
        config.torqueRampEndNm <= AppConfig::kPedalTorqueRampMaxNm)) {
    if (errorMessage) {
      *errorMessage = "torque_ramp_end_out_of_range";
    }
    return false;
  }

  if (!(config.cadenceDisableRpm >= AppConfig::kPedalCadenceDisableMinRpm &&
        config.cadenceDisableRpm <= AppConfig::kPedalCadenceDisableMaxRpm)) {
    if (errorMessage) {
      *errorMessage = "cadence_disable_out_of_range";
    }
    return false;
  }

  if (!(config.cadenceAverageRpm >= AppConfig::kPedalCadenceAverageMinRpm &&
        config.cadenceAverageRpm <= AppConfig::kPedalCadenceAverageMaxRpm &&
        config.cadenceAverageRpm >= config.cadenceDisableRpm)) {
    if (errorMessage) {
      *errorMessage = "cadence_average_out_of_range";
    }
    return false;
  }

  if (!(config.fastFilterAlpha >= AppConfig::kPedalFastFilterAlphaMin &&
        config.fastFilterAlpha <= AppConfig::kPedalFastFilterAlphaMax)) {
    if (errorMessage) {
      *errorMessage = "fast_filter_alpha_out_of_range";
    }
    return false;
  }

  if (!(config.maxCurrentA > 0.0f && config.maxCurrentA <= AppConfig::kPedalCurrentLimitHardMaxA)) {
    if (errorMessage) {
      *errorMessage = "max_current_out_of_range";
    }
    return false;
  }

  return true;
}

bool PedalAssistManager::loadPersistedState_() {
  if (!framStorage_ || !framStorage_->isReady()) {
    return false;
  }

  PersistedState state = {};
  if (!framStorage_->read(AppConfig::kPedalFramConfigAddress,
                          reinterpret_cast<uint8_t*>(&state),
                          sizeof(state))) {
    return false;
  }

  const uint32_t expectedChecksum =
      checksum_(reinterpret_cast<const uint8_t*>(&state), sizeof(state) - sizeof(state.checksum));

  if (state.magic != kPersistMagic || state.version != kPersistVersion ||
      state.checksum != expectedChecksum) {
    return false;
  }

  Config loadedConfig;
  loadedConfig.assistRatio = state.assistRatio;
  loadedConfig.torqueRampStartNm = state.torqueRampStartNm;
  loadedConfig.torqueRampEndNm = state.torqueRampEndNm;
  loadedConfig.cadenceDisableRpm = state.cadenceDisableRpm;
  loadedConfig.cadenceAverageRpm = state.cadenceAverageRpm;
  loadedConfig.fastFilterAlpha = state.fastFilterAlpha;
  loadedConfig.maxCurrentA = state.maxCurrentA;

  String error;
  if (!validateConfig_(loadedConfig, &error)) {
    Serial.printf("[PEDAL] Ignoring invalid persisted config: %s\n", error.c_str());
    return false;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return false;
  }

  config_ = loadedConfig;
  torqueZeroOffsetV_ = state.torqueZeroOffsetV;

  xSemaphoreGive(stateMutex_);

  return true;
}

bool PedalAssistManager::persistNow_() {
  if (!framStorage_ || !framStorage_->isReady()) {
    return false;
  }

  PersistedState state = {};

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return false;
  }

  state = makePersistedState_();

  xSemaphoreGive(stateMutex_);

  const bool written = framStorage_->write(AppConfig::kPedalFramConfigAddress,
                                           reinterpret_cast<const uint8_t*>(&state),
                                           sizeof(state));

  if (written && xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
    lastPersistMs_ = millis();
    dirty_ = false;
    xSemaphoreGive(stateMutex_);
  }

  return written;
}

PedalAssistManager::PersistedState PedalAssistManager::makePersistedState_() const {
  PersistedState state = {};
  state.magic = kPersistMagic;
  state.version = kPersistVersion;
  state.assistRatio = config_.assistRatio;
  state.torqueRampStartNm = config_.torqueRampStartNm;
  state.torqueRampEndNm = config_.torqueRampEndNm;
  state.cadenceDisableRpm = config_.cadenceDisableRpm;
  state.cadenceAverageRpm = config_.cadenceAverageRpm;
  state.fastFilterAlpha = config_.fastFilterAlpha;
  state.maxCurrentA = config_.maxCurrentA;
  state.torqueZeroOffsetV = torqueZeroOffsetV_;
  state.checksum = checksum_(reinterpret_cast<const uint8_t*>(&state), sizeof(state) - sizeof(state.checksum));
  return state;
}

float PedalAssistManager::sensorVoltageFromAdc_(uint16_t adcRaw) const {
  const uint32_t maxAdc = (1u << AppConfig::kPedalAdcResolutionBits) - 1u;
  const float gpioVoltage = (static_cast<float>(adcRaw) * AppConfig::kPedalAdcReferenceVoltage) /
                            static_cast<float>(maxAdc);

  const float dividerScale = (AppConfig::kPedalDividerR1Ohm + AppConfig::kPedalDividerR2Ohm) /
                             AppConfig::kPedalDividerR2Ohm;
  return gpioVoltage * dividerScale;
}

float PedalAssistManager::torqueFromSensorVoltage_(float sensorVoltage, float zeroOffsetSensorVoltage) const {
  const float deltaV = sensorVoltage - zeroOffsetSensorVoltage;
  if (deltaV <= 0.0f) {
    return 0.0f;
  }

  const float kgf = deltaV / AppConfig::kPedalSensorVoltsPerKgf;
  const float forceN = kgf * AppConfig::kPedalGravityMps2;
  return forceN * AppConfig::kPedalCrankLengthMeters;
}

float PedalAssistManager::torqueConstantOutputNmPerA_() const {
  const float motorNmPerAmp =
      1.5f * static_cast<float>(AppConfig::kPedalMotorPolePairs) * AppConfig::kPedalFluxLinkageWb;
  return motorNmPerAmp * AppConfig::kPedalGearRatio;
}

float PedalAssistManager::clamp01_(float value) const {
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

uint32_t PedalAssistManager::checksum_(const uint8_t* data, size_t len) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < len; ++i) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

uint32_t PedalAssistManager::elapsedMs_(uint32_t nowMs, uint32_t startMs) {
  return nowMs - startMs;
}
