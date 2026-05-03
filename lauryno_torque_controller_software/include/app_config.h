#pragma once

#include <Arduino.h>
#include <driver/gpio.h>

namespace AppConfig {

#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID "YOUR_HOME_WIFI_SSID"
#endif

#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD "YOUR_HOME_WIFI_PASSWORD"
#endif

#ifndef WIFI_AP_SSID_PREFIX
#define WIFI_AP_SSID_PREFIX "TorqueCtrl-Setup"
#endif

#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD "torque123"
#endif

constexpr char kStaSsid[] = WIFI_STA_SSID;
constexpr char kStaPassword[] = WIFI_STA_PASSWORD;

constexpr char kApSsidPrefix[] = WIFI_AP_SSID_PREFIX;
constexpr char kApPassword[] = WIFI_AP_PASSWORD;
constexpr char kMdnsHostname[] = "lauryno-dviratis";

constexpr uint32_t kStaConnectTimeoutMs = 15000;
constexpr uint16_t kOtaHttpPort = 80;

constexpr gpio_num_t kCanTxPin = GPIO_NUM_15;
constexpr gpio_num_t kCanRxPin = GPIO_NUM_7;
constexpr uint32_t kCanBaudRate = 500000;
constexpr uint8_t kCanLocalControllerId = 42;
constexpr uint8_t kCanTargetControllerId = 69;

constexpr uint16_t kVescToolTcpPort = 65102;
constexpr bool kVescToolBridgeEnabledByDefault = false;

constexpr gpio_num_t kOdometerHallPin = GPIO_NUM_9;
constexpr gpio_num_t kOdometerI2cSdaPin = GPIO_NUM_6;
constexpr gpio_num_t kOdometerI2cSclPin = GPIO_NUM_5;
constexpr uint8_t kOdometerFramBaseAddress = 0x50;

constexpr uint16_t kOdometerWheelDiameterMm = 659;
constexpr uint8_t kOdometerWheelMagnets = 1;
constexpr uint16_t kOdometerDebounceMs = 35;
constexpr uint32_t kOdometerSpeedStaleMs = 2500;
constexpr uint32_t kOdometerPersistIntervalMs = 30000;
constexpr float kOdometerMaxSpeedKmh = 50.0f;
constexpr float kOdometerHardSpeedLimitKmh = 50.0f;

constexpr uint16_t kOdometerMinWheelDiameterMm = 200;
constexpr uint16_t kOdometerMaxWheelDiameterMm = 900;
constexpr uint16_t kOdometerMinDebounceMs = 1;
constexpr uint16_t kOdometerMaxDebounceMs = 250;
constexpr uint32_t kOdometerMinPersistIntervalMs = 5000;
constexpr uint32_t kOdometerMaxPersistIntervalMs = 300000;

constexpr gpio_num_t kPedalTorqueAdcPin = GPIO_NUM_1;
constexpr gpio_num_t kPedalCadencePin = GPIO_NUM_2;
constexpr uint8_t kPedalCadencePulsesPerRevolution = 32;
constexpr uint32_t kPedalCadenceStaleMs = 1200;
constexpr uint32_t kPedalCadenceDebounceUs = 500;

constexpr uint8_t kPedalAdcResolutionBits = 12;
constexpr float kPedalAdcReferenceVoltage = 3.3f;
constexpr float kPedalDividerR1Ohm = 10000.0f;
constexpr float kPedalDividerR2Ohm = 15000.0f;
constexpr float kPedalSensorVoltsPerKgf = 0.035f;
constexpr float kPedalCrankLengthMeters = 0.175f;
constexpr float kPedalGravityMps2 = 9.80665f;

constexpr float kPedalFluxLinkageWb = 0.013962f;
constexpr uint8_t kPedalMotorPolePairs = 20;
constexpr float kPedalGearRatio = 5.0f;

constexpr float kPedalDefaultAssistRatio = 1.0f;
constexpr float kPedalDefaultTorqueRampStartNm = 20.0f;
constexpr float kPedalDefaultTorqueRampEndNm = 30.0f;
constexpr float kPedalDefaultCadenceDisableRpm = 8.0f;
constexpr float kPedalDefaultCadenceAverageRpm = 55.0f;
constexpr float kPedalDefaultFastFilterAlpha = 0.70f;
constexpr float kPedalDefaultMaxCurrentA = 70.0f;

constexpr float kPedalAssistRatioMin = 0.0f;
constexpr float kPedalAssistRatioMax = 3.0f;
constexpr float kPedalTorqueRampMinNm = 0.0f;
constexpr float kPedalTorqueRampMaxNm = 180.0f;
constexpr float kPedalCadenceDisableMinRpm = 0.0f;
constexpr float kPedalCadenceDisableMaxRpm = 60.0f;
constexpr float kPedalCadenceAverageMinRpm = 20.0f;
constexpr float kPedalCadenceAverageMaxRpm = 180.0f;
constexpr float kPedalFastFilterAlphaMin = 0.05f;
constexpr float kPedalFastFilterAlphaMax = 1.0f;
constexpr float kPedalCurrentLimitHardMaxA = 70.0f;

constexpr uint32_t kPedalControlPeriodMs = 10;
constexpr uint32_t kPedalControlLateFactor = 4;
constexpr uint8_t kPedalTaskPriority = 4;
constexpr uint16_t kPedalTaskStackWords = 4096;

constexpr uint16_t kPedalFramConfigAddress = 256;
constexpr uint32_t kPedalPersistIntervalMs = 30000;

}  // namespace AppConfig
