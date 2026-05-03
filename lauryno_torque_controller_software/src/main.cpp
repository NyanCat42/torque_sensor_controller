#include <Arduino.h>
#include <ESPmDNS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_config.h"
#include "fram_storage.h"
#include "network_manager.h"
#include "odometer_manager.h"
#include "ota_http_server.h"
#include "pedal_assist_manager.h"
#include "vesc_can.h"
#include "vesc_tcp_bridge.h"

namespace {

NetworkManager g_networkManager(
    AppConfig::kStaSsid,
    AppConfig::kStaPassword,
    AppConfig::kApSsidPrefix,
    AppConfig::kApPassword,
    AppConfig::kStaConnectTimeoutMs);

VescCan g_vescCan(AppConfig::kCanTxPin,
          AppConfig::kCanRxPin,
          AppConfig::kCanBaudRate,
          AppConfig::kCanLocalControllerId,
          AppConfig::kCanTargetControllerId);

VescTcpBridge g_vescTcpBridge(
  g_vescCan, AppConfig::kVescToolTcpPort, AppConfig::kVescToolBridgeEnabledByDefault);

FramStorage g_framStorage(
  AppConfig::kOdometerI2cSdaPin,
  AppConfig::kOdometerI2cSclPin,
  AppConfig::kOdometerFramBaseAddress);

OdometerManager::Config g_odometerDefaults = []() {
  OdometerManager::Config config;
  config.wheelDiameterMm = AppConfig::kOdometerWheelDiameterMm;
  config.debounceMs = AppConfig::kOdometerDebounceMs;
  config.persistIntervalMs = AppConfig::kOdometerPersistIntervalMs;
  config.maxSpeedKmh = AppConfig::kOdometerMaxSpeedKmh;
  return config;
}();

OdometerManager g_odometerManager(
  AppConfig::kOdometerHallPin,
  AppConfig::kOdometerWheelMagnets,
  AppConfig::kOdometerSpeedStaleMs,
  AppConfig::kOdometerHardSpeedLimitKmh,
  &g_framStorage,
  g_odometerDefaults);

PedalAssistManager::Config g_pedalDefaults = []() {
  PedalAssistManager::Config config;
  config.assistRatio = AppConfig::kPedalDefaultAssistRatio;
  config.torqueRampStartNm = AppConfig::kPedalDefaultTorqueRampStartNm;
  config.torqueRampEndNm = AppConfig::kPedalDefaultTorqueRampEndNm;
  config.cadenceDisableRpm = AppConfig::kPedalDefaultCadenceDisableRpm;
  config.cadenceAverageRpm = AppConfig::kPedalDefaultCadenceAverageRpm;
  config.fastFilterAlpha = AppConfig::kPedalDefaultFastFilterAlpha;
  config.maxCurrentA = AppConfig::kPedalDefaultMaxCurrentA;
  return config;
}();

PedalAssistManager g_pedalAssistManager(
  AppConfig::kPedalTorqueAdcPin,
  AppConfig::kPedalCadencePin,
  AppConfig::kPedalCadencePulsesPerRevolution,
  AppConfig::kPedalCadenceStaleMs,
  AppConfig::kPedalControlPeriodMs,
  AppConfig::kPedalControlLateFactor,
  &g_vescCan,
  &g_framStorage,
  g_pedalDefaults);

TaskHandle_t g_pedalTaskHandle = nullptr;
TaskHandle_t g_commTaskHandle = nullptr;

constexpr uint16_t kCommTaskStackWords = 4096;
constexpr uint8_t kCommTaskPriority = 1;
constexpr uint32_t kCommTaskPeriodMs = 2;

void PedalAssistTask(void* parameter) {
  (void)parameter;
  const TickType_t periodTicks = pdMS_TO_TICKS(AppConfig::kPedalControlPeriodMs);
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    g_pedalAssistManager.controlTick();
    vTaskDelayUntil(&lastWake, periodTicks);
  }
}

OtaHttpServer g_otaServer(
  AppConfig::kOtaHttpPort,
  &g_networkManager,
  &g_vescCan,
  &g_vescTcpBridge,
  &g_odometerManager,
  &g_pedalAssistManager);

void CommunicationTask(void* parameter) {
  (void)parameter;
  const TickType_t periodTicks = pdMS_TO_TICKS(kCommTaskPeriodMs);
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    g_vescTcpBridge.update();
    g_otaServer.handleClient();
    vTaskDelayUntil(&lastWake, periodTicks);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  const NetworkMode mode = g_networkManager.begin();
  const String modeLabel = (mode == NetworkMode::Station) ? "STA" : "AP";

  const bool mdnsStarted = MDNS.begin(AppConfig::kMdnsHostname);
  if (mdnsStarted) {
    MDNS.addService("http", "tcp", AppConfig::kOtaHttpPort);
    MDNS.addService("vesc", "tcp", AppConfig::kVescToolTcpPort);
    Serial.printf("[APP] mDNS ready: http://%s.local/\n", AppConfig::kMdnsHostname);
  } else {
    Serial.printf("[APP] mDNS failed to start for host: %s.local\n", AppConfig::kMdnsHostname);
  }

  const bool canReady = g_vescCan.begin();
  if (!canReady) {
    Serial.println("[APP] VESC CAN failed to initialize.");
  }

  g_vescTcpBridge.begin();

  const bool framReady = g_framStorage.begin();
  Serial.printf("[APP] FRAM ready: %s\n", framReady ? "yes" : "no");

  const bool odometerReady = g_odometerManager.begin();
  Serial.printf("[APP] Odometer ready: %s\n", odometerReady ? "yes" : "no");

  const bool pedalReady = g_pedalAssistManager.begin();
  Serial.printf("[APP] Pedal assist ready: %s\n", pedalReady ? "yes" : "no");

  const BaseType_t taskOk = xTaskCreatePinnedToCore(
      PedalAssistTask,
      "PedalAssist",
      AppConfig::kPedalTaskStackWords,
      nullptr,
      AppConfig::kPedalTaskPriority,
      &g_pedalTaskHandle,
      1);
  Serial.printf("[APP] Pedal control task: %s\n", taskOk == pdPASS ? "started" : "failed");

  const IPAddress deviceIp = g_networkManager.ipAddress();
  g_otaServer.begin(modeLabel, deviceIp);

  const BaseType_t commTaskOk = xTaskCreatePinnedToCore(
      CommunicationTask,
      "CommTask",
      kCommTaskStackWords,
      nullptr,
      kCommTaskPriority,
      &g_commTaskHandle,
      0);
  Serial.printf("[APP] Comm task (TCP+HTTP): %s\n", commTaskOk == pdPASS ? "started" : "failed");

  Serial.printf("[APP] Device IP: %s\n", deviceIp.toString().c_str());
  Serial.printf("[APP] OTA page: http://%s/\n", deviceIp.toString().c_str());
  Serial.printf("[APP] VESC TCP bridge: %s:%u\n",
                deviceIp.toString().c_str(),
                static_cast<unsigned>(AppConfig::kVescToolTcpPort));
  if (mode == NetworkMode::AccessPoint) {
    Serial.printf("[APP] AP SSID: %s\n", g_networkManager.accessPointSsid().c_str());
  }
}

void loop() {
  g_vescCan.poll();
  g_odometerManager.update();
}