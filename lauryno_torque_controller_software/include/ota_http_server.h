#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <WebServer.h>

class VescCan;
class VescTcpBridge;
class OdometerManager;
class PedalAssistManager;
class NetworkManager;

class OtaHttpServer {
 public:
  OtaHttpServer(uint16_t port,
                NetworkManager* networkManager,
                VescCan* vescCan,
                VescTcpBridge* vescTcpBridge,
                OdometerManager* odometerManager,
                PedalAssistManager* pedalAssistManager);

  void begin(const String& networkModeLabel, const IPAddress& activeIp);
  void handleClient();

 private:
  static constexpr uint32_t kApSwitchDelayMs = 300;

  void registerRoutes_();
  void processDeferredActions_();
  String pageHtml_() const;

  WebServer server_;
  String networkModeLabel_;
  IPAddress activeIp_;
  NetworkManager* networkManager_;
  VescCan* vescCan_;
  VescTcpBridge* vescTcpBridge_;
  OdometerManager* odometerManager_;
  PedalAssistManager* pedalAssistManager_;
  bool apSwitchPending_;
  uint32_t apSwitchAtMs_;
};
