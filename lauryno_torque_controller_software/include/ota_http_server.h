#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <WebServer.h>

class VescCan;
class VescTcpBridge;
class OdometerManager;
class PedalAssistManager;

class OtaHttpServer {
 public:
  OtaHttpServer(uint16_t port,
                VescCan* vescCan,
                VescTcpBridge* vescTcpBridge,
                OdometerManager* odometerManager,
                PedalAssistManager* pedalAssistManager);

  void begin(const String& networkModeLabel, const IPAddress& activeIp);
  void handleClient();

 private:
  void registerRoutes_();
  String pageHtml_() const;

  WebServer server_;
  String networkModeLabel_;
  IPAddress activeIp_;
  VescCan* vescCan_;
  VescTcpBridge* vescTcpBridge_;
  OdometerManager* odometerManager_;
  PedalAssistManager* pedalAssistManager_;
};
