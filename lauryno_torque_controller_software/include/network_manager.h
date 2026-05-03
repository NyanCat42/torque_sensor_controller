#pragma once

#include <Arduino.h>
#include <WiFi.h>

enum class NetworkMode {
  Station,
  AccessPoint
};

class NetworkManager {
 public:
  NetworkManager(const char* staSsid,
                 const char* staPassword,
                 const char* apSsidPrefix,
                 const char* apPassword,
                 uint32_t staConnectTimeoutMs);

  NetworkMode begin();
  bool switchToAccessPoint();

  NetworkMode mode() const;
  IPAddress ipAddress() const;
  String accessPointSsid() const;

 private:
  bool connectToStation_();
  bool startFallbackAp_();

  const char* staSsid_;
  const char* staPassword_;
  const char* apSsidPrefix_;
  const char* apPassword_;
  uint32_t staConnectTimeoutMs_;

  NetworkMode mode_;
  String apSsid_;
};
