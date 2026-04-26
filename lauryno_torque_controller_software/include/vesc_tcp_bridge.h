#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "vesc_can.h"
#include "vesc_packet_codec.h"

class VescTcpBridge {
 public:
  VescTcpBridge(VescCan& can,
                uint16_t tcpPort,
                bool enabledByDefault = false,
                uint8_t maxClientRxChunk = 128);

  void begin();
  void update();

  void setEnabled(bool enabled);
  bool isEnabled() const;

  bool hasClient();
  uint16_t port() const;

 private:
  static void onTcpPacketStatic_(const uint8_t* payload, size_t len, void* userData);
  static void onCanPacketStatic_(const uint8_t* payload, size_t len, void* userData);

  void onTcpPacket_(const uint8_t* payload, size_t len);
  void onCanPacket_(const uint8_t* payload, size_t len);

  void acceptClientIfNeeded_();
  bool isClientConnectedLocked_();

  VescCan& can_;
  WiFiServer server_;
  uint16_t tcpPort_;
  WiFiClient client_;
  bool enabled_;
  uint8_t maxClientRxChunk_;
  mutable SemaphoreHandle_t stateMutex_;

  VescPacket::Parser parser_;
};
