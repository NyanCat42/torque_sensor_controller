#include "vesc_tcp_bridge.h"

VescTcpBridge::VescTcpBridge(VescCan& can,
                             uint16_t tcpPort,
                             bool enabledByDefault,
                             uint8_t maxClientRxChunk)
    : can_(can),
      server_(tcpPort),
      tcpPort_(tcpPort),
      enabled_(enabledByDefault),
      maxClientRxChunk_(maxClientRxChunk),
      stateMutex_(nullptr) {
  parser_.setCallback(&VescTcpBridge::onTcpPacketStatic_, this);
}

void VescTcpBridge::begin() {
  if (!stateMutex_) {
    stateMutex_ = xSemaphoreCreateMutex();
    if (!stateMutex_) {
      Serial.println("[VESC TCP] Failed to create mutex.");
      return;
    }
  }

  server_.begin();
  server_.setNoDelay(true);

  can_.setPacketHandler(&VescTcpBridge::onCanPacketStatic_, this);

  Serial.printf("[VESC TCP] Bridge server listening on port %u (%s)\n",
                static_cast<unsigned>(tcpPort_),
                enabled_ ? "enabled" : "disabled");
}

void VescTcpBridge::update() {
  if (!stateMutex_) {
    return;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }

  if (!enabled_) {
    if (client_) {
      client_.stop();
    }
    xSemaphoreGive(stateMutex_);
    return;
  }

  acceptClientIfNeeded_();

  if (!isClientConnectedLocked_()) {
    xSemaphoreGive(stateMutex_);
    return;
  }

  if (maxClientRxChunk_ == 0) {
    xSemaphoreGive(stateMutex_);
    return;
  }

  uint8_t rxBuffer[255];
  const int chunkCap = (maxClientRxChunk_ > sizeof(rxBuffer)) ? sizeof(rxBuffer) : maxClientRxChunk_;

  while (client_.connected() && client_.available() > 0) {
    const int readLen = client_.read(rxBuffer, chunkCap);
    if (readLen <= 0) {
      break;
    }
    parser_.processBytes(rxBuffer, static_cast<size_t>(readLen));
  }

  xSemaphoreGive(stateMutex_);
}

void VescTcpBridge::setEnabled(bool enabled) {
  if (!stateMutex_) {
    return;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return;
  }

  if (enabled_ == enabled) {
    xSemaphoreGive(stateMutex_);
    return;
  }

  enabled_ = enabled;
  parser_.reset();

  if (!enabled_ && client_) {
    client_.stop();
  }

  xSemaphoreGive(stateMutex_);

  Serial.printf("[VESC TCP] Bridge %s\n", enabled_ ? "enabled" : "disabled");
}

bool VescTcpBridge::isEnabled() const {
  if (!stateMutex_) {
    return enabled_;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return enabled_;
  }

  const bool enabled = enabled_;
  xSemaphoreGive(stateMutex_);
  return enabled;
}

bool VescTcpBridge::hasClient() {
  if (!stateMutex_) {
    return false;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return false;
  }

  const bool connected = isClientConnectedLocked_();
  xSemaphoreGive(stateMutex_);
  return connected;
}

uint16_t VescTcpBridge::port() const {
  return tcpPort_;
}

void VescTcpBridge::onTcpPacketStatic_(const uint8_t* payload, size_t len, void* userData) {
  if (!userData) {
    return;
  }
  static_cast<VescTcpBridge*>(userData)->onTcpPacket_(payload, len);
}

void VescTcpBridge::onCanPacketStatic_(const uint8_t* payload, size_t len, void* userData) {
  if (!userData) {
    return;
  }
  static_cast<VescTcpBridge*>(userData)->onCanPacket_(payload, len);
}

void VescTcpBridge::onTcpPacket_(const uint8_t* payload, size_t len) {
  if (!enabled_ || !payload || len == 0) {
    return;
  }

  const bool sent = can_.sendPassthroughPacket(payload, len, 0);
  if (!sent) {
    Serial.println("[VESC TCP] Failed to forward packet to CAN.");
  }
}

void VescTcpBridge::onCanPacket_(const uint8_t* payload, size_t len) {
  if (!payload || len == 0) {
    return;
  }

  uint8_t frame[VescPacket::kMaxFrameLen];
  const size_t frameLen = VescPacket::EncodeFrame(payload, len, frame, sizeof(frame));
  if (frameLen == 0) {
    return;
  }

  if (!stateMutex_) {
    return;
  }

  if (xSemaphoreTake(stateMutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }

  if (!enabled_ || !isClientConnectedLocked_()) {
    xSemaphoreGive(stateMutex_);
    return;
  }

  const size_t written = client_.write(frame, frameLen);
  xSemaphoreGive(stateMutex_);
  if (written != frameLen) {
    Serial.println("[VESC TCP] Short write to TCP client.");
  }
}

void VescTcpBridge::acceptClientIfNeeded_() {
  if (isClientConnectedLocked_()) {
    return;
  }

  WiFiClient incoming = server_.available();
  if (!incoming) {
    return;
  }

  incoming.setNoDelay(true);

  if (client_) {
    client_.stop();
  }

  client_ = incoming;
  parser_.reset();

  Serial.printf("[VESC TCP] Client connected from %s\n", client_.remoteIP().toString().c_str());
}

bool VescTcpBridge::isClientConnectedLocked_() {
  return client_ && client_.connected();
}
