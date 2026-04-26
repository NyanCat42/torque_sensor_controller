#include "vesc_can.h"

#include "vesc_packet_codec.h"

#include <freertos/FreeRTOS.h>

namespace {

bool SetTwaiTiming(uint32_t baudRate, twai_timing_config_t* timing) {
  if (!timing) {
    return false;
  }

  switch (baudRate) {
    case 1000000:
      *timing = TWAI_TIMING_CONFIG_1MBITS();
      return true;
    case 500000:
      *timing = TWAI_TIMING_CONFIG_500KBITS();
      return true;
    case 250000:
      *timing = TWAI_TIMING_CONFIG_250KBITS();
      return true;
    case 125000:
      *timing = TWAI_TIMING_CONFIG_125KBITS();
      return true;
    default:
      return false;
  }
}

}  // namespace

VescCan::VescCan(gpio_num_t txPin,
                 gpio_num_t rxPin,
                 uint32_t baudRate,
                 uint8_t localControllerId,
                 uint8_t targetControllerId)
    : txPin_(txPin),
      rxPin_(rxPin),
      baudRate_(baudRate),
      localControllerId_(localControllerId),
      targetControllerId_(targetControllerId),
      ready_(false),
      passthroughRxOffset_(0),
      packetHandler_(nullptr),
      packetHandlerUserData_(nullptr),
      driverMutex_(nullptr),
      statusMux_(portMUX_INITIALIZER_UNLOCKED) {}

bool VescCan::begin() {
  if (ready_) {
    return true;
  }

  if (!driverMutex_) {
    driverMutex_ = xSemaphoreCreateMutex();
    if (!driverMutex_) {
      Serial.println("[VESC CAN] Failed to create driver mutex.");
      return false;
    }
  }

  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(txPin_, rxPin_, TWAI_MODE_NORMAL);
  general.tx_queue_len = 20;
  general.rx_queue_len = 20;

  twai_timing_config_t timing;
  if (!SetTwaiTiming(baudRate_, &timing)) {
    Serial.printf("[VESC CAN] Unsupported baud rate: %lu\n", static_cast<unsigned long>(baudRate_));
    return false;
  }

  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&general, &timing, &filter);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf("[VESC CAN] twai_driver_install failed: %d\n", static_cast<int>(err));
    return false;
  }

  err = twai_start();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf("[VESC CAN] twai_start failed: %d\n", static_cast<int>(err));
    return false;
  }

  ready_ = true;
  passthroughRxOffset_ = 0;
  Serial.printf("[VESC CAN] Ready on TX=%d RX=%d @ %lu bps (local id=%u target id=%u)\n",
                static_cast<int>(txPin_),
                static_cast<int>(rxPin_),
                static_cast<unsigned long>(baudRate_),
                static_cast<unsigned>(localControllerId_),
                static_cast<unsigned>(targetControllerId_));
  return true;
}

bool VescCan::isReady() const {
  return ready_;
}

void VescCan::poll() {
  if (!ready_) {
    return;
  }

  int processed = 0;
  while (processed < 64) {
    if (!driverMutex_ || xSemaphoreTake(driverMutex_, pdMS_TO_TICKS(1)) != pdTRUE) {
      break;
    }

    twai_message_t msg;
    const esp_err_t rxErr = twai_receive(&msg, 0);
    xSemaphoreGive(driverMutex_);

    if (rxErr != ESP_OK) {
      break;
    }

    processIncomingFrame_(msg);
    ++processed;
  }
}

bool VescCan::setCurrent(float currentA) {
  if (!ready_) {
    return false;
  }

  const int32_t scaled = static_cast<int32_t>(currentA * 1000.0f);
  uint8_t payload[4];
  payload[0] = static_cast<uint8_t>(scaled >> 24);
  payload[1] = static_cast<uint8_t>(scaled >> 16);
  payload[2] = static_cast<uint8_t>(scaled >> 8);
  payload[3] = static_cast<uint8_t>(scaled);
  return sendExtendedFrame_(targetControllerId_, CAN_PACKET_SET_CURRENT, payload, sizeof(payload));
}

bool VescCan::sendPassthroughPacket(const uint8_t* payload, size_t len, uint8_t sendMode) {
  if (!ready_ || !payload || len == 0 || len > kCanPassthroughBufferSize) {
    return false;
  }

  uint8_t canData[8];

  if (len <= 6) {
    canData[0] = localControllerId_;
    canData[1] = sendMode;
    for (size_t i = 0; i < len; ++i) {
      canData[2 + i] = payload[i];
    }
    return sendExtendedFrame_(targetControllerId_,
                              CAN_PACKET_PROCESS_SHORT_BUFFER,
                              canData,
                              static_cast<uint8_t>(len + 2));
  }

  size_t endA = 0;
  for (size_t offset = 0; offset < len; offset += 7) {
    if (offset > 255) {
      break;
    }

    endA = offset + 7;
    canData[0] = static_cast<uint8_t>(offset);

    uint8_t chunkLen = 7;
    if (offset + chunkLen > len) {
      chunkLen = static_cast<uint8_t>(len - offset);
    }

    for (uint8_t i = 0; i < chunkLen; ++i) {
      canData[1 + i] = payload[offset + i];
    }

    if (!sendExtendedFrame_(targetControllerId_,
                            CAN_PACKET_FILL_RX_BUFFER,
                            canData,
                            static_cast<uint8_t>(chunkLen + 1))) {
      return false;
    }
  }

  for (size_t offset = endA; offset < len; offset += 6) {
    canData[0] = static_cast<uint8_t>(offset >> 8);
    canData[1] = static_cast<uint8_t>(offset);

    uint8_t chunkLen = 6;
    if (offset + chunkLen > len) {
      chunkLen = static_cast<uint8_t>(len - offset);
    }

    for (uint8_t i = 0; i < chunkLen; ++i) {
      canData[2 + i] = payload[offset + i];
    }

    if (!sendExtendedFrame_(targetControllerId_,
                            CAN_PACKET_FILL_RX_BUFFER_LONG,
                            canData,
                            static_cast<uint8_t>(chunkLen + 2))) {
      return false;
    }
  }

  const uint16_t crc = VescPacket::Crc16(payload, len);
  canData[0] = localControllerId_;
  canData[1] = sendMode;
  canData[2] = static_cast<uint8_t>(len >> 8);
  canData[3] = static_cast<uint8_t>(len);
  canData[4] = static_cast<uint8_t>(crc >> 8);
  canData[5] = static_cast<uint8_t>(crc);

  return sendExtendedFrame_(
      targetControllerId_, CAN_PACKET_PROCESS_RX_BUFFER, canData, static_cast<uint8_t>(6));
}

void VescCan::setPacketHandler(PacketHandler handler, void* userData) {
  packetHandler_ = handler;
  packetHandlerUserData_ = userData;
}

void VescCan::setTargetControllerId(uint8_t id) {
  targetControllerId_ = id;
}

uint8_t VescCan::targetControllerId() const {
  return targetControllerId_;
}

uint8_t VescCan::localControllerId() const {
  return localControllerId_;
}

VescCan::Status1 VescCan::status1() const {
  Status1 copy;
  portENTER_CRITICAL(&statusMux_);
  copy = status1_;
  portEXIT_CRITICAL(&statusMux_);
  return copy;
}

bool VescCan::sendExtendedFrame_(uint8_t targetControllerId,
                                 uint8_t command,
                                 const uint8_t* data,
                                 uint8_t len) {
  if (len > 8) {
    return false;
  }

  twai_message_t msg = {};
  msg.extd = 1;
  msg.identifier = static_cast<uint32_t>(targetControllerId) | (static_cast<uint32_t>(command) << 8);
  msg.data_length_code = len;

  for (uint8_t i = 0; i < len; ++i) {
    msg.data[i] = data[i];
  }

  if (!driverMutex_ || xSemaphoreTake(driverMutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
    return false;
  }

  const esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(5));
  xSemaphoreGive(driverMutex_);
  return err == ESP_OK;
}

void VescCan::processIncomingFrame_(const twai_message_t& msg) {
  if (!msg.extd) {
    return;
  }

  const uint8_t endpointId = static_cast<uint8_t>(msg.identifier & 0xFF);
  const uint8_t command = static_cast<uint8_t>((msg.identifier >> 8) & 0xFF);

  switch (command) {
    case CAN_PACKET_FILL_RX_BUFFER:
      if (endpointId == localControllerId_) {
        handleFillRxBuffer_(msg.data, msg.data_length_code);
      }
      break;

    case CAN_PACKET_FILL_RX_BUFFER_LONG:
      if (endpointId == localControllerId_) {
        handleFillRxBufferLong_(msg.data, msg.data_length_code);
      }
      break;

    case CAN_PACKET_PROCESS_RX_BUFFER:
      if (endpointId == localControllerId_) {
        handleProcessRxBuffer_(msg.data, msg.data_length_code);
      }
      break;

    case CAN_PACKET_PROCESS_SHORT_BUFFER:
      if (endpointId == localControllerId_) {
        handleProcessShortBuffer_(msg.data, msg.data_length_code);
      }
      break;

    case CAN_PACKET_STATUS:
      if (endpointId == targetControllerId_) {
        handleStatus1_(msg.data, msg.data_length_code, endpointId);
      }
      break;

    default:
      break;
  }
}

void VescCan::handleFillRxBuffer_(const uint8_t* data, uint8_t len) {
  if (!data || len < 2) {
    return;
  }

  const size_t offset = data[0];
  const size_t dataLen = static_cast<size_t>(len - 1);

  if (offset == 0) {
    passthroughRxOffset_ = 0;
  }

  if (offset != passthroughRxOffset_ || (offset + dataLen) > kCanPassthroughBufferSize) {
    passthroughRxOffset_ = 0;
    return;
  }

  for (size_t i = 0; i < dataLen; ++i) {
    passthroughRxBuffer_[offset + i] = data[1 + i];
  }

  passthroughRxOffset_ = offset + dataLen;
}

void VescCan::handleFillRxBufferLong_(const uint8_t* data, uint8_t len) {
  if (!data || len < 3) {
    return;
  }

  const size_t offset = (static_cast<size_t>(data[0]) << 8) | static_cast<size_t>(data[1]);
  const size_t dataLen = static_cast<size_t>(len - 2);

  if (offset == 0) {
    passthroughRxOffset_ = 0;
  }

  if (offset != passthroughRxOffset_ || (offset + dataLen) > kCanPassthroughBufferSize) {
    passthroughRxOffset_ = 0;
    return;
  }

  for (size_t i = 0; i < dataLen; ++i) {
    passthroughRxBuffer_[offset + i] = data[2 + i];
  }

  passthroughRxOffset_ = offset + dataLen;
}

void VescCan::handleProcessRxBuffer_(const uint8_t* data, uint8_t len) {
  if (!data || len < 6) {
    passthroughRxOffset_ = 0;
    return;
  }

  const uint8_t senderId = data[0];
  const uint16_t expectedLen = (static_cast<uint16_t>(data[2]) << 8) | data[3];
  const uint16_t expectedCrc = (static_cast<uint16_t>(data[4]) << 8) | data[5];

  if (senderId != targetControllerId_ || expectedLen > kCanPassthroughBufferSize ||
      expectedLen != passthroughRxOffset_) {
    passthroughRxOffset_ = 0;
    return;
  }

  const uint16_t crc = VescPacket::Crc16(passthroughRxBuffer_, expectedLen);
  if (crc == expectedCrc && packetHandler_) {
    packetHandler_(passthroughRxBuffer_, expectedLen, packetHandlerUserData_);
  }

  passthroughRxOffset_ = 0;
}

void VescCan::handleProcessShortBuffer_(const uint8_t* data, uint8_t len) {
  if (!data || len < 3) {
    return;
  }

  const uint8_t senderId = data[0];
  if (senderId != targetControllerId_) {
    return;
  }

  if (packetHandler_) {
    packetHandler_(data + 2, static_cast<size_t>(len - 2), packetHandlerUserData_);
  }
}

void VescCan::handleStatus1_(const uint8_t* data, uint8_t len, uint8_t senderId) {
  (void)senderId;
  if (!data || len < 8) {
    return;
  }

  portENTER_CRITICAL(&statusMux_);
  status1_.valid = true;
  status1_.erpm = readInt32_(data);
  status1_.currentA = static_cast<float>(readInt16_(data + 4)) / 10.0f;
  status1_.duty = static_cast<float>(readInt16_(data + 6)) / 1000.0f;
  status1_.updatedAtMs = millis();
  portEXIT_CRITICAL(&statusMux_);
}

int32_t VescCan::readInt32_(const uint8_t* data) {
  return static_cast<int32_t>((static_cast<uint32_t>(data[0]) << 24) |
                              (static_cast<uint32_t>(data[1]) << 16) |
                              (static_cast<uint32_t>(data[2]) << 8) |
                              static_cast<uint32_t>(data[3]));
}

int16_t VescCan::readInt16_(const uint8_t* data) {
  return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) |
                              static_cast<uint16_t>(data[1]));
}
