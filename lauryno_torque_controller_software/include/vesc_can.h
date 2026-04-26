#pragma once

#include <Arduino.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <stddef.h>
#include <stdint.h>

class VescCan {
 public:
  struct Status1 {
    bool valid = false;
    int32_t erpm = 0;
    float currentA = 0.0f;
    float duty = 0.0f;
    uint32_t updatedAtMs = 0;
  };

  using PacketHandler = void (*)(const uint8_t* payload, size_t len, void* userData);

  VescCan(gpio_num_t txPin,
          gpio_num_t rxPin,
          uint32_t baudRate,
          uint8_t localControllerId,
          uint8_t targetControllerId);

  bool begin();
  bool isReady() const;
  void poll();

  bool setCurrent(float currentA);
  bool sendPassthroughPacket(const uint8_t* payload, size_t len, uint8_t sendMode = 0);

  void setPacketHandler(PacketHandler handler, void* userData);

  void setTargetControllerId(uint8_t id);
  uint8_t targetControllerId() const;
  uint8_t localControllerId() const;

  Status1 status1() const;

 private:
  enum CanPacketId : uint8_t {
    CAN_PACKET_SET_CURRENT = 1,
    CAN_PACKET_FILL_RX_BUFFER = 5,
    CAN_PACKET_FILL_RX_BUFFER_LONG = 6,
    CAN_PACKET_PROCESS_RX_BUFFER = 7,
    CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
    CAN_PACKET_STATUS = 9,
  };

  static constexpr size_t kCanPassthroughBufferSize = 2048;

  bool sendExtendedFrame_(uint8_t targetControllerId,
                          uint8_t command,
                          const uint8_t* data,
                          uint8_t len);
  void processIncomingFrame_(const twai_message_t& msg);
  void handleFillRxBuffer_(const uint8_t* data, uint8_t len);
  void handleFillRxBufferLong_(const uint8_t* data, uint8_t len);
  void handleProcessRxBuffer_(const uint8_t* data, uint8_t len);
  void handleProcessShortBuffer_(const uint8_t* data, uint8_t len);
  void handleStatus1_(const uint8_t* data, uint8_t len, uint8_t senderId);

  static int32_t readInt32_(const uint8_t* data);
  static int16_t readInt16_(const uint8_t* data);

  gpio_num_t txPin_;
  gpio_num_t rxPin_;
  uint32_t baudRate_;
  uint8_t localControllerId_;
  uint8_t targetControllerId_;

  bool ready_;

  uint8_t passthroughRxBuffer_[kCanPassthroughBufferSize];
  size_t passthroughRxOffset_;

  PacketHandler packetHandler_;
  void* packetHandlerUserData_;

  SemaphoreHandle_t driverMutex_;
  mutable portMUX_TYPE statusMux_;

  Status1 status1_;
};
