#pragma once

#include <stddef.h>
#include <stdint.h>

namespace VescPacket {

constexpr size_t kMaxPayloadLen = 2048;
constexpr size_t kMaxFrameLen = kMaxPayloadLen + 8;

using PacketCallback = void (*)(const uint8_t* payload, size_t len, void* userData);

uint16_t Crc16(const uint8_t* data, size_t len);
size_t EncodeFrame(const uint8_t* payload,
                   size_t payloadLen,
                   uint8_t* outFrame,
                   size_t outFrameCapacity);

class Parser {
 public:
  Parser();

  void reset();
  void setCallback(PacketCallback callback, void* userData);
  void processByte(uint8_t byte);
  void processBytes(const uint8_t* data, size_t len);

 private:
  enum class State : uint8_t {
    WaitStart,
    ReadLength,
    ReadPayload,
    ReadCrcHigh,
    ReadCrcLow,
    WaitStop,
  };

  void resetCurrentFrame_();

  State state_;
  uint8_t startByte_;
  uint8_t lengthBytesExpected_;
  uint8_t lengthBytesRead_;
  size_t payloadLength_;
  size_t payloadRead_;
  uint16_t receivedCrc_;
  uint16_t computedCrc_;
  uint8_t payload_[kMaxPayloadLen];

  PacketCallback callback_;
  void* callbackUserData_;
};

}  // namespace VescPacket
