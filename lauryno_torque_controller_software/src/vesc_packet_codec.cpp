#include "vesc_packet_codec.h"

#include <string.h>

namespace VescPacket {
namespace {

uint8_t LengthBytesFromStart(uint8_t startByte) {
  if (startByte == 2) {
    return 1;
  }
  if (startByte == 3) {
    return 2;
  }
  if (startByte == 4) {
    return 3;
  }
  return 0;
}

}  // namespace

uint16_t Crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0;

  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

size_t EncodeFrame(const uint8_t* payload,
                   size_t payloadLen,
                   uint8_t* outFrame,
                   size_t outFrameCapacity) {
  if (!payload || !outFrame || payloadLen == 0 || payloadLen > kMaxPayloadLen) {
    return 0;
  }

  uint8_t lengthBytes = 0;
  uint8_t startByte = 0;

  if (payloadLen <= 0xFF) {
    startByte = 2;
    lengthBytes = 1;
  } else if (payloadLen <= 0xFFFF) {
    startByte = 3;
    lengthBytes = 2;
  } else {
    startByte = 4;
    lengthBytes = 3;
  }

  const size_t total = 1 + lengthBytes + payloadLen + 2 + 1;
  if (total > outFrameCapacity) {
    return 0;
  }

  size_t index = 0;
  outFrame[index++] = startByte;

  if (lengthBytes == 1) {
    outFrame[index++] = static_cast<uint8_t>(payloadLen);
  } else if (lengthBytes == 2) {
    outFrame[index++] = static_cast<uint8_t>(payloadLen >> 8);
    outFrame[index++] = static_cast<uint8_t>(payloadLen);
  } else {
    outFrame[index++] = static_cast<uint8_t>(payloadLen >> 16);
    outFrame[index++] = static_cast<uint8_t>(payloadLen >> 8);
    outFrame[index++] = static_cast<uint8_t>(payloadLen);
  }

  memcpy(outFrame + index, payload, payloadLen);
  index += payloadLen;

  const uint16_t crc = Crc16(payload, payloadLen);
  outFrame[index++] = static_cast<uint8_t>(crc >> 8);
  outFrame[index++] = static_cast<uint8_t>(crc);
  outFrame[index++] = 3;

  return index;
}

Parser::Parser()
    : state_(State::WaitStart),
      startByte_(0),
      lengthBytesExpected_(0),
      lengthBytesRead_(0),
      payloadLength_(0),
      payloadRead_(0),
      receivedCrc_(0),
      computedCrc_(0),
      callback_(nullptr),
      callbackUserData_(nullptr) {}

void Parser::reset() {
  state_ = State::WaitStart;
  resetCurrentFrame_();
}

void Parser::setCallback(PacketCallback callback, void* userData) {
  callback_ = callback;
  callbackUserData_ = userData;
}

void Parser::processByte(uint8_t byte) {
  switch (state_) {
    case State::WaitStart: {
      const uint8_t lengthBytes = LengthBytesFromStart(byte);
      if (lengthBytes == 0) {
        return;
      }

      startByte_ = byte;
      lengthBytesExpected_ = lengthBytes;
      lengthBytesRead_ = 0;
      payloadLength_ = 0;
      payloadRead_ = 0;
      receivedCrc_ = 0;
      computedCrc_ = 0;
      state_ = State::ReadLength;
    } break;

    case State::ReadLength:
      payloadLength_ = (payloadLength_ << 8) | byte;
      lengthBytesRead_++;

      if (lengthBytesRead_ >= lengthBytesExpected_) {
        if (payloadLength_ == 0 || payloadLength_ > kMaxPayloadLen) {
          reset();
          return;
        }

        if ((startByte_ == 3 && payloadLength_ < 0xFF) ||
            (startByte_ == 4 && payloadLength_ < 0xFFFF)) {
          reset();
          return;
        }

        payloadRead_ = 0;
        state_ = State::ReadPayload;
      }
      break;

    case State::ReadPayload:
      payload_[payloadRead_++] = byte;
      if (payloadRead_ >= payloadLength_) {
        computedCrc_ = Crc16(payload_, payloadLength_);
        state_ = State::ReadCrcHigh;
      }
      break;

    case State::ReadCrcHigh:
      receivedCrc_ = static_cast<uint16_t>(byte) << 8;
      state_ = State::ReadCrcLow;
      break;

    case State::ReadCrcLow:
      receivedCrc_ |= byte;
      state_ = State::WaitStop;
      break;

    case State::WaitStop:
      if (byte == 3 && receivedCrc_ == computedCrc_ && callback_) {
        callback_(payload_, payloadLength_, callbackUserData_);
      }
      reset();
      break;
  }
}

void Parser::processBytes(const uint8_t* data, size_t len) {
  if (!data || len == 0) {
    return;
  }

  for (size_t i = 0; i < len; ++i) {
    processByte(data[i]);
  }
}

void Parser::resetCurrentFrame_() {
  startByte_ = 0;
  lengthBytesExpected_ = 0;
  lengthBytesRead_ = 0;
  payloadLength_ = 0;
  payloadRead_ = 0;
  receivedCrc_ = 0;
  computedCrc_ = 0;
}

}  // namespace VescPacket
