#include "fram_storage.h"

#include <Wire.h>

#include <algorithm>

FramStorage::FramStorage(gpio_num_t sdaPin, gpio_num_t sclPin, uint8_t baseAddress)
    : sdaPin_(sdaPin),
      sclPin_(sclPin),
      baseAddress_(baseAddress),
      busMutex_(nullptr),
      ready_(false),
      hasError_(false),
      errorCount_(0) {}

bool FramStorage::begin() {
  if (!busMutex_) {
    busMutex_ = xSemaphoreCreateMutex();
    if (!busMutex_) {
      markError_();
      return false;
    }
  }

  if (xSemaphoreTake(busMutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }

  Wire.begin(static_cast<int>(sdaPin_), static_cast<int>(sclPin_));

  Wire.beginTransmission(baseAddress_);
  if (Wire.endTransmission() != 0) {
    markError_();
    ready_ = false;
    xSemaphoreGive(busMutex_);
    return false;
  }

  ready_ = true;
  xSemaphoreGive(busMutex_);
  return true;
}

bool FramStorage::isReady() const {
  if (!busMutex_) {
    return ready_;
  }

  if (xSemaphoreTake(busMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return false;
  }

  const bool ready = ready_;
  xSemaphoreGive(busMutex_);
  return ready;
}

bool FramStorage::read(uint16_t address, uint8_t* data, size_t len) {
  if (!busMutex_) {
    markError_();
    return false;
  }

  if (xSemaphoreTake(busMutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }

  if (!ready_ || !data || len == 0 || !rangeValid_(address, len)) {
    markError_();
    xSemaphoreGive(busMutex_);
    return false;
  }

  size_t offset = 0;
  while (offset < len) {
    const uint16_t currentAddress = static_cast<uint16_t>(address + offset);
    const size_t blockRemaining = kBlockSizeBytes - static_cast<size_t>(wordAddressFor_(currentAddress));
    const size_t chunkLen = std::min(len - offset, blockRemaining);

    const uint8_t deviceAddress = deviceAddressFor_(currentAddress);
    const uint8_t wordAddress = wordAddressFor_(currentAddress);

    Wire.beginTransmission(deviceAddress);
    Wire.write(wordAddress);
    if (Wire.endTransmission(false) != 0) {
      markError_();
      xSemaphoreGive(busMutex_);
      return false;
    }

    const uint8_t requested = static_cast<uint8_t>(chunkLen);
    const uint8_t received = Wire.requestFrom(static_cast<int>(deviceAddress), static_cast<int>(requested), true);
    if (received != requested) {
      markError_();
      xSemaphoreGive(busMutex_);
      return false;
    }

    for (uint8_t i = 0; i < requested; ++i) {
      if (!Wire.available()) {
        markError_();
        xSemaphoreGive(busMutex_);
        return false;
      }
      data[offset + i] = static_cast<uint8_t>(Wire.read());
    }

    offset += chunkLen;
  }

  xSemaphoreGive(busMutex_);
  return ready_;
}

bool FramStorage::write(uint16_t address, const uint8_t* data, size_t len) {
  if (!busMutex_) {
    markError_();
    return false;
  }

  if (xSemaphoreTake(busMutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }

  if (!ready_ || !data || len == 0 || !rangeValid_(address, len)) {
    markError_();
    xSemaphoreGive(busMutex_);
    return false;
  }

  size_t offset = 0;
  while (offset < len) {
    const uint16_t currentAddress = static_cast<uint16_t>(address + offset);
    const size_t blockRemaining = kBlockSizeBytes - static_cast<size_t>(wordAddressFor_(currentAddress));
    const size_t chunkLen = std::min(len - offset, blockRemaining);

    const uint8_t deviceAddress = deviceAddressFor_(currentAddress);
    const uint8_t wordAddress = wordAddressFor_(currentAddress);

    Wire.beginTransmission(deviceAddress);
    Wire.write(wordAddress);
    for (size_t i = 0; i < chunkLen; ++i) {
      Wire.write(data[offset + i]);
    }

    if (Wire.endTransmission(true) != 0) {
      markError_();
      xSemaphoreGive(busMutex_);
      return false;
    }

    offset += chunkLen;
  }

  xSemaphoreGive(busMutex_);
  return true;
}

bool FramStorage::hasError() const {
  if (!busMutex_) {
    return hasError_;
  }

  if (xSemaphoreTake(busMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return true;
  }

  const bool hasError = hasError_;
  xSemaphoreGive(busMutex_);
  return hasError;
}

uint32_t FramStorage::errorCount() const {
  if (!busMutex_) {
    return errorCount_;
  }

  if (xSemaphoreTake(busMutex_, pdMS_TO_TICKS(20)) != pdTRUE) {
    return errorCount_;
  }

  const uint32_t count = errorCount_;
  xSemaphoreGive(busMutex_);
  return count;
}

bool FramStorage::rangeValid_(uint16_t address, size_t len) const {
  return len <= kFramSizeBytes &&
         static_cast<size_t>(address) + len <= static_cast<size_t>(kFramSizeBytes);
}

uint8_t FramStorage::deviceAddressFor_(uint16_t address) const {
  return static_cast<uint8_t>(baseAddress_ + ((address >> 8) & 0x07));
}

uint8_t FramStorage::wordAddressFor_(uint16_t address) const {
  return static_cast<uint8_t>(address & 0xFF);
}

void FramStorage::markError_() {
  hasError_ = true;
  ++errorCount_;
}
