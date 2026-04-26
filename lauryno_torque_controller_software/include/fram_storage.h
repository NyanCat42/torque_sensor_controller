#pragma once

#include <Arduino.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <stddef.h>
#include <stdint.h>

class FramStorage {
 public:
  FramStorage(gpio_num_t sdaPin, gpio_num_t sclPin, uint8_t baseAddress);

  bool begin();
  bool isReady() const;

  bool read(uint16_t address, uint8_t* data, size_t len);
  bool write(uint16_t address, const uint8_t* data, size_t len);

  bool hasError() const;
  uint32_t errorCount() const;

 private:
  static constexpr uint16_t kFramSizeBytes = 2048;
  static constexpr uint16_t kBlockSizeBytes = 256;

  bool rangeValid_(uint16_t address, size_t len) const;
  uint8_t deviceAddressFor_(uint16_t address) const;
  uint8_t wordAddressFor_(uint16_t address) const;
  void markError_();

  gpio_num_t sdaPin_;
  gpio_num_t sclPin_;
  uint8_t baseAddress_;
  mutable SemaphoreHandle_t busMutex_;
  bool ready_;
  bool hasError_;
  uint32_t errorCount_;
};
