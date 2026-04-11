#pragma once
#include <Arduino.h>
#include "config.h"

#define VESC_RX_PIN 16
#define VESC_TX_PIN 17

void vescInit(const AppConfig &cfg);
void vescSetCurrent(float amps, const AppConfig &cfg); // clamp + send

// For BLE bridge: raw UART access
HardwareSerial &vescGetSerial();
