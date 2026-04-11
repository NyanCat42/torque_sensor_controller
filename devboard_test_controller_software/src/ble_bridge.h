#pragma once
#include <Arduino.h>
#include "config.h"

void bleBridgeInit();
void bleBridgeStart();   // start advertising
void bleBridgeStop();    // stop advertising, disconnect
bool bleBridgeConnected(); // true if a BLE client is connected (control loop should pause)
void bleBridgeLoop();    // call in loop to relay UART ↔ BLE
