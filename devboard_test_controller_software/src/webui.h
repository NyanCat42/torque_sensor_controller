#pragma once
#include <Arduino.h>
#include "config.h"

void webuiInit(AppConfig &cfg);
void webuiLoop();  // call in main loop (for ElegantOTA)

// Live status values updated by control loop
struct LiveStatus {
    float torque_nm;
    float torque_sensor_v;
    float cadence_rpm;
    float rider_power_w;
    float motor_current_a;
    float esp_temp_c;             // ESP32 internal temperature (°C)
    bool  ble_bridge_active;      // BLE client currently connected
    bool  torque_assist_enabled;  // torque-sensor assist toggle state
};

void webuiSetStatus(const LiveStatus &s);

// Callbacks
typedef void (*ConfigChangedCb)();
typedef void (*BleBridgeToggleCb)(bool enabled);
void webuiOnConfigChanged(ConfigChangedCb cb);
void webuiOnBleBridgeToggle(BleBridgeToggleCb cb);
