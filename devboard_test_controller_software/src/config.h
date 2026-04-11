#pragma once
#include <Arduino.h>
#include <Preferences.h>

struct AppConfig {
    // Assist
    float assist_ratio;        // multiplier: motor_power = rider_power * ratio
    float max_motor_current;   // amps, hard clamp
    float min_cadence_rpm;     // below this, zero assist
    float current_ramp_rate;   // max A/s change rate
    float assist_start_nm;     // torque below this = no assist (deadband)
    float assist_full_nm;      // torque at which assist ramp reaches 100%

    // Torque sensor (pre-divider values)
    float torque_min_v;        // sensor voltage at 0 Nm
    float torque_max_v;        // sensor voltage at max Nm
    float torque_max_nm;       // max torque value
    float divider_ratio;       // R2/(R1+R2)
    float torque_ema_alpha;    // EMA filter weight (0..1), higher = faster response

    // Cadence sensor
    uint16_t cadence_ppr;      // pulses per revolution

    // Motor / VESC
    float flux_linkage;        // Wb, measured by VESC Tool (for torque constant)
    uint32_t vesc_baud;        // UART baud rate

    // WiFi
    char wifi_ssid[33];
    char wifi_pass[65];
    char ap_pass[65];

    // BLE bridge
    bool ble_bridge_enabled;

    // Control mode
    bool torque_assist_enabled; // false = passthrough (throttle controls VESC directly)
};

// Default configuration values
inline AppConfig defaultConfig() {
    AppConfig c = {};
    c.assist_ratio      = 1.00f;
    c.max_motor_current = 30.0f;
    c.min_cadence_rpm   = 15.0f;
    c.current_ramp_rate = 10.0f;
    c.assist_start_nm   = 5.0f;    // no assist below 5 Nm
    c.assist_full_nm    = 20.0f;   // full assist above 20 Nm
    c.torque_min_v      = 0.75f;
    c.torque_max_v      = 5.0f;
    c.torque_max_nm     = 140.0f;
    c.divider_ratio     = 0.6623f;  // 10k / (5.1k + 10k)
    c.torque_ema_alpha  = 0.3f;     // EMA weight, 0.1 = smooth, 0.9 = snappy
    c.cadence_ppr       = 32;
    c.flux_linkage      = 0.01f;   // Wb — measure with VESC Tool
    c.vesc_baud         = 115200;
    strlcpy(c.wifi_ssid, "",         sizeof(c.wifi_ssid));
    strlcpy(c.wifi_pass, "",         sizeof(c.wifi_pass));
    strlcpy(c.ap_pass,   "ebike123", sizeof(c.ap_pass));
    c.ble_bridge_enabled = false;
    c.torque_assist_enabled = true;
    return c;
}

void configLoad(AppConfig &cfg);
void configSave(const AppConfig &cfg);
