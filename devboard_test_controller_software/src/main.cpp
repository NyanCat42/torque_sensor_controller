#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "vesc.h"
#include "webui.h"
#include "ble_bridge.h"
#include "esp_system.h"  // temperatureRead()

// --- Global state ---
static AppConfig cfg;
static float commandedCurrent = 0.0f;
static volatile bool controlEnabled = true;
static TaskHandle_t controlTaskHandle = nullptr;

// --- Motor constants (hardcoded) ---
#define MOTOR_POLES     20
#define POLE_PAIRS      (MOTOR_POLES / 2)   // 10
#define GEAR_RATIO      5.0f                // 1:5 reduction
#define KT_FACTOR       (1.5f * POLE_PAIRS) // 15

// --- Control loop FreeRTOS task ---
// ALL VESC UART access happens here (single-threaded, no race conditions)
#define CONTROL_PERIOD_MS 10   // 100 Hz

static void controlTask(void *param) {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

        // If BLE bridge client is connected, pause control — VESC Tool has UART
        if (bleBridgeConnected()) {
            if (commandedCurrent != 0.0f) {
                commandedCurrent = 0.0f;
                // Can't send zero to VESC here — BLE owns UART
            }
            continue;
        }

        // Update sensors
        sensorsUpdate(cfg);

        float torque  = getTorqueNm();
        float cadence = getCadenceRpm();

        // Determine target current
        float targetCurrent = 0.0f;

        if (controlEnabled
            && cadence >= cfg.min_cadence_rpm)
        {
            // Torque matching: motor provides assist_ratio × rider torque at wheel
            // Kt = 1.5 * pole_pairs * flux_linkage
            // I  = (rider_torque * assist_ratio) / (gear_ratio * Kt)
            float Kt = KT_FACTOR * cfg.flux_linkage;  // 15 * 0.013891 = 0.208365
            targetCurrent = (torque * cfg.assist_ratio) / (GEAR_RATIO * Kt);  // 40 Nm * 1 / (5 * 0.208365) ≈ 38.4 A at motor for 40 Nm at crank

            // Torque-based assist ramp: no assist below start, linear ramp to full
            if (cfg.assist_start_nm < cfg.assist_full_nm) {
                float rampFactor;
                if (torque <= cfg.assist_start_nm) {
                    rampFactor = 0.0f;
                } else if (torque >= cfg.assist_full_nm) {
                    rampFactor = 1.0f;
                } else {
                    rampFactor = (torque - cfg.assist_start_nm)
                               / (cfg.assist_full_nm - cfg.assist_start_nm);
                }
                targetCurrent *= rampFactor;
            }
        }

        // Clamp
        if (targetCurrent < 0.0f) targetCurrent = 0.0f;
        if (targetCurrent > cfg.max_motor_current) targetCurrent = cfg.max_motor_current;

        // Soft ramp: limit rate of change
        float maxDelta = cfg.current_ramp_rate * (CONTROL_PERIOD_MS / 1000.0f);
        float delta = targetCurrent - commandedCurrent;
        if (delta > maxDelta)  delta = maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;
        commandedCurrent += delta;

        // Extra safety: snap to zero if target is zero and we're close
        if (targetCurrent == 0.0f && commandedCurrent < 0.5f) {
            commandedCurrent = 0.0f;
        }

        // Send to VESC (skip when passthrough mode active)
        if (cfg.torque_assist_enabled) {
            vescSetCurrent(commandedCurrent, cfg);
        }

        // Update web UI live status
        LiveStatus st;
        st.torque_nm             = torque;
        st.torque_sensor_v       = getTorqueSensorV();
        st.cadence_rpm           = cadence;
        st.rider_power_w         = getRiderPowerW();
        st.motor_current_a       = commandedCurrent;
        st.esp_temp_c            = temperatureRead();
        st.ble_bridge_active     = bleBridgeConnected();
        st.torque_assist_enabled = cfg.torque_assist_enabled;
        webuiSetStatus(st);
    }
}

// --- Callbacks from web UI ---
static void onConfigChanged() {
    Serial.println("[Main] Config updated");
}

static void onBleToggle(bool enabled) {
    if (enabled) {
        bleBridgeInit();
        bleBridgeStart();
        Serial.println("[Main] BLE bridge enabled");
    } else {
        bleBridgeStop();
        Serial.println("[Main] BLE bridge disabled");
    }
}

// --- Arduino entry points ---
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP32 Ebike Controller ===");

    // Load config from NVS
    configLoad(cfg);
    Serial.printf("[Config] Assist=%.1fx, MaxCur=%.0fA, BLE=%s\n",
                  cfg.assist_ratio, cfg.max_motor_current,
                  cfg.ble_bridge_enabled ? "on" : "off");

    // Init subsystems
    sensorsInit(cfg);
    vescInit(cfg);

    // Web UI + WiFi
    webuiOnConfigChanged(onConfigChanged);
    webuiOnBleBridgeToggle(onBleToggle);
    webuiInit(cfg);

    // BLE bridge (if enabled in config)
    if (cfg.ble_bridge_enabled) {
        bleBridgeInit();
        bleBridgeStart();
    }

    // Start control loop on core 1 (Arduino loop runs on core 1 too, but this is a dedicated task)
    xTaskCreatePinnedToCore(
        controlTask,
        "control",
        4096,
        nullptr,
        2,           // priority above loop()
        &controlTaskHandle,
        1            // core 1
    );

    Serial.println("[Main] Setup complete");
}

void loop() {
    // OTA handling
    webuiLoop();

    // BLE bridge relay (UART RX → BLE TX)
    if (cfg.ble_bridge_enabled) {
        bleBridgeLoop();
    }

    delay(10);
}