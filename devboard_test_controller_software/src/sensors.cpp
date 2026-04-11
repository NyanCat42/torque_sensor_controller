#include "sensors.h"
#include <driver/pcnt.h>
#include <esp_adc_cal.h>

// --- Torque ADC (EMA filter) ---
static float emaMv = 0.0f;
static bool  emaInit = false;
static esp_adc_cal_characteristics_t adcChars;

// --- Cadence PCNT ---
#define PCNT_UNIT PCNT_UNIT_0
static volatile int16_t lastPcntCount = 0;
static uint32_t lastCadenceReadMs = 0;

// --- Computed values ---
static float torqueNm     = 0.0f;
static float torqueSensorV = 0.0f;
static float cadenceRpm   = 0.0f;
static float riderPowerW  = 0.0f;
static uint32_t lastNonZeroPulseMs = 0;

// ---- Cadence timeout: if no pulses for this long, RPM = 0 ----
#define CADENCE_TIMEOUT_MS 1500
// ---- Cadence read interval ----
#define CADENCE_INTERVAL_MS 500

void sensorsInit(const AppConfig &cfg) {
    // --- ADC setup with calibration ---
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adcChars);

    emaMv = 0.0f;
    emaInit = false;

    // --- PCNT setup for cadence ---
    pcnt_config_t pcntCfg = {};
    pcntCfg.pulse_gpio_num = CADENCE_PIN;
    pcntCfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
    pcntCfg.channel        = PCNT_CHANNEL_0;
    pcntCfg.unit           = PCNT_UNIT;
    pcntCfg.pos_mode       = PCNT_COUNT_INC;  // count rising edges
    pcntCfg.neg_mode       = PCNT_COUNT_DIS;
    pcntCfg.lctrl_mode     = PCNT_MODE_KEEP;
    pcntCfg.hctrl_mode     = PCNT_MODE_KEEP;
    pcntCfg.counter_h_lim  = 30000;
    pcntCfg.counter_l_lim  = 0;

    pcnt_unit_config(&pcntCfg);

    // Filter glitches shorter than ~1 us (APB_CLK / 1023 ≈ 78 kHz)
    pcnt_set_filter_value(PCNT_UNIT, 1023);
    pcnt_filter_enable(PCNT_UNIT);

    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);

    lastCadenceReadMs = millis();
    lastNonZeroPulseMs = millis();
}

static float readTorque(const AppConfig &cfg) {
    // Read calibrated millivolts from ADC
    uint32_t mv = 0;
    esp_adc_cal_get_voltage(ADC_CHANNEL_0, &adcChars, &mv);

    // EMA filter: out = alpha * new + (1 - alpha) * prev
    if (!emaInit) {
        emaMv = (float)mv;
        emaInit = true;
    } else {
        emaMv += cfg.torque_ema_alpha * ((float)mv - emaMv);
    }

    // Convert from post-divider mV to pre-divider sensor voltage
    float sensorV = (emaMv / 1000.0f) / cfg.divider_ratio;

    // Clamp to valid sensor range
    if (sensorV < cfg.torque_min_v) sensorV = cfg.torque_min_v;
    if (sensorV > cfg.torque_max_v) sensorV = cfg.torque_max_v;

    torqueSensorV = sensorV;

    // Map sensor voltage to torque
    float nm = (sensorV - cfg.torque_min_v)
             / (cfg.torque_max_v - cfg.torque_min_v)
             * cfg.torque_max_nm;
    return nm;
}

static float readCadence(const AppConfig &cfg) {
    uint32_t now = millis();
    uint32_t elapsed = now - lastCadenceReadMs;
    if (elapsed < CADENCE_INTERVAL_MS) {
        return cadenceRpm; // not time yet, return last value
    }

    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT, &count);
    pcnt_counter_clear(PCNT_UNIT);

    int16_t pulses = count; // accumulated since last read
    lastCadenceReadMs = now;

    if (pulses > 0) {
        lastNonZeroPulseMs = now;
        float revolutions = (float)pulses / cfg.cadence_ppr;
        float intervalSec = elapsed / 1000.0f;
        return (revolutions / intervalSec) * 60.0f;
    }

    // No pulses — check timeout
    if ((now - lastNonZeroPulseMs) > CADENCE_TIMEOUT_MS) {
        return 0.0f;
    }
    return cadenceRpm; // hold last value briefly
} 


void sensorsUpdate(const AppConfig &cfg) {
    torqueNm    = readTorque(cfg);
    cadenceRpm  = readCadence(cfg);
    // Power = torque * angular_velocity = torque * (RPM * 2π / 60)
    riderPowerW = torqueNm * cadenceRpm * 0.10472f; // 2*PI/60 ≈ 0.10472
}

float getTorqueNm()      { return torqueNm; }
float getTorqueSensorV() { return torqueSensorV; }
float getCadenceRpm()    { return cadenceRpm; }
float getRiderPowerW()   { return riderPowerW; }
