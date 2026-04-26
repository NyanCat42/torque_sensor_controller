#include "ota_http_server.h"

#include <Update.h>

#include <stdlib.h>

#include "app_config.h"
#include "odometer_manager.h"
#include "pedal_assist_manager.h"
#include "vesc_can.h"
#include "vesc_tcp_bridge.h"

namespace {

String JsonBool(bool value) {
  return value ? "true" : "false";
}

String JsonFloat(float value, uint8_t decimals) {
  return String(static_cast<double>(value), static_cast<unsigned int>(decimals));
}

bool ParseUint16Arg(WebServer& server, const char* name, uint16_t* outValue) {
  if (!outValue || !server.hasArg(name)) {
    return false;
  }

  const String raw = server.arg(name);
  char* endPtr = nullptr;
  const long parsed = strtol(raw.c_str(), &endPtr, 10);
  if (endPtr == raw.c_str() || *endPtr != '\0' || parsed < 0 || parsed > 65535) {
    return false;
  }

  *outValue = static_cast<uint16_t>(parsed);
  return true;
}

bool ParseUint32Arg(WebServer& server, const char* name, uint32_t* outValue) {
  if (!outValue || !server.hasArg(name)) {
    return false;
  }

  const String raw = server.arg(name);
  char* endPtr = nullptr;
  const unsigned long parsed = strtoul(raw.c_str(), &endPtr, 10);
  if (endPtr == raw.c_str() || *endPtr != '\0') {
    return false;
  }

  *outValue = static_cast<uint32_t>(parsed);
  return true;
}

bool ParseFloatArg(WebServer& server, const char* name, float* outValue) {
  if (!outValue || !server.hasArg(name)) {
    return false;
  }

  const String raw = server.arg(name);
  char* endPtr = nullptr;
  const double parsed = strtod(raw.c_str(), &endPtr);
  if (endPtr == raw.c_str() || *endPtr != '\0') {
    return false;
  }

  *outValue = static_cast<float>(parsed);
  return true;
}

}  // namespace

OtaHttpServer::OtaHttpServer(uint16_t port,
                             VescCan* vescCan,
                             VescTcpBridge* vescTcpBridge,
                             OdometerManager* odometerManager,
                             PedalAssistManager* pedalAssistManager)
    : server_(port),
      activeIp_(0, 0, 0, 0),
      vescCan_(vescCan),
      vescTcpBridge_(vescTcpBridge),
      odometerManager_(odometerManager),
      pedalAssistManager_(pedalAssistManager) {}

void OtaHttpServer::begin(const String& networkModeLabel, const IPAddress& activeIp) {
  networkModeLabel_ = networkModeLabel;
  activeIp_ = activeIp;

  registerRoutes_();
  server_.begin();

  Serial.printf("[OTA] HTTP OTA server ready at http://%s/\n", activeIp_.toString().c_str());
}

void OtaHttpServer::handleClient() {
  server_.handleClient();
}

void OtaHttpServer::registerRoutes_() {
  server_.on("/", HTTP_GET, [this]() {
    server_.send(200, "text/html", pageHtml_());
  });

  server_.on("/health", HTTP_GET, [this]() {
    String status = "{\"ok\":true,\"mode\":\"" + networkModeLabel_ + "\",\"ip\":\"" + activeIp_.toString() + "\"}";
    server_.send(200, "application/json", status);
  });

  server_.on("/api/vesc/status", HTTP_GET, [this]() {
    if (!vescCan_) {
      server_.send(503, "application/json", "{\"ok\":false,\"error\":\"vesc_can_not_configured\"}");
      return;
    }

    const VescCan::Status1 status1 = vescCan_->status1();
    const uint32_t ageMs = status1.valid ? (millis() - status1.updatedAtMs) : 0;
    String json = "{";
    json += "\"ok\":true";
    json += ",\"can_ready\":" + JsonBool(vescCan_->isReady());
    json += ",\"local_id\":" + String(vescCan_->localControllerId());
    json += ",\"target_id\":" + String(vescCan_->targetControllerId());
    json += ",\"status1_valid\":" + JsonBool(status1.valid);
    json += ",\"erpm\":" + String(status1.erpm);
    json += ",\"current_a\":" + String(status1.currentA, 2);
    json += ",\"duty\":" + String(status1.duty, 3);
    json += ",\"updated_ms\":" + String(status1.updatedAtMs);
    json += ",\"status_age_ms\":" + String(ageMs);
    json += "}";

    server_.send(200, "application/json", json);
  });

  server_.on("/api/bridge", HTTP_GET, [this]() {
    if (!vescTcpBridge_) {
      server_.send(503, "application/json", "{\"ok\":false,\"error\":\"bridge_not_configured\"}");
      return;
    }

    String json = "{";
    json += "\"ok\":true";
    json += ",\"enabled\":" + JsonBool(vescTcpBridge_->isEnabled());
    json += ",\"has_client\":" + JsonBool(vescTcpBridge_->hasClient());
    json += ",\"port\":" + String(vescTcpBridge_->port());
    json += "}";

    server_.send(200, "application/json", json);
  });

  server_.on("/api/bridge/enable", HTTP_POST, [this]() {
    if (!vescTcpBridge_) {
      server_.send(503, "application/json", "{\"ok\":false,\"error\":\"bridge_not_configured\"}");
      return;
    }
    vescTcpBridge_->setEnabled(true);
    server_.send(200, "application/json", "{\"ok\":true,\"enabled\":true}");
  });

  server_.on("/api/bridge/disable", HTTP_POST, [this]() {
    if (!vescTcpBridge_) {
      server_.send(503, "application/json", "{\"ok\":false,\"error\":\"bridge_not_configured\"}");
      return;
    }
    vescTcpBridge_->setEnabled(false);
    server_.send(200, "application/json", "{\"ok\":true,\"enabled\":false}");
  });

  server_.on("/api/odometer/status", HTTP_GET, [this]() {
    if (!odometerManager_) {
      server_.send(503, "application/json", "{\"ok\":false,\"error\":\"odometer_not_configured\"}");
      return;
    }

    const OdometerManager::Status status = odometerManager_->status();
    const uint32_t ageMs = status.lastAcceptedPulseMs == 0 ? 0 : millis() - status.lastAcceptedPulseMs;

    String json = "{";
    json += "\"ok\":true";
    json += ",\"initialized\":" + JsonBool(status.initialized);
    json += ",\"hall_active\":" + JsonBool(status.hallActive);
    json += ",\"speed_fresh\":" + JsonBool(status.speedFresh);
    json += ",\"speed_kmh\":" + JsonFloat(status.speedKmh, 2);
    json += ",\"distance_km\":" + JsonFloat(status.totalDistanceKm, 3);
    json += ",\"distance_um\":" + String(static_cast<uint32_t>(status.totalDistanceUm >> 32));
    json += ",\"distance_um_low\":" + String(static_cast<uint32_t>(status.totalDistanceUm));
    json += ",\"accepted_pulses\":" + String(status.acceptedPulses);
    json += ",\"rejected_pulses\":" + String(status.rejectedPulses);
    json += ",\"last_pulse_ms\":" + String(status.lastAcceptedPulseMs);
    json += ",\"pulse_age_ms\":" + String(ageMs);
    json += ",\"last_persist_ms\":" + String(status.lastPersistMs);
    json += ",\"fram_ready\":" + JsonBool(status.framReady);
    json += ",\"fram_error\":" + JsonBool(status.framError);
    json += ",\"fram_error_count\":" + String(status.framErrorCount);
    json += "}";

    server_.send(200, "application/json", json);
  });

  server_.on("/api/odometer/config", HTTP_GET, [this]() {
    if (!odometerManager_) {
      server_.send(503, "application/json", "{\"ok\":false,\"error\":\"odometer_not_configured\"}");
      return;
    }

    const OdometerManager::Config config = odometerManager_->config();

    String json = "{";
    json += "\"ok\":true";
    json += ",\"wheel_diameter_mm\":" + String(config.wheelDiameterMm);
    json += ",\"debounce_ms\":" + String(config.debounceMs);
    json += ",\"persist_interval_ms\":" + String(config.persistIntervalMs);
    json += ",\"max_speed_kmh\":" + JsonFloat(config.maxSpeedKmh, 1);
    json += ",\"bounds\":{";
    json += "\"wheel_min\":" + String(AppConfig::kOdometerMinWheelDiameterMm);
    json += ",\"wheel_max\":" + String(AppConfig::kOdometerMaxWheelDiameterMm);
    json += ",\"debounce_min\":" + String(AppConfig::kOdometerMinDebounceMs);
    json += ",\"debounce_max\":" + String(AppConfig::kOdometerMaxDebounceMs);
    json += ",\"persist_min\":" + String(AppConfig::kOdometerMinPersistIntervalMs);
    json += ",\"persist_max\":" + String(AppConfig::kOdometerMaxPersistIntervalMs);
    json += ",\"max_speed_hard\":" + JsonFloat(AppConfig::kOdometerHardSpeedLimitKmh, 1);
    json += "}";
    json += "}";

    server_.send(200, "application/json", json);
  });

  server_.on("/api/odometer/config", HTTP_POST, [this]() {
    if (!odometerManager_) {
      server_.send(503, "application/json", "{\"ok\":false,\"error\":\"odometer_not_configured\"}");
      return;
    }

    OdometerManager::Config config = odometerManager_->config();

    if (!ParseUint16Arg(server_, "wheel_diameter_mm", &config.wheelDiameterMm) ||
        !ParseUint16Arg(server_, "debounce_ms", &config.debounceMs) ||
        !ParseUint32Arg(server_, "persist_interval_ms", &config.persistIntervalMs) ||
        !ParseFloatArg(server_, "max_speed_kmh", &config.maxSpeedKmh)) {
      server_.send(400,
                   "application/json",
                   "{\"ok\":false,\"error\":\"invalid_or_missing_form_fields\"}");
      return;
    }

    String error;
    if (!odometerManager_->setConfig(config, &error)) {
      String json = "{\"ok\":false,\"error\":\"" + error + "\"}";
      server_.send(400, "application/json", json);
      return;
    }

    String json = "{";
    json += "\"ok\":true";
    if (error.length() > 0) {
      json += ",\"warning\":\"" + error + "\"";
    }
    json += "}";
    server_.send(200, "application/json", json);
  });

  server_.on("/api/pedal/status", HTTP_GET, [this]() {
    if (!pedalAssistManager_) {
      server_.send(503, "application/json", "{\"ok\":false,\"error\":\"pedal_not_configured\"}");
      return;
    }

    const PedalAssistManager::Status status = pedalAssistManager_->status();
    const uint32_t commandAgeMs = status.lastCommandMs == 0 ? 0 : millis() - status.lastCommandMs;
    const uint32_t cadenceAgeMs = status.lastCadencePulseMs == 0 ? 0 : millis() - status.lastCadencePulseMs;

    String json = "{";
    json += "\"ok\":true";
    json += ",\"initialized\":" + JsonBool(status.initialized);
    json += ",\"cadence_fresh\":" + JsonBool(status.cadenceFresh);
    json += ",\"cadence_active\":" + JsonBool(status.cadenceActive);
    json += ",\"assist_enabled\":" + JsonBool(status.assistEnabled);
    json += ",\"high_cadence_averaging\":" + JsonBool(status.highCadenceAveraging);
    json += ",\"fail_safe_active\":" + JsonBool(status.failSafeActive);
    json += ",\"vesc_ready\":" + JsonBool(status.vescReady);
    json += ",\"control_late\":" + JsonBool(status.controlLate);
    json += ",\"last_can_write_ok\":" + JsonBool(status.lastCanWriteOk);
    json += ",\"updated_ms\":" + String(status.updatedAtMs);
    json += ",\"last_cadence_pulse_ms\":" + String(status.lastCadencePulseMs);
    json += ",\"last_command_ms\":" + String(status.lastCommandMs);
    json += ",\"command_age_ms\":" + String(commandAgeMs);
    json += ",\"cadence_age_ms\":" + String(cadenceAgeMs);
    json += ",\"cadence_pulses_total\":" + String(status.cadencePulsesTotal);
    json += ",\"torque_adc_raw\":" + String(status.torqueAdcRaw);
    json += ",\"cadence_rpm\":" + JsonFloat(status.cadenceRpm, 2);
    json += ",\"torque_raw_nm\":" + JsonFloat(status.torqueRawNm, 2);
    json += ",\"torque_filtered_nm\":" + JsonFloat(status.torqueFilteredNm, 2);
    json += ",\"torque_rotation_avg_nm\":" + JsonFloat(status.torqueRotationAvgNm, 2);
    json += ",\"torque_zero_offset_v\":" + JsonFloat(status.torqueZeroOffsetV, 4);
    json += ",\"gpio_voltage_v\":" + JsonFloat(status.gpioVoltageV, 3);
    json += ",\"sensor_voltage_v\":" + JsonFloat(status.sensorVoltageV, 3);
    json += ",\"ramp_factor\":" + JsonFloat(status.rampFactor, 3);
    json += ",\"assist_ratio\":" + JsonFloat(status.assistRatio, 2);
    json += ",\"motor_torque_cmd_nm\":" + JsonFloat(status.motorTorqueCmdNm, 2);
    json += ",\"motor_current_cmd_a\":" + JsonFloat(status.motorCurrentCmdA, 2);
    json += "}";

    server_.send(200, "application/json", json);
  });

  server_.on("/api/pedal/config", HTTP_GET, [this]() {
    if (!pedalAssistManager_) {
      server_.send(503, "application/json", "{\"ok\":false,\"error\":\"pedal_not_configured\"}");
      return;
    }

    const PedalAssistManager::Config config = pedalAssistManager_->config();

    String json = "{";
    json += "\"ok\":true";
    json += ",\"assist_ratio\":" + JsonFloat(config.assistRatio, 2);
    json += ",\"torque_ramp_start_nm\":" + JsonFloat(config.torqueRampStartNm, 1);
    json += ",\"torque_ramp_end_nm\":" + JsonFloat(config.torqueRampEndNm, 1);
    json += ",\"cadence_disable_rpm\":" + JsonFloat(config.cadenceDisableRpm, 1);
    json += ",\"cadence_average_rpm\":" + JsonFloat(config.cadenceAverageRpm, 1);
    json += ",\"fast_filter_alpha\":" + JsonFloat(config.fastFilterAlpha, 2);
    json += ",\"max_current_a\":" + JsonFloat(config.maxCurrentA, 1);
    json += ",\"bounds\":{";
    json += "\"assist_ratio_min\":" + JsonFloat(AppConfig::kPedalAssistRatioMin, 2);
    json += ",\"assist_ratio_max\":" + JsonFloat(AppConfig::kPedalAssistRatioMax, 2);
    json += ",\"torque_ramp_min_nm\":" + JsonFloat(AppConfig::kPedalTorqueRampMinNm, 1);
    json += ",\"torque_ramp_max_nm\":" + JsonFloat(AppConfig::kPedalTorqueRampMaxNm, 1);
    json += ",\"cadence_disable_min_rpm\":" + JsonFloat(AppConfig::kPedalCadenceDisableMinRpm, 1);
    json += ",\"cadence_disable_max_rpm\":" + JsonFloat(AppConfig::kPedalCadenceDisableMaxRpm, 1);
    json += ",\"cadence_average_min_rpm\":" + JsonFloat(AppConfig::kPedalCadenceAverageMinRpm, 1);
    json += ",\"cadence_average_max_rpm\":" + JsonFloat(AppConfig::kPedalCadenceAverageMaxRpm, 1);
    json += ",\"fast_filter_alpha_min\":" + JsonFloat(AppConfig::kPedalFastFilterAlphaMin, 2);
    json += ",\"fast_filter_alpha_max\":" + JsonFloat(AppConfig::kPedalFastFilterAlphaMax, 2);
    json += ",\"max_current_hard_max_a\":" + JsonFloat(AppConfig::kPedalCurrentLimitHardMaxA, 1);
    json += "}";
    json += "}";

    server_.send(200, "application/json", json);
  });

  server_.on("/api/pedal/config", HTTP_POST, [this]() {
    if (!pedalAssistManager_) {
      server_.send(503, "application/json", "{\"ok\":false,\"error\":\"pedal_not_configured\"}");
      return;
    }

    PedalAssistManager::Config config = pedalAssistManager_->config();

    if (!ParseFloatArg(server_, "assist_ratio", &config.assistRatio) ||
        !ParseFloatArg(server_, "torque_ramp_start_nm", &config.torqueRampStartNm) ||
        !ParseFloatArg(server_, "torque_ramp_end_nm", &config.torqueRampEndNm) ||
        !ParseFloatArg(server_, "cadence_disable_rpm", &config.cadenceDisableRpm) ||
        !ParseFloatArg(server_, "cadence_average_rpm", &config.cadenceAverageRpm) ||
        !ParseFloatArg(server_, "fast_filter_alpha", &config.fastFilterAlpha) ||
        !ParseFloatArg(server_, "max_current_a", &config.maxCurrentA)) {
      server_.send(400,
                   "application/json",
                   "{\"ok\":false,\"error\":\"invalid_or_missing_form_fields\"}");
      return;
    }

    String error;
    if (!pedalAssistManager_->setConfig(config, &error)) {
      const String json = "{\"ok\":false,\"error\":\"" + error + "\"}";
      server_.send(400, "application/json", json);
      return;
    }

    String json = "{";
    json += "\"ok\":true";
    if (error.length() > 0) {
      json += ",\"warning\":\"" + error + "\"";
    }
    json += "}";
    server_.send(200, "application/json", json);
  });

  server_.on("/api/pedal/calibrate_zero", HTTP_POST, [this]() {
    if (!pedalAssistManager_) {
      server_.send(503, "application/json", "{\"ok\":false,\"error\":\"pedal_not_configured\"}");
      return;
    }

    String error;
    if (!pedalAssistManager_->calibrateZero(&error)) {
      const String json = "{\"ok\":false,\"error\":\"" + error + "\"}";
      server_.send(400, "application/json", json);
      return;
    }

    server_.send(200, "application/json", "{\"ok\":true}");
  });

  server_.on(
      "/update",
      HTTP_POST,
      [this]() {
        const bool success = !Update.hasError();
        server_.send(success ? 200 : 500,
                     "text/plain",
                     success ? "Update complete. Rebooting..." : "Update failed.");
        if (success) {
          delay(200);
          ESP.restart();
        }
      },
      [this]() {
        HTTPUpload& upload = server_.upload();

        if (upload.status == UPLOAD_FILE_START) {
          Serial.printf("[OTA] Upload start: %s\n", upload.filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_END) {
          if (Update.end(true)) {
            Serial.printf("[OTA] Upload complete: %u bytes\n", upload.totalSize);
          } else {
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_ABORTED) {
          Update.abort();
          Serial.println("[OTA] Upload aborted.");
        }
      });
}

String OtaHttpServer::pageHtml_() const {
  String html;
  html.reserve(13000);

  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Torque Controller Console</title>";
  html += "<style>";
  html += "body{font-family:'IBM Plex Sans',sans-serif;margin:0;padding:1rem;background:linear-gradient(160deg,#eef3ff,#f9fff3);color:#112;}";
  html += ".grid{max-width:860px;margin:1rem auto;display:grid;gap:1rem;grid-template-columns:1fr;}";
  html += ".card{background:#fff;border-radius:14px;padding:1rem 1.2rem;box-shadow:0 8px 24px rgba(0,0,0,.08);border:1px solid #e6ecf5;}";
  html += "h1{margin:.2rem 0 .5rem;font-size:1.4rem;}h2{margin:.2rem 0 .5rem;font-size:1.05rem;}";
  html += "small{display:block;margin:.2rem 0;color:#334;}";
  html += ".row{display:flex;align-items:center;gap:.65rem;flex-wrap:wrap;}";
  html += ".pill{display:inline-block;padding:.25rem .55rem;border-radius:999px;background:#edf4ff;color:#17428a;font-size:.85rem;}";
  html += "input[type=number]{width:130px;padding:.45rem .5rem;border-radius:8px;border:1px solid #bfd3ea;background:#fcfdff;}";
  html += "input[type=file]{display:block;margin:1rem 0;}button{padding:.55rem .9rem;border:0;border-radius:9px;background:#1565c0;color:#fff;font-weight:600;cursor:pointer;}";
  html += "button.alt{background:#00695c;}button:disabled{opacity:.6;cursor:not-allowed;}progress{width:100%;height:16px;}";
  html += "label{font-size:.82rem;color:#334;display:flex;flex-direction:column;gap:.25rem;}";
  html += "#log{margin-top:.8rem;font-size:.95rem;color:#222;}#bridgeLog,#odoLog,#pedalLog{margin-top:.6rem;font-size:.92rem;color:#222;}";
  html += "code{font-family:'IBM Plex Mono',monospace;background:#f2f6fb;padding:.1rem .3rem;border-radius:6px;}";
  html += "@media (max-width:680px){body{padding:.6rem;}h1{font-size:1.2rem;}}";
  html += "</style></head><body>";
  html += "<div class='grid'>";
  html += "<div class='card'><h1>Torque Controller Console</h1>";
  html += "<small><strong>Mode:</strong> " + networkModeLabel_ + "</small>";
  html += "<small><strong>Device IP:</strong> " + activeIp_.toString() + "</small>";
  html += "</div>";

  html += "<div class='card'><h2>VESC TCP Bridge</h2>";
  html += "<div class='row'>";
  html += "<span class='pill' id='bridgeState'>...</span>";
  html += "<span id='bridgeClient'>Client: -</span>";
  html += "<span id='bridgePort'>Port: -</span>";
  html += "</div>";
  html += "<small>Connect VESC Tool over TCP to <code>" + activeIp_.toString() + "</code>.</small>";
  html += "<div class='row' style='margin-top:.7rem'>";
  html += "<button id='bridgeToggle' class='alt' type='button'>Toggle Bridge</button>";
  html += "</div>";
  html += "<div id='bridgeLog'></div>";
  html += "</div>";

  html += "<div class='card'><h2>VESC CAN Status</h2>";
  html += "<div id='vescStatus'>Loading...</div>";
  html += "</div>";

  html += "<div class='card'><h2>Odometer</h2>";
  html += "<div id='odometerStatus'>Loading...</div>";
  html += "<form id='odometerConfigForm' style='margin-top:.85rem'>";
  html += "<div class='row'>";
  html += "<label>Wheel diameter (mm)<input id='wheelDiameterMm' type='number' min='200' max='900' step='1'></label>";
  html += "<label>Debounce (ms)<input id='debounceMs' type='number' min='1' max='250' step='1'></label>";
  html += "<label>Persist interval (ms)<input id='persistMs' type='number' min='5000' max='300000' step='1000'></label>";
  html += "<label>Max speed (km/h)<input id='maxSpeedKmh' type='number' min='1' max='50' step='0.5'></label>";
  html += "</div>";
  html += "<div class='row' style='margin-top:.7rem'><button id='odometerSaveBtn' type='submit'>Save Odometer Config</button></div>";
  html += "</form><div id='odoLog'></div></div>";

  html += "<div class='card'><h2>Pedal Assist</h2>";
  html += "<div id='pedalStatus'>Loading...</div>";
  html += "<form id='pedalConfigForm' style='margin-top:.85rem'>";
  html += "<div class='row'>";
  html += "<label>Assist ratio<input id='assistRatio' type='number' min='0' max='3' step='0.1'></label>";
  html += "<label>Ramp start torque (Nm)<input id='rampStartNm' type='number' min='0' max='180' step='0.5'></label>";
  html += "<label>Ramp end torque (Nm)<input id='rampEndNm' type='number' min='0' max='180' step='0.5'></label>";
  html += "<label>Disable below cadence (rpm)<input id='cadenceDisableRpm' type='number' min='0' max='60' step='0.5'></label>";
  html += "<label>Rotation average cadence (rpm)<input id='cadenceAverageRpm' type='number' min='20' max='180' step='0.5'></label>";
  html += "<label>Fast filter alpha<input id='fastFilterAlpha' type='number' min='0.05' max='1.0' step='0.01'></label>";
  html += "<label>Current limit (A)<input id='maxCurrentA' type='number' min='1' max='70' step='0.5'></label>";
  html += "</div>";
  html += "<div class='row' style='margin-top:.7rem'>";
  html += "<button id='pedalSaveBtn' type='submit'>Save Pedal Config</button>";
  html += "<button id='pedalCalibrateBtn' class='alt' type='button'>Calibrate Zero (No Pedal Load)</button>";
  html += "</div></form><div id='pedalLog'></div></div>";

  html += "<div class='card'><h2>Firmware OTA</h2><form id='f'>";
  html += "<input id='bin' type='file' accept='.bin' required>";
  html += "<button id='btn' type='submit'>Upload Firmware</button>";
  html += "</form><progress id='p' value='0' max='100'></progress><div id='log'></div></div>";
  html += "</div>";

  html += "<script>";
  html += "const form=document.getElementById('f');const log=document.getElementById('log');const p=document.getElementById('p');const btn=document.getElementById('btn');";
  html += "const bridgeState=document.getElementById('bridgeState');const bridgeClient=document.getElementById('bridgeClient');";
  html += "const bridgePort=document.getElementById('bridgePort');const bridgeLog=document.getElementById('bridgeLog');const vescStatus=document.getElementById('vescStatus');";
  html += "const odometerStatus=document.getElementById('odometerStatus');const odoLog=document.getElementById('odoLog');";
  html += "const pedalStatus=document.getElementById('pedalStatus');const pedalLog=document.getElementById('pedalLog');";
  html += "const odometerConfigForm=document.getElementById('odometerConfigForm');const wheelDiameterMm=document.getElementById('wheelDiameterMm');";
  html += "const debounceMs=document.getElementById('debounceMs');const persistMs=document.getElementById('persistMs');const maxSpeedKmh=document.getElementById('maxSpeedKmh');";
  html += "const pedalConfigForm=document.getElementById('pedalConfigForm');const assistRatio=document.getElementById('assistRatio');";
  html += "const rampStartNm=document.getElementById('rampStartNm');const rampEndNm=document.getElementById('rampEndNm');";
  html += "const cadenceDisableRpm=document.getElementById('cadenceDisableRpm');const cadenceAverageRpm=document.getElementById('cadenceAverageRpm');";
  html += "const fastFilterAlpha=document.getElementById('fastFilterAlpha');const maxCurrentA=document.getElementById('maxCurrentA');";
  html += "const pedalCalibrateBtn=document.getElementById('pedalCalibrateBtn');";
  html += "let bridgeEnabled=false;";

  html += "async function refreshBridge(){try{const r=await fetch('/api/bridge');const d=await r.json();if(!d.ok){bridgeLog.textContent='Bridge API unavailable.';return;}";
  html += "bridgeEnabled=!!d.enabled;bridgeState.textContent=bridgeEnabled?'Bridge Enabled':'Bridge Disabled';bridgeState.style.background=bridgeEnabled?'#e8f8ef':'#fff0f0';";
  html += "bridgeState.style.color=bridgeEnabled?'#11663b':'#8c1f1f';bridgeClient.textContent='Client: '+(d.has_client?'connected':'none');bridgePort.textContent='Port: '+d.port;}";
  html += "catch(e){bridgeLog.textContent='Bridge status error: '+e;}}";

  html += "async function refreshVesc(){try{const r=await fetch('/api/vesc/status');const d=await r.json();if(!d.ok){vescStatus.textContent='VESC status unavailable.';return;}";
  html += "if(!d.can_ready){vescStatus.textContent='CAN not ready.';return;}";
  html += "vescStatus.innerHTML='Target ID: <strong>'+d.target_id+'</strong><br>ERPM: <strong>'+d.erpm+'</strong><br>Current: <strong>'+d.current_a+' A</strong><br>Duty: <strong>'+d.duty+'</strong><br>Fresh: <strong>'+((d.status1_valid&&d.status_age_ms<3000)?'yes':'no')+'</strong>';";
  html += "}catch(e){vescStatus.textContent='VESC status error: '+e;}}";

  html += "async function refreshOdometer(){try{const r=await fetch('/api/odometer/status');const d=await r.json();if(!d.ok){odometerStatus.textContent='Odometer status unavailable.';return;}";
  html += "const framState=d.fram_ready?(d.fram_error?'degraded':'ok'):'not ready';";
  html += "odometerStatus.innerHTML='Distance: <strong>'+d.distance_km+' km</strong><br>Speed: <strong>'+d.speed_kmh+' km/h</strong><br>Fresh: <strong>'+(d.speed_fresh?'yes':'no')+'</strong><br>Pulses: <strong>'+d.accepted_pulses+'</strong> accepted / <strong>'+d.rejected_pulses+'</strong> rejected<br>FRAM: <strong>'+framState+'</strong>';";
  html += "}catch(e){odometerStatus.textContent='Odometer error: '+e;}}";

  html += "async function loadOdometerConfig(){try{const r=await fetch('/api/odometer/config');const d=await r.json();if(!d.ok){odoLog.textContent='Odometer config API unavailable.';return;}";
  html += "wheelDiameterMm.value=d.wheel_diameter_mm;debounceMs.value=d.debounce_ms;persistMs.value=d.persist_interval_ms;maxSpeedKmh.value=d.max_speed_kmh;";
  html += "wheelDiameterMm.min=d.bounds.wheel_min;wheelDiameterMm.max=d.bounds.wheel_max;";
  html += "debounceMs.min=d.bounds.debounce_min;debounceMs.max=d.bounds.debounce_max;";
  html += "persistMs.min=d.bounds.persist_min;persistMs.max=d.bounds.persist_max;maxSpeedKmh.max=d.bounds.max_speed_hard;";
  html += "}catch(e){odoLog.textContent='Config load failed: '+e;}}";

  html += "async function refreshPedal(){try{const r=await fetch('/api/pedal/status');const d=await r.json();if(!d.ok){pedalStatus.textContent='Pedal status unavailable.';return;}";
  html += "const mode=d.high_cadence_averaging?'rotation-average':'fast-filter';const safe=d.fail_safe_active?'active':'ok';";
  html += "pedalStatus.innerHTML='Cadence: <strong>'+d.cadence_rpm+' rpm</strong><br>Torque raw/filtered: <strong>'+d.torque_raw_nm+' / '+d.torque_filtered_nm+' Nm</strong><br>Mode: <strong>'+mode+'</strong><br>Ramp: <strong>'+(Math.round(d.ramp_factor*100))+'%</strong><br>Assist ratio: <strong>'+d.assist_ratio+'</strong><br>Cmd torque/current: <strong>'+d.motor_torque_cmd_nm+' Nm / '+d.motor_current_cmd_a+' A</strong><br>Cadence active: <strong>'+(d.cadence_active?'yes':'no')+'</strong><br>Fail-safe: <strong>'+safe+'</strong><br>CAN write: <strong>'+(d.last_can_write_ok?'ok':'failed')+'</strong>';";
  html += "}catch(e){pedalStatus.textContent='Pedal status error: '+e;}}";

  html += "async function loadPedalConfig(){try{const r=await fetch('/api/pedal/config');const d=await r.json();if(!d.ok){pedalLog.textContent='Pedal config API unavailable.';return;}";
  html += "assistRatio.value=d.assist_ratio;rampStartNm.value=d.torque_ramp_start_nm;rampEndNm.value=d.torque_ramp_end_nm;";
  html += "cadenceDisableRpm.value=d.cadence_disable_rpm;cadenceAverageRpm.value=d.cadence_average_rpm;";
  html += "fastFilterAlpha.value=d.fast_filter_alpha;maxCurrentA.value=d.max_current_a;";
  html += "assistRatio.min=d.bounds.assist_ratio_min;assistRatio.max=d.bounds.assist_ratio_max;";
  html += "rampStartNm.min=d.bounds.torque_ramp_min_nm;rampStartNm.max=d.bounds.torque_ramp_max_nm;";
  html += "rampEndNm.min=d.bounds.torque_ramp_min_nm;rampEndNm.max=d.bounds.torque_ramp_max_nm;";
  html += "cadenceDisableRpm.min=d.bounds.cadence_disable_min_rpm;cadenceDisableRpm.max=d.bounds.cadence_disable_max_rpm;";
  html += "cadenceAverageRpm.min=d.bounds.cadence_average_min_rpm;cadenceAverageRpm.max=d.bounds.cadence_average_max_rpm;";
  html += "fastFilterAlpha.min=d.bounds.fast_filter_alpha_min;fastFilterAlpha.max=d.bounds.fast_filter_alpha_max;";
  html += "maxCurrentA.max=d.bounds.max_current_hard_max_a;";
  html += "}catch(e){pedalLog.textContent='Pedal config load failed: '+e;}}";

  html += "document.getElementById('bridgeToggle').addEventListener('click',async()=>{const url=bridgeEnabled?'/api/bridge/disable':'/api/bridge/enable';";
  html += "try{const r=await fetch(url,{method:'POST'});const d=await r.json();bridgeLog.textContent=d.ok?('Bridge '+(d.enabled?'enabled':'disabled')):'Bridge request failed.';}";
  html += "catch(e){bridgeLog.textContent='Bridge toggle failed: '+e;}await refreshBridge();});";

  html += "odometerConfigForm.addEventListener('submit',async(e)=>{e.preventDefault();const payload=new URLSearchParams();";
  html += "payload.append('wheel_diameter_mm',wheelDiameterMm.value);payload.append('debounce_ms',debounceMs.value);";
  html += "payload.append('persist_interval_ms',persistMs.value);payload.append('max_speed_kmh',maxSpeedKmh.value);";
  html += "try{const r=await fetch('/api/odometer/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:payload.toString()});";
  html += "const d=await r.json();if(!d.ok){odoLog.textContent='Save failed: '+(d.error||'unknown_error');return;}";
  html += "odoLog.textContent=d.warning?('Saved with warning: '+d.warning):'Odometer config saved.';await refreshOdometer();}";
  html += "catch(e){odoLog.textContent='Save failed: '+e;}});";

  html += "pedalConfigForm.addEventListener('submit',async(e)=>{e.preventDefault();const payload=new URLSearchParams();";
  html += "payload.append('assist_ratio',assistRatio.value);payload.append('torque_ramp_start_nm',rampStartNm.value);";
  html += "payload.append('torque_ramp_end_nm',rampEndNm.value);payload.append('cadence_disable_rpm',cadenceDisableRpm.value);";
  html += "payload.append('cadence_average_rpm',cadenceAverageRpm.value);payload.append('fast_filter_alpha',fastFilterAlpha.value);";
  html += "payload.append('max_current_a',maxCurrentA.value);";
  html += "try{const r=await fetch('/api/pedal/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:payload.toString()});";
  html += "const d=await r.json();if(!d.ok){pedalLog.textContent='Save failed: '+(d.error||'unknown_error');return;}";
  html += "pedalLog.textContent=d.warning?('Saved with warning: '+d.warning):'Pedal config saved.';await refreshPedal();}";
  html += "catch(e){pedalLog.textContent='Save failed: '+e;}});";

  html += "pedalCalibrateBtn.addEventListener('click',async()=>{";
  html += "pedalLog.textContent='Calibrating zero offset... keep pedals unloaded.';";
  html += "try{const r=await fetch('/api/pedal/calibrate_zero',{method:'POST'});const d=await r.json();";
  html += "if(!d.ok){pedalLog.textContent='Calibration failed: '+(d.error||'unknown_error');return;}";
  html += "pedalLog.textContent='Zero offset calibrated.';await refreshPedal();}";
  html += "catch(e){pedalLog.textContent='Calibration failed: '+e;}});";

  html += "form.addEventListener('submit',e=>{e.preventDefault();const file=document.getElementById('bin').files[0];";
  html += "if(!file){log.textContent='Choose a .bin file first.';return;}const xhr=new XMLHttpRequest();";
  html += "xhr.open('POST','/update',true);xhr.upload.onprogress=(ev)=>{if(ev.lengthComputable){";
  html += "p.value=Math.round((ev.loaded/ev.total)*100);}};xhr.onload=()=>{btn.disabled=false;";
  html += "log.textContent='HTTP '+xhr.status+': '+xhr.responseText;};xhr.onerror=()=>{btn.disabled=false;";
  html += "log.textContent='Upload failed due to network error.';};btn.disabled=true;";
  html += "const data=new FormData();data.append('firmware',file,file.name);xhr.send(data);});";
  html += "refreshBridge();refreshVesc();refreshOdometer();loadOdometerConfig();refreshPedal();loadPedalConfig();";
  html += "setInterval(()=>{refreshBridge();refreshVesc();refreshOdometer();refreshPedal();},1000);";
  html += "</script></body></html>";

  return html;
}
