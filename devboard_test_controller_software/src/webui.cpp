#include "webui.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ElegantOTA.h>
#include <ArduinoJson.h>

static AsyncWebServer server(80);
static AppConfig *cfgPtr = nullptr;
static LiveStatus liveStatus = {};
static ConfigChangedCb onConfigChanged = nullptr;
static BleBridgeToggleCb onBleToggle = nullptr;
static bool apMode = false;

// ---------- HTML UI (PROGMEM) ----------
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lauryno dviratis</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#1a1a2e;color:#e0e0e0;padding:16px;max-width:600px;margin:0 auto}
h1{color:#0ff;margin-bottom:12px;font-size:1.4em}
h2{color:#0cf;margin:18px 0 8px;font-size:1.1em;border-bottom:1px solid #333;padding-bottom:4px}
.card{background:#16213e;border-radius:8px;padding:14px;margin-bottom:12px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.val{font-size:1.6em;font-weight:700;color:#0f0}
.unit{font-size:0.7em;color:#888;margin-left:2px}
label{display:block;margin:8px 0 3px;font-size:0.9em;color:#aaa}
input[type=number],input[type=text],input[type=password]{width:100%;padding:8px;border:1px solid #333;border-radius:4px;background:#0d1b2a;color:#fff;font-size:1em}
input[type=range]{width:100%;margin:4px 0}
.row{display:flex;align-items:center;gap:8px}
.row span{min-width:3em;text-align:right;color:#0f0;font-weight:700}
button{background:#0cf;color:#000;border:none;padding:10px 20px;border-radius:4px;font-size:1em;cursor:pointer;margin-top:8px}
button:hover{background:#0af}
.toggle{position:relative;width:48px;height:26px;display:inline-block}
.toggle input{opacity:0;width:0;height:0}
.toggle .slider{position:absolute;inset:0;background:#333;border-radius:13px;transition:.3s}
.toggle input:checked+.slider{background:#0c6}
.toggle .slider::before{content:'';position:absolute;width:20px;height:20px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:.3s}
.toggle input:checked+.slider::before{transform:translateX(22px)}
.status-dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px}
.ok{background:#0f0}.err{background:#f44}
#msg{color:#0f0;margin-top:8px;font-size:0.9em;min-height:1.2em}
</style>
</head>
<body>
<h1>&#9889; Lauryno dviratis</h1>

<div class="card">
<h2>Live Status</h2>
<div class="grid">
<div>Torque<br><span class="val" id="tq">--</span><span class="unit">Nm</span></div>
<div>Cadence<br><span class="val" id="cad">--</span><span class="unit">RPM</span></div>
<div>Rider Power<br><span class="val" id="pw">--</span><span class="unit">W</span></div>
<div>Motor Current<br><span class="val" id="mc">--</span><span class="unit">A</span></div>
<div>BLE Bridge<br><span class="val" id="ble">--</span></div>
<div>Torque Sensor<br><span class="val" id="tsv">--</span><span class="unit">V</span></div>
<div>ESP32 Temp<br><span class="val" id="espt">--</span><span class="unit">°C</span></div>
<div>Torque Assist<br><span class="val" id="ta">--</span></div>
</div>
</div>

<div class="card">
<h2>Assist Settings</h2>
<label>Assist Ratio</label>
<input type="number" id="ar" step="0.01" min="0" max="1" value="1">
<label>Max Motor Current (A)</label>
<input type="number" id="mmc" step="1" min="1" max="150" value="30">
<label>Min Cadence RPM</label>
<input type="number" id="mcr" step="1" min="0" max="60" value="15">
<label>Current Ramp Rate (A/s)</label>
<input type="number" id="crr" step="1" min="1" max="100" value="10">
<label>Assist Start Torque (Nm) &mdash; no assist below</label>
<input type="number" id="asnm" step="0.5" min="0" max="200" value="5">
<label>Assist Full Torque (Nm) &mdash; full assist above</label>
<input type="number" id="afnm" step="0.5" min="0" max="200" value="20">
</div>

<div class="card">
<h2>Sensor Calibration</h2>
<label>Torque Sensor Min Voltage (V)</label>
<input type="number" id="tminv" step="0.01" value="0.75">
<label>Torque Sensor Max Voltage (V)</label>
<input type="number" id="tmaxv" step="0.01" value="5.0">
<label>Max Torque (Nm)</label>
<input type="number" id="tmaxnm" step="1" value="140">
<label>Divider Ratio R2/(R1+R2)</label>
<input type="number" id="divr" step="0.0001" value="0.6623">
<label>Cadence PPR</label>
<input type="number" id="ppr" step="1" min="1" max="256" value="32">
<label>Torque EMA Alpha (0.01=smooth, 1.0=raw)</label>
<div class="row">
<input type="range" id="emaa" min="0.01" max="1" step="0.01" value="0.3">
<span id="emaav">0.30</span>
</div>
</div>

<div class="card">
<h2>Motor / VESC</h2>
<label>Flux Linkage (Wb, from VESC Tool)</label>
<input type="number" id="flnk" step="0.001" min="0.001" max="1" value="0.01">
<label>UART Baud Rate</label>
<input type="number" id="vbaud" step="1" value="115200">
<label class="row" style="margin-top:10px">
<label class="toggle"><input type="checkbox" id="tqassist"><span class="slider"></span></label>
Torque-sensor assist (uncheck for throttle passthrough)
</label>
</div>

<div class="card">
<h2>WiFi</h2>
<label>SSID (STA mode)</label>
<input type="text" id="ssid" maxlength="32" autocomplete="off">
<label>Password (STA)</label>
<input type="password" id="wpass" maxlength="64" autocomplete="off">
<label>AP Password</label>
<input type="password" id="apass" maxlength="64" autocomplete="off">
</div>

<div class="card">
<h2>BLE VESC Bridge</h2>
<label class="row">
<label class="toggle"><input type="checkbox" id="bletog"><span class="slider"></span></label>
Enable BLE-UART Bridge
</label>
<p style="font-size:0.8em;color:#888;margin-top:6px">When a BLE client connects, the motor control loop pauses to give VESC Tool exclusive UART access.</p>
</div>

<button onclick="saveConfig()">Save Settings</button>
<span id="msg"></span>

<div class="card" style="margin-top:16px">
<h2>Firmware Update</h2>
<p>Go to <a href="/update" style="color:#0cf">/update</a> for OTA firmware upload.</p>
</div>

<script>
const $=id=>document.getElementById(id);
const ar=$('ar'),arv=$('arv');
ar.oninput=()=>arv.textContent=parseFloat(ar.value).toFixed(1);
const emaa=$('emaa'),emaav=$('emaav');
emaa.oninput=()=>emaav.textContent=parseFloat(emaa.value).toFixed(2);

async function poll(){
  try{
    const r=await fetch('/api/status');
    const d=await r.json();
    $('tq').textContent=d.torque_nm.toFixed(1);
    $('cad').textContent=d.cadence_rpm.toFixed(0);
    $('pw').textContent=d.rider_power_w.toFixed(0);
    $('mc').textContent=d.motor_current_a.toFixed(1);
    $('ble').textContent=d.ble_bridge_active?'Active':'Off';
    $('ble').style.color=d.ble_bridge_active?'#f90':'#888';
    $('tsv').textContent=d.torque_sensor_v.toFixed(3);
    $('espt').textContent=d.esp_temp_c.toFixed(1);
    $('ta').textContent=d.torque_assist_enabled?'On':'Off';
    $('ta').style.color=d.torque_assist_enabled?'#0f0':'#888';
  }catch(e){}
}
setInterval(poll,1000);
poll();

async function loadConfig(){
  try{
    const r=await fetch('/api/config');
    const c=await r.json();
    ar.value=c.assist_ratio; arv.textContent=parseFloat(c.assist_ratio).toFixed(1);
    $('mmc').value=c.max_motor_current;
    $('mcr').value=c.min_cadence_rpm;
    $('crr').value=c.current_ramp_rate;
    $('asnm').value=c.assist_start_nm;
    $('afnm').value=c.assist_full_nm;
    $('tminv').value=c.torque_min_v;
    $('tmaxv').value=c.torque_max_v;
    $('tmaxnm').value=c.torque_max_nm;
    $('divr').value=c.divider_ratio;
    $('ppr').value=c.cadence_ppr;
    emaa.value=c.torque_ema_alpha; emaav.textContent=parseFloat(c.torque_ema_alpha).toFixed(2);
    $('flnk').value=c.flux_linkage;
    $('vbaud').value=c.vesc_baud;
    $('ssid').value=c.wifi_ssid||'';
    $('wpass').value='';
    $('apass').value='';
    $('bletog').checked=!!c.ble_bridge_enabled;
    $('tqassist').checked=!!c.torque_assist_enabled;
  }catch(e){$('msg').textContent='Failed to load config';}
}
loadConfig();

async function saveConfig(){
  const body={
    assist_ratio:parseFloat(ar.value),
    max_motor_current:parseFloat($('mmc').value),
    min_cadence_rpm:parseFloat($('mcr').value),
    current_ramp_rate:parseFloat($('crr').value),
    assist_start_nm:parseFloat($('asnm').value),
    assist_full_nm:parseFloat($('afnm').value),
    torque_min_v:parseFloat($('tminv').value),
    torque_max_v:parseFloat($('tmaxv').value),
    torque_max_nm:parseFloat($('tmaxnm').value),
    divider_ratio:parseFloat($('divr').value),
    cadence_ppr:parseInt($('ppr').value),
    torque_ema_alpha:parseFloat(emaa.value),
    flux_linkage:parseFloat($('flnk').value),
    vesc_baud:parseInt($('vbaud').value),
    wifi_ssid:$('ssid').value,
    wifi_pass:$('wpass').value,
    ap_pass:$('apass').value,
    ble_bridge_enabled:$('bletog').checked,
    torque_assist_enabled:$('tqassist').checked
  };
  try{
    const r=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    if(r.ok){$('msg').textContent='Saved! Some changes take effect after reboot.';$('msg').style.color='#0f0';}
    else{$('msg').textContent='Save failed';$('msg').style.color='#f44';}
  }catch(e){$('msg').textContent='Network error';$('msg').style.color='#f44';}
  setTimeout(()=>$('msg').textContent='',4000);
}
</script>
</body>
</html>
)rawliteral";

// ---------- WiFi ----------
static void wifiSetup(AppConfig &cfg) {
    if (strlen(cfg.wifi_ssid) > 0) {
        Serial.printf("[WiFi] Connecting to %s...\n", cfg.wifi_ssid);
        WiFi.mode(WIFI_STA);
        WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
        uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - start) < 10000) {
            delay(250);
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[WiFi] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
            apMode = false;
            return;
        }
        Serial.println("[WiFi] STA failed, falling back to AP");
    }

    // AP fallback
    WiFi.mode(WIFI_AP);
    const char *apPass = strlen(cfg.ap_pass) >= 8 ? cfg.ap_pass : "ebike123";
    WiFi.softAP("Lauryno_dviratis", apPass);
    apMode = true;
    Serial.printf("[WiFi] AP mode, IP: %s\n", WiFi.softAPIP().toString().c_str());
}

// ---------- API ----------
static void setupApi(AppConfig &cfg) {
    // Serve HTML
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "text/html", INDEX_HTML);
    });

    // Live status
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["torque_nm"]              = liveStatus.torque_nm;
        doc["torque_sensor_v"]        = liveStatus.torque_sensor_v;
        doc["cadence_rpm"]            = liveStatus.cadence_rpm;
        doc["rider_power_w"]          = liveStatus.rider_power_w;
        doc["motor_current_a"]        = liveStatus.motor_current_a;
        doc["esp_temp_c"]             = liveStatus.esp_temp_c;
        doc["ble_bridge_active"]      = liveStatus.ble_bridge_active;
        doc["torque_assist_enabled"]  = liveStatus.torque_assist_enabled;
        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // Get config
    server.on("/api/config", HTTP_GET, [&cfg](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["assist_ratio"]      = cfg.assist_ratio;
        doc["max_motor_current"] = cfg.max_motor_current;
        doc["min_cadence_rpm"]   = cfg.min_cadence_rpm;
        doc["current_ramp_rate"] = cfg.current_ramp_rate;
        doc["assist_start_nm"]   = cfg.assist_start_nm;
        doc["assist_full_nm"]    = cfg.assist_full_nm;
        doc["torque_min_v"]      = cfg.torque_min_v;
        doc["torque_max_v"]      = cfg.torque_max_v;
        doc["torque_max_nm"]     = cfg.torque_max_nm;
        doc["divider_ratio"]     = cfg.divider_ratio;
        doc["cadence_ppr"]       = cfg.cadence_ppr;
        doc["torque_ema_alpha"]  = cfg.torque_ema_alpha;
        doc["flux_linkage"]      = cfg.flux_linkage;
        doc["vesc_baud"]         = cfg.vesc_baud;
        doc["wifi_ssid"]         = cfg.wifi_ssid;
        // Don't send passwords
        doc["ble_bridge_enabled"] = cfg.ble_bridge_enabled;
        doc["torque_assist_enabled"] = cfg.torque_assist_enabled;
        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // Update config (POST with JSON body)
    AsyncCallbackJsonWebHandler *postHandler = new AsyncCallbackJsonWebHandler(
        "/api/config",
        [&cfg](AsyncWebServerRequest *req, JsonVariant &json) {
            JsonObject obj = json.as<JsonObject>();
            bool bleChanged = false;
            bool prevBle = cfg.ble_bridge_enabled;

            if (obj.containsKey("assist_ratio"))      cfg.assist_ratio      = obj["assist_ratio"].as<float>();
            if (obj.containsKey("max_motor_current"))  cfg.max_motor_current = obj["max_motor_current"].as<float>();
            if (obj.containsKey("min_cadence_rpm"))    cfg.min_cadence_rpm   = obj["min_cadence_rpm"].as<float>();
            if (obj.containsKey("current_ramp_rate"))  cfg.current_ramp_rate = obj["current_ramp_rate"].as<float>();
            if (obj.containsKey("assist_start_nm"))    cfg.assist_start_nm   = obj["assist_start_nm"].as<float>();
            if (obj.containsKey("assist_full_nm"))     cfg.assist_full_nm    = obj["assist_full_nm"].as<float>();
            if (obj.containsKey("torque_min_v"))       cfg.torque_min_v      = obj["torque_min_v"].as<float>();
            if (obj.containsKey("torque_max_v"))       cfg.torque_max_v      = obj["torque_max_v"].as<float>();
            if (obj.containsKey("torque_max_nm"))      cfg.torque_max_nm     = obj["torque_max_nm"].as<float>();
            if (obj.containsKey("divider_ratio"))      cfg.divider_ratio     = obj["divider_ratio"].as<float>();
            if (obj.containsKey("cadence_ppr"))        cfg.cadence_ppr       = obj["cadence_ppr"].as<uint16_t>();
            if (obj.containsKey("torque_ema_alpha"))   cfg.torque_ema_alpha  = obj["torque_ema_alpha"].as<float>();
            if (obj.containsKey("flux_linkage"))       cfg.flux_linkage      = obj["flux_linkage"].as<float>();
            if (obj.containsKey("vesc_baud"))          cfg.vesc_baud         = obj["vesc_baud"].as<uint32_t>();

            if (obj.containsKey("wifi_ssid")) {
                strlcpy(cfg.wifi_ssid, obj["wifi_ssid"].as<const char*>(), sizeof(cfg.wifi_ssid));
            }
            if (obj.containsKey("wifi_pass") && strlen(obj["wifi_pass"].as<const char*>()) > 0) {
                strlcpy(cfg.wifi_pass, obj["wifi_pass"].as<const char*>(), sizeof(cfg.wifi_pass));
            }
            if (obj.containsKey("ap_pass") && strlen(obj["ap_pass"].as<const char*>()) > 0) {
                strlcpy(cfg.ap_pass, obj["ap_pass"].as<const char*>(), sizeof(cfg.ap_pass));
            }
            if (obj.containsKey("ble_bridge_enabled")) {
                cfg.ble_bridge_enabled = obj["ble_bridge_enabled"].as<bool>();
                bleChanged = (cfg.ble_bridge_enabled != prevBle);
            }
            if (obj.containsKey("torque_assist_enabled")) {
                cfg.torque_assist_enabled = obj["torque_assist_enabled"].as<bool>();
            }

            // Clamp dangerous values
            if (cfg.assist_ratio < 0.00f)      cfg.assist_ratio = 0.00f;
            if (cfg.assist_ratio > 10.00f)     cfg.assist_ratio = 10.00f;
            if (cfg.max_motor_current < 0.0f) cfg.max_motor_current = 0.0f;
            if (cfg.max_motor_current > 150.0f) cfg.max_motor_current = 150.0f;
            if (cfg.flux_linkage < 0.001f)    cfg.flux_linkage = 0.001f;
            if (cfg.flux_linkage > 1.0f)      cfg.flux_linkage = 1.0f;
            if (cfg.torque_ema_alpha < 0.01f)  cfg.torque_ema_alpha = 0.01f;
            if (cfg.torque_ema_alpha > 1.0f)   cfg.torque_ema_alpha = 1.0f;

            configSave(cfg);

            if (onConfigChanged) onConfigChanged();
            if (bleChanged && onBleToggle) onBleToggle(cfg.ble_bridge_enabled);

            req->send(200, "application/json", "{\"ok\":true}");
        }
    );
    server.addHandler(postHandler);
}

// ---------- Public API ----------
void webuiInit(AppConfig &cfg) {
    cfgPtr = &cfg;
    wifiSetup(cfg);
    setupApi(cfg);
    ElegantOTA.begin(&server);
    server.begin();
    Serial.println("[WebUI] Server started");
}

void webuiLoop() {
    ElegantOTA.loop();
}

void webuiSetStatus(const LiveStatus &s) {
    liveStatus = s;
}

void webuiOnConfigChanged(ConfigChangedCb cb)    { onConfigChanged = cb; }
void webuiOnBleBridgeToggle(BleBridgeToggleCb cb) { onBleToggle = cb; }
