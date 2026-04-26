#include "network_manager.h"

NetworkManager::NetworkManager(const char* staSsid,
                               const char* staPassword,
                               const char* apSsidPrefix,
                               const char* apPassword,
                               uint32_t staConnectTimeoutMs)
    : staSsid_(staSsid),
      staPassword_(staPassword),
      apSsidPrefix_(apSsidPrefix),
      apPassword_(apPassword),
      staConnectTimeoutMs_(staConnectTimeoutMs),
      mode_(NetworkMode::AccessPoint) {}

NetworkMode NetworkManager::begin() {
  if (connectToStation_()) {
    mode_ = NetworkMode::Station;
    return mode_;
  }

  startFallbackAp_();
  mode_ = NetworkMode::AccessPoint;
  return mode_;
}

NetworkMode NetworkManager::mode() const {
  return mode_;
}

IPAddress NetworkManager::ipAddress() const {
  if (mode_ == NetworkMode::Station) {
    return WiFi.localIP();
  }
  return WiFi.softAPIP();
}

String NetworkManager::accessPointSsid() const {
  return apSsid_;
}

bool NetworkManager::connectToStation_() {
  Serial.printf("[NET] Connecting to STA SSID: %s\n", staSsid_);

  WiFi.mode(WIFI_STA);
  WiFi.begin(staSsid_, staPassword_);

  const uint32_t startedAtMs = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startedAtMs) < staConnectTimeoutMs_) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[NET] STA connected. IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
  }

  Serial.println("[NET] STA connect timed out. Switching to AP fallback.");
  WiFi.disconnect(true, true);
  return false;
}

void NetworkManager::startFallbackAp_() {
  const uint32_t chipId = static_cast<uint32_t>(ESP.getEfuseMac());
  apSsid_ = String(apSsidPrefix_) + "-" + String(chipId, HEX);

  WiFi.mode(WIFI_AP);

  bool started = false;
  if (strlen(apPassword_) >= 8) {
    started = WiFi.softAP(apSsid_.c_str(), apPassword_);
  } else {
    started = WiFi.softAP(apSsid_.c_str());
  }

  if (started) {
    Serial.printf("[NET] AP fallback ready. SSID: %s IP: %s\n",
                  apSsid_.c_str(),
                  WiFi.softAPIP().toString().c_str());
    return;
  }

  Serial.println("[NET] Failed to start fallback AP.");
}
