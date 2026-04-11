#include "ble_bridge.h"
#include "vesc.h"

#include <NimBLEDevice.h>

// Nordic UART Service UUIDs (VESC Tool compatible)
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // write (phone → ESP)
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // notify (ESP → phone)

static NimBLEServer *pServer = nullptr;
static NimBLECharacteristic *pTxChar = nullptr;
static NimBLECharacteristic *pRxChar = nullptr;
static bool bleClientConnected = false;
static bool bleInitialized = false;
static bool bleAdvertising = false;

// Relay buffer from BLE RX → UART
#define BLE_BUF_SIZE 512
static uint8_t bleTxBuf[BLE_BUF_SIZE];

// --- Callbacks ---
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *s) override {
        bleClientConnected = true;
        Serial.println("[BLE] Client connected — control loop paused");
    }
    void onDisconnect(NimBLEServer *s) override {
        bleClientConnected = false;
        Serial.println("[BLE] Client disconnected — control loop resumed");
        if (bleAdvertising) {
            NimBLEDevice::startAdvertising();
        }
    }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar) override {
        // Relay BLE RX data → VESC UART TX
        std::string val = pChar->getValue();
        if (val.length() > 0) {
            vescGetSerial().write((const uint8_t *)val.data(), val.length());
        }
    }
};

// --- Public API ---
void bleBridgeInit() {
    if (bleInitialized) return;

    NimBLEDevice::init("Lauryno_dviratis");
    NimBLEDevice::setMTU(512);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *pService = pServer->createService(NUS_SERVICE_UUID);

    pTxChar = pService->createCharacteristic(
        NUS_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    pRxChar = pService->createCharacteristic(
        NUS_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pRxChar->setCallbacks(new RxCallbacks());

    pService->start();

    bleInitialized = true;
    Serial.println("[BLE] Bridge initialized");
}

void bleBridgeStart() {
    if (!bleInitialized) bleBridgeInit();
    if (bleAdvertising) return;

    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(NUS_SERVICE_UUID);
    pAdv->setScanResponse(true);
    NimBLEDevice::startAdvertising();
    bleAdvertising = true;
    Serial.println("[BLE] Advertising started");
}

void bleBridgeStop() {
    if (!bleInitialized) return;

    NimBLEDevice::stopAdvertising();
    bleAdvertising = false;

    // Disconnect any connected client
    if (pServer && bleClientConnected) {
        pServer->disconnect(0);
        bleClientConnected = false;
    }
    Serial.println("[BLE] Bridge stopped");
}

bool bleBridgeConnected() {
    return bleClientConnected;
}

void bleBridgeLoop() {
    if (!bleClientConnected || !pTxChar) return;

    // Relay VESC UART RX → BLE TX notify
    HardwareSerial &uart = vescGetSerial();
    size_t avail = uart.available();
    if (avail > 0) {
        if (avail > BLE_BUF_SIZE) avail = BLE_BUF_SIZE;
        size_t len = uart.readBytes(bleTxBuf, avail);
        if (len > 0) {
            pTxChar->setValue(bleTxBuf, len);
            pTxChar->notify();
        }
    }
}
