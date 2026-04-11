#include "vesc.h"

// ---------- Minimal VESC UART protocol implementation ----------
// Only implements setCurrent command.

// VESC COMM packet commands
#define COMM_SET_CURRENT 6

// CRC16/XMODEM (poly 0x1021, init 0, same as VESC uses)
static uint16_t crc16(const uint8_t *buf, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

// Send a VESC packet: [0x02][len][payload][crc_hi][crc_lo][0x03]
static void vescSendPacket(const uint8_t *payload, uint8_t len) {
    uint16_t c = crc16(payload, len);
    uint8_t pkt[len + 5];
    pkt[0] = 0x02;
    pkt[1] = len;
    memcpy(&pkt[2], payload, len);
    pkt[2 + len] = (uint8_t)(c >> 8);
    pkt[3 + len] = (uint8_t)(c & 0xFF);
    pkt[4 + len] = 0x03;
    Serial2.write(pkt, len + 5);
    Serial2.flush();
}

// ---------- Public API ----------

void vescInit(const AppConfig &cfg) {
    Serial2.begin(cfg.vesc_baud, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN);
}

void vescSetCurrent(float amps, const AppConfig &cfg) {
    if (amps < 0.0f) amps = 0.0f;
    if (amps > cfg.max_motor_current) amps = cfg.max_motor_current;

    int32_t mA = (int32_t)(amps * 1000.0f);
    uint8_t buf[5];
    buf[0] = COMM_SET_CURRENT;
    buf[1] = (mA >> 24) & 0xFF;
    buf[2] = (mA >> 16) & 0xFF;
    buf[3] = (mA >> 8) & 0xFF;
    buf[4] = mA & 0xFF;
    vescSendPacket(buf, 5);
}

HardwareSerial &vescGetSerial() {
    return Serial2;
}
