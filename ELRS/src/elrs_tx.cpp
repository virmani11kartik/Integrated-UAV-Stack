// ELRS Transmitter: UART CRSF in -> OTA pack -> RF
#if defined(BUILD_ELRS_TX)

#include "config.h"
#include "crsf_serial.h"
#include "ota.h"
#include "fhss.h"
#include "radio_sx1280.h"
#include "hw_timer.h"
#include <Arduino.h>
#if ELRS_CLASSIC_BIND
#include <esp_efuse.h>
#endif

static uint8_t s_uid[ELRS_UID_LEN];
static uint16_t s_crsfChannels[CRSF_NUM_CHANNELS];
static uint32_t s_otaChannels[4];  // first 4 for OTA pack
static uint8_t s_syncCounter = 0;
static bool s_sendingSync = false;
#if ELRS_CLASSIC_BIND
static uint8_t s_packetCounter = 0;
#endif

static void txTimerCallback() {
    OTA_Packet4_t pkt;
    bool sendBind = false;
#if ELRS_CLASSIC_BIND
    s_packetCounter++;
    if (s_packetCounter >= ELRS_BIND_INTERVAL_PACKETS) {
        s_packetCounter = 0;
        sendBind = true;
    }
#endif
    if (sendBind) {
        OtaPackBind(&pkt, s_uid);
        Radio.setFrequency(FHSSGetInitialFreq());  // bind on sync channel
    } else if (s_sendingSync || (s_syncCounter % 20 == 0)) {
        OtaPackSync(&pkt, FHSSGetCurrIndex(), ELRS_RF_RATE_ENUM, s_uid);
        s_sendingSync = false;
        s_syncCounter = 0;
    } else {
        for (int i = 0; i < 4; i++) s_otaChannels[i] = s_crsfChannels[i];
        OtaPackRcData(&pkt, s_otaChannels);
    }
    if (!sendBind) OtaGenerateCrc(&pkt);
    Radio.transmit((uint8_t*)&pkt, ELRS_PACKET_SIZE);

    if (!sendBind) {
        OtaNonce++;
        if (OtaNonce % ELRS_FHSS_HOP_INTERVAL == 0)
            Radio.setFrequency(FHSSGetNextFreq());
        else
            Radio.setFrequency(FHSSGetFreqAt(FHSSGetCurrIndex()));
    } else {
        Radio.setFrequency(FHSSGetFreqAt(FHSSGetCurrIndex()));  // restore to curr hop for next pkt
    }

    s_syncCounter++;
}

void ELRS_TX_Setup() {
    memset(s_uid, 0, ELRS_UID_LEN);
#if ELRS_CLASSIC_BIND
    esp_efuse_mac_get_default(s_uid);
#else
    size_t len = strlen(ELRS_BIND_PHRASE);
    if (len > ELRS_UID_LEN) len = ELRS_UID_LEN;
    memcpy(s_uid, ELRS_BIND_PHRASE, len);
#endif

    OtaInitCrcFromUid(s_uid);
    FHSSInit(((uint32_t)s_uid[0] << 24) | ((uint32_t)s_uid[1] << 16) | ((uint32_t)s_uid[2] << 8) | s_uid[3]);

    for (int i = 0; i < CRSF_NUM_CHANNELS; i++)
        s_crsfChannels[i] = CRSF_CHANNEL_VALUE_MID;

    Crsf.begin(&Serial1, CRSF_PIN_TX, CRSF_PIN_RX);

    if (!Radio.begin(s_uid)) {
        Serial.println("[ELRS TX] Radio init failed");
        return;
    }
    Radio.setFrequency(FHSSGetInitialFreq());

    HwTimerInit(txTimerCallback);
    HwTimerSetIntervalUs(ELRS_PACKET_INTERVAL_US);
    HwTimerStart();
    Serial.println("[ELRS TX] Started");
}

void ELRS_TX_Loop() {
    Crsf.update();
    if (Crsf.hasNewChannels()) {
        Crsf.getChannels(s_crsfChannels);
        Crsf.clearNewChannels();
    }
}

#endif
