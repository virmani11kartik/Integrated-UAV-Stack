// ELRS Receiver: RF -> OTA unpack -> UART CRSF out
#if defined(BUILD_ELRS_RX)

#include "config.h"
#include "crsf_serial.h"
#include "ota.h"
#include "fhss.h"
#include "radio_sx1280.h"
#include "hw_timer.h"
#include <Arduino.h>
#include <Preferences.h>
#if ELRS_CLASSIC_BIND
#define ELRS_NVS_NAMESPACE "elrs"
#define ELRS_NVS_UID_KEY   "uid"
#endif

static uint8_t s_uid[ELRS_UID_LEN];
static uint32_t s_channelData[CRSF_NUM_CHANNELS];
static uint16_t s_crsfChannels[CRSF_NUM_CHANNELS];
static bool s_locked = false;
static uint32_t s_lastPacketMs = 0;
#if ELRS_CLASSIC_BIND
static bool s_bindMode = false;  // listening for BIND packet
#endif

static void rxTimerCallback() {
    if (!s_locked) return;
    Crsf.sendRcChannels(s_crsfChannels);  // Send at packet rate when locked
}

#if ELRS_CLASSIC_BIND
static bool loadStoredUid(uint8_t* uid) {
    Preferences prefs;
    if (!prefs.begin(ELRS_NVS_NAMESPACE, true)) return false;
    bool ok = prefs.getBytesLength(ELRS_NVS_UID_KEY) == ELRS_UID_LEN;
    if (ok) ok = prefs.getBytes(ELRS_NVS_UID_KEY, uid, ELRS_UID_LEN) == ELRS_UID_LEN;
    prefs.end();
    return ok;
}
static void storeUid(const uint8_t* uid) {
    Preferences prefs;
    prefs.begin(ELRS_NVS_NAMESPACE, false);
    prefs.putBytes(ELRS_NVS_UID_KEY, uid, ELRS_UID_LEN);
    prefs.end();
}
#endif

void ELRS_RX_Setup() {
    memset(s_uid, 0, ELRS_UID_LEN);
#if ELRS_CLASSIC_BIND
    if (loadStoredUid(s_uid)) {
        s_bindMode = false;
        Serial.println("[ELRS RX] Using stored UID (bound)");
    } else {
        s_bindMode = true;
        memset(s_uid, 0, ELRS_UID_LEN);  // will be filled by BIND packet
        Serial.println("[ELRS RX] Bind mode: waiting for TX...");
    }
#else
    size_t len = strlen(ELRS_BIND_PHRASE);
    if (len > ELRS_UID_LEN) len = ELRS_UID_LEN;
    memcpy(s_uid, ELRS_BIND_PHRASE, len);
#endif

#if ELRS_CLASSIC_BIND
    if (!s_bindMode)
#endif
    {
        OtaInitCrcFromUid(s_uid);
        FHSSInit(((uint32_t)s_uid[0] << 24) | ((uint32_t)s_uid[1] << 16) | ((uint32_t)s_uid[2] << 8) | s_uid[3]);
    }

    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        s_channelData[i] = CRSF_CHANNEL_VALUE_MID;
        s_crsfChannels[i] = CRSF_CHANNEL_VALUE_MID;
    }

    Crsf.begin(&Serial1, CRSF_PIN_TX, CRSF_PIN_RX);

    if (!Radio.begin(s_uid)) {
        Serial.println("[ELRS RX] Radio init failed");
        return;
    }
    Radio.setFrequency(FHSSGetInitialFreq());  // sync channel for both bind and initial sync
    Radio.startReceive();

    HwTimerInit(rxTimerCallback);
    HwTimerSetIntervalUs(ELRS_PACKET_INTERVAL_US);
    HwTimerStart();
#if ELRS_CLASSIC_BIND
    Serial.println(s_bindMode ? "[ELRS RX] Listening for BIND on sync channel" : "[ELRS RX] Started, waiting for sync");
#else
    Serial.println("[ELRS RX] Started, waiting for sync");
#endif
}

void ELRS_RX_Loop() {
#if ELRS_CLASSIC_BIND
    if (s_bindMode) {
        if (Radio.isRxDone()) {
            OTA_Packet4_t pkt;
            if (Radio.readPacket((uint8_t*)&pkt, ELRS_PACKET_SIZE)) {
                if (OtaValidateBindPacket(&pkt, s_uid)) {
                    storeUid(s_uid);
                    Serial.println("[ELRS RX] Bound! Restarting with new UID...");
                    delay(100);
                    ESP.restart();
                }
            }
            Radio.startReceive();
        }
        return;
    }
#endif
    if (Radio.isRxDone()) {
        OTA_Packet4_t pkt;
        if (Radio.readPacket((uint8_t*)&pkt, ELRS_PACKET_SIZE) && OtaValidateCrc(&pkt)) {
            s_lastPacketMs = millis();
            if ((pkt.type & 0x03) == PACKET_TYPE_SYNC) {
                FHSSSetCurrIndex(pkt.sync.fhssIndex);
                OtaNonce = pkt.sync.nonce;
                s_locked = true;
            } else if ((pkt.type & 0x03) == PACKET_TYPE_RCDATA && s_locked) {
                if (OtaUnpackRcData(&pkt, s_channelData)) {
                    for (int i = 0; i < CRSF_NUM_CHANNELS; i++)
                        s_crsfChannels[i] = (uint16_t)s_channelData[i];
                }
                OtaNonce++;
                if (OtaNonce % ELRS_FHSS_HOP_INTERVAL == 0)
                    Radio.setFrequency(FHSSGetNextFreq());
                else
                    Radio.setFrequency(FHSSGetFreqAt(FHSSGetCurrIndex()));
            }
        }
        Radio.startReceive();
    }

    if (s_locked && (millis() - s_lastPacketMs > 500))
        s_locked = false;
}

#endif
