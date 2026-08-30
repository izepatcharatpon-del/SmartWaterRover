#include "EspNowLink.h"
#include "config.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <stdio.h>
#include <string.h>

static const uint8_t PMK_KEY[16] = {
    'U','S','V','_','P','M','K','_','1','6','b','y','t','e','s','!'
};
static const uint8_t LMK_KEY[16] = {
    'U','S','V','_','L','M','K','_','1','6','b','y','t','e','s','!'
};

QueueHandle_t EspNowLink::commandQueue  = nullptr;
QueueHandle_t EspNowLink::waypointQueue = nullptr;
QueueHandle_t EspNowLink::ackQueue      = nullptr;
uint32_t      EspNowLink::droppedPackets = 0;
uint32_t      EspNowLink::droppedIsrPackets = 0;
uint32_t      EspNowLink::txFailCount  = 0;
uint32_t      EspNowLink::rxAcceptCount = 0;
uint32_t      EspNowLink::dropMac      = 0;
uint32_t      EspNowLink::dropLen      = 0;
uint32_t      EspNowLink::dropCmd      = 0;
EspNowLink::TxDiag EspNowLink::lastTx  = {};
uint32_t      EspNowLink::txSequence   = 0;
uint32_t      EspNowLink::stageSequence = 0;
uint8_t       EspNowLink::_peerMac[6]  = {0};
EventGroupHandle_t EspNowLink::_safetyEvents = nullptr;
SemaphoreHandle_t  EspNowLink::_txDone = nullptr;
volatile bool      EspNowLink::_txOk = false;

static const char* txResultName(EspNowLink::TxResult r) {
    switch (r) {
        case EspNowLink::TX_RESULT_OK:       return "OK";
        case EspNowLink::TX_RESULT_SEND_ERR: return "SEND_ERR";
        case EspNowLink::TX_RESULT_NO_ACK:   return "NO_ACK";
        case EspNowLink::TX_RESULT_TIMEOUT:  return "TIMEOUT";
        default:                             return "?";
    }
}

static const char* espNowErrName(esp_err_t e) {
    switch (e) {
        case ESP_OK:                    return "ESP_OK";
        case ESP_ERR_ESPNOW_NOT_INIT:   return "NOT_INIT";
        case ESP_ERR_ESPNOW_ARG:        return "BAD_ARG";
        case ESP_ERR_ESPNOW_NO_MEM:     return "NO_MEM";
        case ESP_ERR_ESPNOW_NOT_FOUND:  return "NO_PEER";
        case ESP_ERR_ESPNOW_INTERNAL:   return "INTERNAL";
        case ESP_ERR_ESPNOW_IF:         return "BAD_IF";
#ifdef ESP_ERR_ESPNOW_CHAN
        case ESP_ERR_ESPNOW_CHAN:       return "BAD_CHAN";
#endif
        default:                        return "ERR";
    }
}

void EspNowLink::onDataRecv(const uint8_t* mac_addr,
                            const uint8_t* data, int len) {
    // Wi-Fi task context (not a classic ISR) — use task-level queue send.
    if (!mac_addr || memcmp(mac_addr, _peerMac, 6) != 0) {
        droppedPackets++;
        dropMac++;
        return;
    }

    if (len == (int)sizeof(CommandPacket)) {
        CommandPacket cmd;
        memcpy(&cmd, data, sizeof(cmd));
        if (cmd.sys_command > CMD_FORCE_SPIRAL) {
            droppedPackets++;
            dropCmd++;
            return;
        }
        rxAcceptCount++;
        // Set the dedicated event before queueing. Even if the queue is full,
        // navTask observes the latched E-Stop bit.
        if (cmd.sys_command == CMD_EMERGENCY_STOP && _safetyEvents) {
            xEventGroupSetBits(_safetyEvents, ESTOP_EVENT_BIT);
        }
        if (commandQueue) {
            if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
                droppedIsrPackets++;
            }
        }
    } else if (len == (int)sizeof(WaypointArrayPacket)) {
        WaypointArrayPacket wps;
        memcpy(&wps, data, sizeof(wps));
        rxAcceptCount++;
        if (waypointQueue) {
            if (xQueueSend(waypointQueue, &wps, 0) != pdTRUE) {
                droppedIsrPackets++;
            }
        }
    } else {
        droppedPackets++;
        dropLen++;
    }
}

void EspNowLink::onDataSent(const uint8_t* /*mac_addr*/,
                            esp_now_send_status_t status) {
    // Called from the Wi-Fi task (not a classic ISR).
    _txOk = (status == ESP_NOW_SEND_SUCCESS);
    if (_txDone) {
        xSemaphoreGive(_txDone);
    }
}

bool EspNowLink::sendWithConfirm(const uint8_t* data, size_t len) {
    lastTx = {};
    lastTx.result = TX_RESULT_TIMEOUT;
    lastTx.sendErr = ESP_OK;

    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    if (esp_wifi_get_channel(&primary, &second) == ESP_OK) {
        lastTx.wifiChannel = primary;
    }
    lastTx.peerExist = esp_now_is_peer_exist(_peerMac) ? 1 : 0;

    for (int attempt = 0; attempt < ESPNOW_TX_RETRIES; attempt++) {
        lastTx.attemptsUsed = (uint8_t)(attempt + 1);
        if (_txDone) {
            // Drain a stale give from a previous send.
            xSemaphoreTake(_txDone, 0);
        }
        _txOk = false;
        const esp_err_t err = esp_now_send(_peerMac, data, len);
        lastTx.sendErr = err;
        if (err != ESP_OK) {
            lastTx.sendErrCount++;
            lastTx.result = TX_RESULT_SEND_ERR;
            continue;
        }
        if (_txDone &&
            xSemaphoreTake(_txDone, pdMS_TO_TICKS(ESPNOW_TX_TIMEOUT_MS)) == pdTRUE) {
            if (_txOk) {
                lastTx.result = TX_RESULT_OK;
                lastTx.sendErr = ESP_OK;
                return true;
            }
            lastTx.noAckCount++;
            lastTx.result = TX_RESULT_NO_ACK;
        } else if (!_txDone) {
            // Callback not available — treat esp_now_send OK as best-effort success.
            lastTx.result = TX_RESULT_OK;
            lastTx.sendErr = ESP_OK;
            return true;
        } else {
            lastTx.timeoutCount++;
            lastTx.result = TX_RESULT_TIMEOUT;
        }
    }
    txFailCount++;
    return false;
}

const uint8_t* EspNowLink::peerMac() {
    return _peerMac;
}

const char* EspNowLink::describeLastTx() {
    static char buf[128];
    snprintf(buf, sizeof(buf),
             "why=%s send=%s tries=%u noAck=%u sendErr=%u tmo=%u ch=%u peer=%u",
             txResultName(lastTx.result),
             espNowErrName(lastTx.sendErr),
             (unsigned)lastTx.attemptsUsed,
             (unsigned)lastTx.noAckCount,
             (unsigned)lastTx.sendErrCount,
             (unsigned)lastTx.timeoutCount,
             (unsigned)lastTx.wifiChannel,
             (unsigned)lastTx.peerExist);
    return buf;
}

const char* EspNowLink::pairingHint() {
    static char buf[192];
    if (lastTx.result == TX_RESULT_OK) {
        snprintf(buf, sizeof(buf), "TX OK");
        return buf;
    }
    if (lastTx.wifiChannel != 0 &&
        lastTx.wifiChannel != (uint8_t)ESPNOW_WIFI_CHANNEL) {
        snprintf(buf, sizeof(buf),
                 "WiFi ch=%u but firmware wants %u — set gateway to channel %u",
                 (unsigned)lastTx.wifiChannel,
                 (unsigned)ESPNOW_WIFI_CHANNEL,
                 (unsigned)ESPNOW_WIFI_CHANNEL);
        return buf;
    }
    switch (lastTx.result) {
        case TX_RESULT_SEND_ERR:
            snprintf(buf, sizeof(buf),
                     "esp_now_send rejected (%s). Boat-side peer/init/memory, not gateway app logic",
                     espNowErrName(lastTx.sendErr));
            break;
        case TX_RESULT_NO_ACK:
            snprintf(buf, sizeof(buf),
                     "NO_ACK: boat sent OK, gateway did not ACK. Match MAC, WiFi ch=%u, PMK/LMK, encrypt, LR, WIFI_PS_NONE both sides",
                     (unsigned)ESPNOW_WIFI_CHANNEL);
            break;
        case TX_RESULT_TIMEOUT:
            snprintf(buf, sizeof(buf),
                     "TIMEOUT: no send callback in %u ms x %u tries — radio stack, not packet contents",
                     (unsigned)ESPNOW_TX_TIMEOUT_MS,
                     (unsigned)ESPNOW_TX_RETRIES);
            break;
        default:
            snprintf(buf, sizeof(buf), "TX failed");
            break;
    }
    return buf;
}

bool EspNowLink::begin(uint8_t channel, const uint8_t peerMac[6]) {
    memcpy(_peerMac, peerMac, 6);

    commandQueue  = xQueueCreate(8, sizeof(CommandPacket));
    waypointQueue = xQueueCreate(4, sizeof(WaypointArrayPacket));
    ackQueue      = xQueueCreate(8, sizeof(WaypointACKPacket));
    _safetyEvents = xEventGroupCreate();
    _txDone       = xSemaphoreCreateBinary();
    if (!commandQueue || !waypointQueue || !ackQueue || !_safetyEvents || !_txDone) {
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                                       WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    // STA default is modem sleep; ESP-NOW unicast ACK is lost if the radio
    // sleeps before the peer's ACK. Both ends need WIFI_PS_NONE.
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (esp_now_init() != ESP_OK) return false;

    esp_now_set_pmk(PMK_KEY);
    esp_now_register_recv_cb(EspNowLink::onDataRecv);
    esp_now_register_send_cb(EspNowLink::onDataSent);

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, _peerMac, 6);
    peer.channel = channel;
    peer.encrypt = true;
    memcpy(peer.lmk, LMK_KEY, 16);
    peer.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peer) != ESP_OK) return false;
    return true;
}

bool EspNowLink::sendTelemetry(TelemetryPacket& pkt) {
    pkt.sequence_number = ++txSequence;
    return sendWithConfirm((const uint8_t*)&pkt, sizeof(pkt));
}

bool EspNowLink::sendAck(const WaypointACKPacket& pkt) {
    return sendWithConfirm((const uint8_t*)&pkt, sizeof(pkt));
}

bool EspNowLink::sendBatteryReport(const BatteryReportPacket& pkt) {
    return sendWithConfirm((const uint8_t*)&pkt, sizeof(pkt));
}

bool EspNowLink::sendStageStatus(StageStatusPacket& pkt) {
    pkt.sequence_number = ++stageSequence;
    return sendWithConfirm((const uint8_t*)&pkt, sizeof(pkt));
}

bool EspNowLink::emergencyPending() {
    if (!_safetyEvents) return false;
    return (xEventGroupGetBits(_safetyEvents) & ESTOP_EVENT_BIT) != 0;
}

void EspNowLink::clearEmergencyPending() {
    if (_safetyEvents) {
        xEventGroupClearBits(_safetyEvents, ESTOP_EVENT_BIT);
    }
}
