#include "EspNowLink.h"
#include "config.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
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
uint32_t      EspNowLink::txSequence   = 0;
uint32_t      EspNowLink::stageSequence = 0;
uint8_t       EspNowLink::_peerMac[6]  = {0};
EventGroupHandle_t EspNowLink::_safetyEvents = nullptr;
SemaphoreHandle_t  EspNowLink::_txDone = nullptr;
volatile bool      EspNowLink::_txOk = false;

void EspNowLink::onDataRecv(const uint8_t* mac_addr,
                            const uint8_t* data, int len) {
    // Wi-Fi task context (not a classic ISR) — use task-level queue send.
    if (!mac_addr || memcmp(mac_addr, _peerMac, 6) != 0) {
        droppedPackets++;
        return;
    }

    if (len == (int)sizeof(CommandPacket)) {
        CommandPacket cmd;
        memcpy(&cmd, data, sizeof(cmd));
        if (cmd.sys_command > CMD_MISSION_UPLOAD_DONE) {
            droppedPackets++;
            return;
        }
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
        if (waypointQueue) {
            if (xQueueSend(waypointQueue, &wps, 0) != pdTRUE) {
                droppedIsrPackets++;
            }
        }
    } else {
        droppedPackets++;
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
    for (int attempt = 0; attempt < ESPNOW_TX_RETRIES; attempt++) {
        if (_txDone) {
            // Drain a stale give from a previous send.
            xSemaphoreTake(_txDone, 0);
        }
        _txOk = false;
        if (esp_now_send(_peerMac, data, len) != ESP_OK) {
            continue;
        }
        if (_txDone &&
            xSemaphoreTake(_txDone, pdMS_TO_TICKS(ESPNOW_TX_TIMEOUT_MS)) == pdTRUE) {
            if (_txOk) return true;
        } else if (!_txDone) {
            // Callback not available — treat esp_now_send OK as best-effort success.
            return true;
        }
    }
    txFailCount++;
    return false;
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
