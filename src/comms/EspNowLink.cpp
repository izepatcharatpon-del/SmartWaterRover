#include "EspNowLink.h"
#include "config.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---------------------------------------------------------------------------
// SHARED SECRETS — must be identical on the shore/gateway board.
// TODO: replace these placeholders with the values agreed with the gateway.
// ---------------------------------------------------------------------------
static const uint8_t PMK_KEY[16] = {
    'U','S','V','_','P','M','K','_','1','6','b','y','t','e','s','!'
};
static const uint8_t LMK_KEY[16] = {
    'U','S','V','_','L','M','K','_','1','6','b','y','t','e','s','!'
};

// Static member definitions
QueueHandle_t EspNowLink::commandQueue  = nullptr;
QueueHandle_t EspNowLink::waypointQueue = nullptr;
uint32_t      EspNowLink::droppedPackets = 0;
uint8_t       EspNowLink::_peerMac[6]   = {0};
uint32_t      EspNowLink::_txSequence   = 0;

void EspNowLink::onDataRecv(const uint8_t* mac_addr,
                            const uint8_t* data, int len) {
    (void)mac_addr;
    BaseType_t higherWoken = pdFALSE;

    // Demux purely by wire length (contract-defined, static_assert'd sizes).
    if (len == (int)sizeof(CommandPacket)) {
        CommandPacket cmd;
        memcpy(&cmd, data, sizeof(cmd));
        if (commandQueue) {
            xQueueSendFromISR(commandQueue, &cmd, &higherWoken);
        }
    } else if (len == (int)sizeof(WaypointArrayPacket)) {
        WaypointArrayPacket wps;
        memcpy(&wps, data, sizeof(wps));
        if (waypointQueue) {
            xQueueSendFromISR(waypointQueue, &wps, &higherWoken);
        }
    } else {
        droppedPackets++;   // unknown/foreign packet size -> discard
    }

    if (higherWoken == pdTRUE) portYIELD_FROM_ISR();
}

bool EspNowLink::begin(uint8_t channel, const uint8_t peerMac[6]) {
    memcpy(_peerMac, peerMac, 6);

    commandQueue  = xQueueCreate(8, sizeof(CommandPacket));
    waypointQueue = xQueueCreate(4, sizeof(WaypointArrayPacket));
    if (!commandQueue || !waypointQueue) return false;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Lock a fixed channel so both ends always match (no scanning).
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    // Long-range mode for extended over-water range (must match the gateway).
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                                       WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);

    if (esp_now_init() != ESP_OK) return false;

    esp_now_set_pmk(PMK_KEY);
    esp_now_register_recv_cb(EspNowLink::onDataRecv);

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
    pkt.sequence_number = ++_txSequence;   // monotonic, replay protection
    return esp_now_send(_peerMac, (const uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}

bool EspNowLink::sendAck(const WaypointACKPacket& pkt) {
    return esp_now_send(_peerMac, (const uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}

bool EspNowLink::sendBatteryReport(const BatteryReportPacket& pkt) {
    return esp_now_send(_peerMac, (const uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
}
