#ifndef USV_ESPNOW_LINK_H
#define USV_ESPNOW_LINK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "Packets.h"

// Boat-side half of the ESP-NOW Interface Contract (see plan section 10).
// - locks the Wi-Fi channel + enables long-range (LR) mode
// - registers a single encrypted peer (the shore gateway)
// - RX callback demuxes inbound packets by size into FreeRTOS queues
// - TX helpers stamp/emit telemetry (seq++) and waypoint ACKs
//
// The gateway (friend's project) MUST mirror this contract; we do NOT code it.
class EspNowLink {
public:
    bool begin(uint8_t channel, const uint8_t peerMac[6]);

    bool sendTelemetry(TelemetryPacket& pkt);   // stamps sequence_number
    bool sendAck(const WaypointACKPacket& pkt);
    bool sendBatteryReport(const BatteryReportPacket& pkt);

    // Inbound queues (created in begin), drained by the IO task.
    static QueueHandle_t commandQueue;    // holds CommandPacket
    static QueueHandle_t waypointQueue;   // holds WaypointArrayPacket

    static uint32_t droppedPackets;       // malformed / unknown length

private:
    static uint8_t  _peerMac[6];
    static uint32_t _txSequence;

    static void onDataRecv(const uint8_t* mac_addr,
                           const uint8_t* data, int len);
};

#endif // USV_ESPNOW_LINK_H
