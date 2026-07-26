#ifndef USV_ESPNOW_LINK_H
#define USV_ESPNOW_LINK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <esp_now.h>
#include "Packets.h"

class EspNowLink {
public:
    bool begin(uint8_t channel, const uint8_t peerMac[6]);

    bool sendTelemetry(TelemetryPacket& pkt);
    bool sendAck(const WaypointACKPacket& pkt);
    bool sendBatteryReport(const BatteryReportPacket& pkt);

    // Inbound queues — drained by navigation task (waypoints) / IO task (commands).
    static QueueHandle_t commandQueue;
    static QueueHandle_t waypointQueue;
    // Outbound ACK requests from nav task -> IO task.
    static QueueHandle_t ackQueue;

    static uint32_t droppedPackets;
    static uint32_t droppedIsrPackets;
    static uint32_t txFailCount;

    // Dedicated latched safety path: an E-Stop cannot be lost when commandQueue is full.
    static bool emergencyPending();
    static void clearEmergencyPending();

private:
    static uint8_t  _peerMac[6];
    static uint32_t txSequence; // IO-task owned
    static EventGroupHandle_t _safetyEvents;
    static SemaphoreHandle_t  _txDone;
    static volatile bool      _txOk;
    enum : EventBits_t { ESTOP_EVENT_BIT = (1U << 0) };

    static void onDataRecv(const uint8_t* mac_addr,
                           const uint8_t* data, int len);
    static void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
    static bool sendWithConfirm(const uint8_t* data, size_t len);
};

#endif // USV_ESPNOW_LINK_H
