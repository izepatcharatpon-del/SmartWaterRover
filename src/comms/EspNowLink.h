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
    bool sendStageStatus(StageStatusPacket& pkt);

    static QueueHandle_t commandQueue;
    static QueueHandle_t waypointQueue;
    static QueueHandle_t ackQueue;

    static uint32_t droppedPackets;
    static uint32_t droppedIsrPackets;
    static uint32_t txFailCount;
    static uint32_t rxAcceptCount;
    static uint32_t dropMac;
    static uint32_t dropLen;
    static uint32_t dropCmd;

    enum TxResult : uint8_t {
        TX_RESULT_OK = 0,
        TX_RESULT_SEND_ERR,
        TX_RESULT_NO_ACK,
        TX_RESULT_TIMEOUT
    };

    struct TxDiag {
        TxResult  result;
        esp_err_t sendErr;
        uint8_t   attemptsUsed;
        uint8_t   noAckCount;
        uint8_t   sendErrCount;
        uint8_t   timeoutCount;
        uint8_t   wifiChannel;
        uint8_t   peerExist;
    };

    static TxDiag lastTx;
    static const uint8_t* peerMac();
    static const char* describeLastTx();
    static const char* pairingHint();

    static bool emergencyPending();
    static void clearEmergencyPending();

private:
    static uint8_t  _peerMac[6];
    static uint32_t txSequence;
    static uint32_t stageSequence;
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
