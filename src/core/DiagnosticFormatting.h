#ifndef USV_DIAGNOSTIC_FORMATTING_H
#define USV_DIAGNOSTIC_FORMATTING_H

#include <stddef.h>
#include <stdint.h>

namespace DiagnosticFormatting {

enum class PhReadReason : uint8_t {
    NO_SAMPLE = 0,
    OK,
    ADS_UNAVAILABLE,
    ADC_INVALID,
    COMPUTATION_INVALID,
    ELECTRODE_RANGE,
    TEMP_INVALID,
    PH_IMPLAUSIBLE
};

enum class PhSuppressionReason : uint8_t {
    NONE = 0,
    NOT_QUIET_WINDOW,
    MODE_SUPPRESSED,
    MOTORS_NOT_STOPPED,
    NO_FINITE_SAMPLE
};

enum class TxResult : uint8_t {
    ACK = 0,
    SEND_ERR,
    NO_ACK,
    TIMEOUT,
    UNKNOWN
};

struct PhLogRecord {
    uint32_t sampleId;
    int16_t rawAdc;
    uint32_t sampleMs;
    uint32_t sampleAgeMs;
    bool samplePresent;
    bool newSample;
    float adcVoltage;
    float sensorPh;
    bool qualityValid;
    uint16_t quietCount;
    float sensorTempC;
    float effectiveTempC;
    bool temperatureValid;
    const char* calibrationSource;
    PhReadReason readReason;
    PhSuppressionReason suppressionReason;
    const char* utcSource;
    bool utcAvailable;
    bool utcAgeKnown;
    uint32_t utcAgeMs;
};

struct TxLogRecord {
    uint32_t sequence;
    uint32_t sendStartMs;
    uint32_t sendEndMs;
    double latitude;
    double longitude;
    float ph;
    float turbidity;
    float temperatureC;
    uint8_t mode;
    uint8_t batteryPercent;
    float headingDeg;
    TxResult result;
    int32_t sendError;
    uint8_t attempts;
    uint8_t noAckCount;
    uint8_t sendErrorCount;
    uint8_t timeoutCount;
    uint8_t wifiChannel;
    bool peerExists;
};

const char* phReadReasonName(PhReadReason reason);
const char* phSuppressionReasonName(PhSuppressionReason reason);
const char* txResultName(TxResult result);

// Formats the 18 appended SD CSV fields, without changing legacy field order.
bool formatPhCsvSuffix(const PhLogRecord& record, char* output, size_t outputSize);
// Formats one complete TX ledger row matching the TX ledger header.
bool formatTxCsvRow(const TxLogRecord& record, char* output, size_t outputSize);

} // namespace DiagnosticFormatting

#endif // USV_DIAGNOSTIC_FORMATTING_H
