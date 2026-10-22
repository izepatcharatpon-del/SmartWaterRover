#include "core/DiagnosticFormatting.h"

#include <math.h>
#include <stdio.h>

namespace DiagnosticFormatting {

const char* phReadReasonName(PhReadReason reason) {
    switch (reason) {
        case PhReadReason::NO_SAMPLE: return "NO_SAMPLE";
        case PhReadReason::OK: return "OK";
        case PhReadReason::ADS_UNAVAILABLE: return "ADS_UNAVAILABLE";
        case PhReadReason::ADC_INVALID: return "ADC_INVALID";
        case PhReadReason::COMPUTATION_INVALID: return "COMPUTATION_INVALID";
        case PhReadReason::ELECTRODE_RANGE: return "ELECTRODE_RANGE";
        case PhReadReason::TEMP_INVALID: return "TEMP_INVALID";
        case PhReadReason::PH_IMPLAUSIBLE: return "PH_IMPLAUSIBLE";
        default: return "UNKNOWN";
    }
}

const char* phSuppressionReasonName(PhSuppressionReason reason) {
    switch (reason) {
        case PhSuppressionReason::NONE: return "NONE";
        case PhSuppressionReason::NOT_QUIET_WINDOW: return "NOT_QUIET_WINDOW";
        case PhSuppressionReason::MODE_SUPPRESSED: return "MODE_SUPPRESSED";
        case PhSuppressionReason::MOTORS_NOT_STOPPED: return "MOTORS_NOT_STOPPED";
        case PhSuppressionReason::NO_FINITE_SAMPLE: return "NO_FINITE_SAMPLE";
        default: return "UNKNOWN";
    }
}

const char* txResultName(TxResult result) {
    switch (result) {
        case TxResult::ACK: return "ACK";
        case TxResult::SEND_ERR: return "SEND_ERR";
        case TxResult::NO_ACK: return "NO_ACK";
        case TxResult::TIMEOUT: return "TIMEOUT";
        default: return "UNKNOWN";
    }
}

bool formatPhCsvSuffix(const PhLogRecord& record, char* output, size_t outputSize) {
    if (!output || outputSize == 0) return false;
    char sampleId[16] = "";
    char rawAdc[16] = "";
    char sampleMs[16] = "";
    char sampleAgeMs[16] = "";
    char utcAgeMs[16] = "";
    if (record.samplePresent) {
        snprintf(sampleId, sizeof(sampleId), "%lu", (unsigned long)record.sampleId);
        snprintf(rawAdc, sizeof(rawAdc), "%d", (int)record.rawAdc);
        snprintf(sampleMs, sizeof(sampleMs), "%lu", (unsigned long)record.sampleMs);
        snprintf(sampleAgeMs, sizeof(sampleAgeMs), "%lu", (unsigned long)record.sampleAgeMs);
    }
    if (record.utcAgeKnown) {
        snprintf(utcAgeMs, sizeof(utcAgeMs), "%lu", (unsigned long)record.utcAgeMs);
    }
    const int written = snprintf(
        output, outputSize,
        ",%s,%s,%s,%s,%u,%.3f,%.3f,%u,%u,%.3f,%.3f,%u,%s,%s,%s,%s,%s,%s",
        sampleId,
        rawAdc,
        sampleMs,
        sampleAgeMs,
        record.newSample ? 1U : 0U,
        (double)record.adcVoltage,
        (double)record.sensorPh,
        record.qualityValid ? 1U : 0U,
        (unsigned)record.quietCount,
        (double)record.sensorTempC,
        (double)record.effectiveTempC,
        record.temperatureValid ? 1U : 0U,
        record.calibrationSource ? record.calibrationSource : "UNKNOWN",
        phReadReasonName(record.readReason),
        phSuppressionReasonName(record.suppressionReason),
        record.utcSource ? record.utcSource : "NONE",
        record.utcAvailable ? "AVAILABLE" : "UNAVAILABLE",
        utcAgeMs);
    if (written < 0 || (size_t)written >= outputSize) {
        output[outputSize - 1] = '\0';
        return false;
    }
    return true;
}

bool formatTxCsvRow(const TxLogRecord& record, char* output, size_t outputSize) {
    if (!output || outputSize == 0) return false;
    const uint32_t duration = record.sendEndMs - record.sendStartMs;
    const int written = snprintf(
        output, outputSize,
        "%lu,%lu,%lu,%lu,%.6f,%.6f,%.3f,%.3f,%.3f,%u,%u,%.3f,%s,%ld,%u,%u,%u,%u,%u,%u\n",
        (unsigned long)record.sequence,
        (unsigned long)record.sendStartMs,
        (unsigned long)record.sendEndMs,
        (unsigned long)duration,
        record.latitude,
        record.longitude,
        (double)record.ph,
        (double)record.turbidity,
        (double)record.temperatureC,
        (unsigned)record.mode,
        (unsigned)record.batteryPercent,
        (double)record.headingDeg,
        txResultName(record.result),
        (long)record.sendError,
        (unsigned)record.attempts,
        (unsigned)record.noAckCount,
        (unsigned)record.sendErrorCount,
        (unsigned)record.timeoutCount,
        (unsigned)record.wifiChannel,
        record.peerExists ? 1U : 0U);
    if (written < 0 || (size_t)written >= outputSize) {
        output[outputSize - 1] = '\0';
        return false;
    }
    return true;
}

} // namespace DiagnosticFormatting
