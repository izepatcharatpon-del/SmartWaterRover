#include "storage/DataLogger.h"
#include "HostTestSupport.h"

#include <cstdio>
#include <cstring>
#include <string>

static unsigned failures = 0;

static void check(bool condition, const char* description) {
    std::printf("ASSERT_%s: %s\n", condition ? "PASS" : "FAIL", description);
    if (!condition) ++failures;
}

static DiagnosticFormatting::TxLogRecord makeRecord() {
    DiagnosticFormatting::TxLogRecord record{};
    record.sequence = 42;
    record.sendStartMs = 1000;
    record.sendEndMs = 1037;
    record.latitude = 20.123456;
    record.longitude = 99.654321;
    record.ph = 6.86f;
    record.turbidity = 12.5f;
    record.temperatureC = 31.25f;
    record.mode = 1;
    record.batteryPercent = 88;
    record.headingDeg = 270.5f;
    record.result = DiagnosticFormatting::TxResult::NO_ACK;
    record.sendError = -7;
    record.attempts = 3;
    record.noAckCount = 2;
    record.sendErrorCount = 1;
    record.timeoutCount = 0;
    record.wifiChannel = 6;
    record.peerExists = true;
    return record;
}

static bool sameRecord(const DiagnosticFormatting::TxLogRecord& a,
                       const DiagnosticFormatting::TxLogRecord& b) {
    return a.sequence == b.sequence && a.sendStartMs == b.sendStartMs &&
        a.sendEndMs == b.sendEndMs && a.latitude == b.latitude &&
        a.longitude == b.longitude && a.ph == b.ph &&
        a.turbidity == b.turbidity && a.temperatureC == b.temperatureC &&
        a.mode == b.mode && a.batteryPercent == b.batteryPercent &&
        a.headingDeg == b.headingDeg && a.result == b.result &&
        a.sendError == b.sendError && a.attempts == b.attempts &&
        a.noAckCount == b.noAckCount && a.sendErrorCount == b.sendErrorCount &&
        a.timeoutCount == b.timeoutCount && a.wifiChannel == b.wifiChannel &&
        a.peerExists == b.peerExists;
}

static void checkFailureIsDiagnosticOnly(DataLogger& logger,
                                         uint32_t legacyFailuresBefore) {
    check(logger.isReady(), "TX diagnostic failure preserves logger readiness");
    check(logger.writeFailCount() == legacyFailuresBefore,
          "TX diagnostic failure leaves legacy writer counter unchanged");
}

static DataLogger makeReadyLogger() {
    DataLogger logger;
    check(logger.begin(4.01f, 6.86f, 9.18f, true, "HOST_TEST"),
          "logger begins with host-only SD backend");
    check(logger.isReady(), "legacy logger remains ready in host fixture");
    return logger;
}

static void testUnavailableFile() {
    HostTest::reset();
    HostTest::txUnavailable = true;
    DataLogger logger = makeReadyLogger();
    const auto record = makeRecord();
    const auto before = record;
    const uint32_t failuresBefore = logger.txDiagnosticWriteFailCount();
    const uint32_t legacyFailuresBefore = logger.writeFailCount();
    check(!logger.logTxDiagnostic(record), "unavailable TX sidecar returns false");
    check(logger.txDiagnosticWriteFailCount() == failuresBefore + 1,
          "unavailable file increments diagnostic failure counter once");
    checkFailureIsDiagnosticOnly(logger, legacyFailuresBefore);
    check(HostTest::txBytes().empty(), "unavailable file writes no TX bytes");
    check(sameRecord(record, before), "unavailable-file path preserves immutable record");
}

static void testLockFailure() {
    HostTest::reset();
    DataLogger logger = makeReadyLogger();
    const auto record = makeRecord();
    const auto before = record;
    const std::string bytesBefore = HostTest::txBytes();
    const uint32_t failuresBefore = logger.txDiagnosticWriteFailCount();
    const uint32_t legacyFailuresBefore = logger.writeFailCount();
    HostTest::failNextMutexTake = true;
    check(!logger.logTxDiagnostic(record), "mutex-take failure returns false");
    check(logger.txDiagnosticWriteFailCount() == failuresBefore + 1,
          "lock failure increments diagnostic counter once");
    checkFailureIsDiagnosticOnly(logger, legacyFailuresBefore);
    check(HostTest::txBytes() == bytesBefore, "lock failure writes no bytes");
    check(sameRecord(record, before), "lock-failure path preserves immutable record");
}

static void testShortWrite() {
    HostTest::reset();
    DataLogger logger = makeReadyLogger();
    const auto record = makeRecord();
    const auto before = record;
    char expected[320];
    check(DiagnosticFormatting::formatTxCsvRow(record, expected, sizeof(expected)),
          "short-write expected row formats");
    const std::string bytesBefore = HostTest::txBytes();
    const uint32_t failuresBefore = logger.txDiagnosticWriteFailCount();
    const uint32_t legacyFailuresBefore = logger.writeFailCount();
    HostTest::shortTxWrite = true;
    check(!logger.logTxDiagnostic(record), "short SD write returns false");
    check(logger.txDiagnosticWriteFailCount() == failuresBefore + 1,
          "short write increments diagnostic counter once");
    checkFailureIsDiagnosticOnly(logger, legacyFailuresBefore);
    const std::string bytesAfter = HostTest::txBytes();
    check(bytesAfter.size() == bytesBefore.size() + std::strlen(expected) - 1 &&
          bytesAfter.compare(bytesBefore.size(), std::strlen(expected) - 1,
                             expected, std::strlen(expected) - 1) == 0,
          "short-write fixture records one byte less than intended row");
    check(sameRecord(record, before), "short-write path preserves immutable record");
}

static void testSuccessfulWrite() {
    HostTest::reset();
    DataLogger logger = makeReadyLogger();
    const auto record = makeRecord();
    const auto before = record;
    char expected[320];
    check(DiagnosticFormatting::formatTxCsvRow(record, expected, sizeof(expected)),
          "expected row formatter accepts host record");
    const std::string bytesBefore = HostTest::txBytes();
    const uint32_t failuresBefore = logger.txDiagnosticWriteFailCount();
    const uint32_t legacyFailuresBefore = logger.writeFailCount();
    check(logger.logTxDiagnostic(record), "actual DataLogger TX writer succeeds");
    const std::string bytesAfter = HostTest::txBytes();
    check(bytesAfter.size() == bytesBefore.size() + std::strlen(expected) &&
          bytesAfter.compare(bytesBefore.size(), std::strlen(expected), expected) == 0,
          "success writes exact formatted telemetry-diagnostic row");
    check(logger.txDiagnosticWriteFailCount() == failuresBefore,
          "success does not increment diagnostic failure counter");
    check(logger.writeFailCount() == legacyFailuresBefore,
          "success does not change legacy writer failure counter");
    check(sameRecord(record, before), "success path preserves immutable record/payload");
}

int main() {
    std::puts("HOST-ONLY fixture: actual DataLogger.cpp::logTxDiagnostic; no radio, receiver, boat, or pond execution");
    testUnavailableFile();
    testLockFailure();
    testShortWrite();
    testSuccessfulWrite();
    std::printf("TX_WRITER_ASSERTION_FAILURES=%u\n", failures);
    return failures == 0 ? 0 : 1;
}
