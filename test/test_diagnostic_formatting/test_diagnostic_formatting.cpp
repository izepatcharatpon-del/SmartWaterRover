#include <unity.h>
#include <string.h>

#include "core/DiagnosticFormatting.h"
#include "Packets.h"

using namespace DiagnosticFormatting;

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

void test_ph_formatter_logs_sample_identity_age_quality_and_utc_provenance() {
    PhLogRecord record{};
    record.sampleId = 7;
    record.rawAdc = 32767;
    record.sampleMs = 1500;
    record.sampleAgeMs = 125;
    record.samplePresent = true;
    record.newSample = false;
    record.adcVoltage = 1.725f;
    record.sensorPh = 11.8f;
    record.qualityValid = false;
    record.quietCount = 3;
    record.sensorTempC = 31.5f;
    record.effectiveTempC = 31.5f;
    record.temperatureValid = true;
    record.calibrationSource = "UNKNOWN";
    record.readReason = PhReadReason::ELECTRODE_RANGE;
    record.suppressionReason = PhSuppressionReason::NOT_QUIET_WINDOW;
    record.utcSource = "GPS";
    record.utcAvailable = true;
    record.utcAgeKnown = true;
    record.utcAgeMs = 800;

    char line[256];
    TEST_ASSERT_TRUE(formatPhCsvSuffix(record, line, sizeof(line)));
    TEST_ASSERT_NOT_NULL(strstr(line, ",7,32767,1500,125,0,1.725,11.800,0,3,31.500,31.500,1,UNKNOWN,ELECTRODE_RANGE,NOT_QUIET_WINDOW,GPS,AVAILABLE,800"));
}

void test_ph_formatter_marks_missing_time_and_nonfinite_values_explicitly() {
    PhLogRecord record{};
    record.samplePresent = false;
    record.adcVoltage = NAN;
    record.sensorPh = NAN;
    record.sensorTempC = NAN;
    record.effectiveTempC = 25.0f;
    record.calibrationSource = "UNKNOWN";
    record.readReason = PhReadReason::ADS_UNAVAILABLE;
    record.suppressionReason = PhSuppressionReason::NO_FINITE_SAMPLE;
    record.utcSource = "NONE";
    record.utcAgeKnown = false;

    char line[256];
    TEST_ASSERT_TRUE(formatPhCsvSuffix(record, line, sizeof(line)));
    TEST_ASSERT_NOT_NULL(strstr(line, ",,,,,0,nan,nan,0,0,nan,25.000,0,UNKNOWN,ADS_UNAVAILABLE,NO_FINITE_SAMPLE,NONE,UNAVAILABLE,"));
}

void test_repeated_sd_observations_keep_one_adc_identity_and_age_changes() {
    PhLogRecord record{};
    record.sampleId = 19;
    record.rawAdc = 1234;
    record.sampleMs = 500;
    record.samplePresent = true;
    record.adcVoltage = 1.7f;
    record.sensorPh = 11.9f;
    record.calibrationSource = "UNKNOWN";
    record.readReason = PhReadReason::ELECTRODE_RANGE;
    record.suppressionReason = PhSuppressionReason::NOT_QUIET_WINDOW;
    char first[256], repeated[256];
    record.sampleAgeMs = 40;
    record.newSample = true;
    TEST_ASSERT_TRUE(formatPhCsvSuffix(record, first, sizeof(first)));
    record.sampleAgeMs = 90;
    record.newSample = false;
    TEST_ASSERT_TRUE(formatPhCsvSuffix(record, repeated, sizeof(repeated)));
    TEST_ASSERT_NOT_NULL(strstr(first, ",19,1234,500,40,1,"));
    TEST_ASSERT_NOT_NULL(strstr(repeated, ",19,1234,500,90,0,"));
    TEST_ASSERT_EQUAL_UINT(38, sizeof(TelemetryPacket));
}

void test_tx_formatter_records_actual_payload_result_retries_and_bounds() {
    TxLogRecord record{};
    record.sequence = 42;
    record.sendStartMs = 0xFFFFFFF0u;
    record.sendEndMs = 25;
    record.latitude = 20.123456;
    record.longitude = 99.654321;
    record.ph = NAN;
    record.turbidity = 12.5f;
    record.temperatureC = 31.25f;
    record.mode = 1;
    record.batteryPercent = 88;
    record.headingDeg = 270.5f;
    record.result = TxResult::NO_ACK;
    record.sendError = 0;
    record.attempts = 3;
    record.noAckCount = 3;
    record.sendErrorCount = 0;
    record.timeoutCount = 0;
    record.wifiChannel = 6;
    record.peerExists = true;

    char line[256];
    TEST_ASSERT_TRUE(formatTxCsvRow(record, line, sizeof(line)));
    TEST_ASSERT_NOT_NULL(strstr(line, "42,4294967280,25,41,20.123456,99.654321,nan,12.500,31.250,1,88,270.500,NO_ACK,0,3,3,0,0,6,1"));

    char tooSmall[4];
    TEST_ASSERT_FALSE(formatTxCsvRow(record, tooSmall, sizeof(tooSmall)));
    TEST_ASSERT_EQUAL_CHAR('\0', tooSmall[sizeof(tooSmall) - 1]);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_ph_formatter_logs_sample_identity_age_quality_and_utc_provenance);
    RUN_TEST(test_ph_formatter_marks_missing_time_and_nonfinite_values_explicitly);
    RUN_TEST(test_repeated_sd_observations_keep_one_adc_identity_and_age_changes);
    RUN_TEST(test_tx_formatter_records_actual_payload_result_retries_and_bounds);
    return UNITY_END();
}
