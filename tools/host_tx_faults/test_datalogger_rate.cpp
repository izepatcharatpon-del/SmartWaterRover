#include "storage/DataLogger.h"
#include "HostTestSupport.h"
#include <cstdio>
#include <deque>
#include <fstream>
#include <string>
#include <algorithm>

static unsigned failures = 0;
static void check(bool value, const char* message) { if (!value) ++failures; std::printf("ASSERT_%s: %s\n", value ? "PASS" : "FAIL", message); }
static SdLogRecord row(unsigned seq, unsigned base) {
    SdLogRecord r{};
    r.boot_ms = base + (seq - 1) * 50; r.sequence_number = seq;
    r.gps_lat = 20.0 + (seq % 100) * .000001; r.gps_lon = 99.0;
    r.water_ph = 6.86f; r.water_turbidity = 12.5f; r.water_temp = 30;
    r.ph_voltage = 2.5f; r.turb_voltage = 1.5f; r.heading_angle = seq % 360;
    r.gps_hdop = .6f; r.gps_sats = 12; r.current_mode = 1; r.battery_percent = 90;
    r.utc_valid = 1; r.utc_year = 2026; r.utc_month = 10; r.utc_day = 15;
    unsigned sec = (seq - 1) / 20; r.utc_hour = 12; r.utc_minute = sec / 60; r.utc_second = sec % 60;
    r.ph_diagnostic.sampleId = (seq - 1) / 5 + 1; r.ph_diagnostic.samplePresent = true;
    r.ph_diagnostic.newSample = (seq - 1) % 5 == 0;
    r.ph_diagnostic.rawAdc = 12345; r.ph_diagnostic.sampleMs = base + ((seq - 1) / 5) * 250;
    r.ph_diagnostic.sampleAgeMs = ((seq - 1) % 5) * 50;
    r.ph_diagnostic.adcVoltage = 2.5; r.ph_diagnostic.sensorPh = 6.86;
    r.ph_diagnostic.temperatureValid = true; r.ph_diagnostic.sensorTempC = r.ph_diagnostic.effectiveTempC = 30;
    r.ph_diagnostic.calibrationSource = "UNKNOWN"; r.ph_diagnostic.utcSource = "GPS";
    r.ph_diagnostic.utcAvailable = true; r.ph_diagnostic.utcAgeKnown = true;
    return r;
}
static DiagnosticFormatting::TxLogRecord tx(unsigned seq, unsigned base) {
    DiagnosticFormatting::TxLogRecord r{}; r.sequence = seq;
    r.sendStartMs = base + (seq - 1) * 1000; r.sendEndMs = r.sendStartMs + 37;
    r.latitude = 20; r.longitude = 99; r.ph = 6.86; r.turbidity = 12.5; r.temperatureC = 30;
    r.mode = 1; r.result = DiagnosticFormatting::TxResult::NO_ACK; r.attempts = 3; r.noAckCount = 3;
    return r;
}
static std::string data(const std::string& suffix) {
    for (const auto& p : HostTest::files()) if (p.first.size() >= suffix.size() && p.first.compare(p.first.size()-suffix.size(), suffix.size(), suffix) == 0) return p.second->bytes;
    return {};
}
static void save(const std::string& prefix) {
    for (const char* suffix : {".csv", ".ph.csv", ".tx.csv"}) {
        const std::string name = std::string("/m_00001.csv") + (std::string(suffix)==".csv" ? "" : suffix);
        auto it=HostTest::files().find(name); check(it!=HostTest::files().end(), "expected output file exists");
        if (it!=HostTest::files().end()) { std::ofstream f(prefix+suffix, std::ios::binary);f<<it->second->bytes; }
    }
}
int main(int argc, char** argv) {
    if (argc != 3) return 2;
    const std::string profile=argv[1], prefix=argv[2];
    HostTest::reset(); DataLogger logger;
    check(logger.begin(3.01f,2.572f,2.184f,true,"HOST_TEST"),"logger startup");
    const unsigned base=HostTest::clockMs; const unsigned total=2400;
#ifdef USV_BUFFERED_SD
    HostTest::writeDelayMs = profile=="stall" ? 10 : 2;
    HostTest::switchDelayMs = profile=="stall" ? 10 : 5;
    HostTest::syncDelayMs = profile=="stall" ? 15 : 3;
    HostTest::bytesPerMs=50; // Explicit synthetic 400-kbit/s wire component; not a board measurement.
    if (profile=="stall") { HostTest::oneShotAt=base+12000; HostTest::oneShotDelay=1200; }
    unsigned callsBefore=HostTest::writeCalls;
    check(logger.logRecord(row(1,base)) && logger.logPhDiagnostic(row(1,base).boot_ms,1,row(1,base).ph_diagnostic),"first row pair accepted");
    check(HostTest::writeCalls==callsBefore,"main/pH append performs no physical file write");
    unsigned produced=1;
#else
    unsigned produced=0;
#endif
    unsigned nextTx=1, dropped=0, maxLoop=0, maxQueue=0; std::deque<unsigned> q;
    const unsigned guard=base+300000;
    while (HostTest::clockMs<guard) {
        unsigned started=HostTest::clockMs;
        while (produced<total && base+produced*50<=HostTest::clockMs) {
            ++produced;
            if (q.size()<64) q.push_back(produced); else ++dropped;
        }
        maxQueue=std::max(maxQueue,(unsigned)q.size());
        while(nextTx<=total/20 && base+(nextTx-1)*1000<=HostTest::clockMs) check(logger.logTxDiagnostic(tx(nextTx++,base)),"TX event accepted once");
        unsigned n=0;
        while(!q.empty()) {
#ifdef USV_BUFFERED_SD
            if(n>=SD_LOG_ROWS_PER_IO_LOOP || !logger.canAcceptLogPair()) break;
#endif
            SdLogRecord r=row(q.front(),base);q.pop_front();++n;
            check(logger.logRecord(r),"main row accepted");
            check(logger.logPhDiagnostic(r.boot_ms,r.sequence_number,r.ph_diagnostic),"matching pH row accepted");
        }
#ifdef USV_BUFFERED_SD
        check(logger.service(),"bounded service phase succeeds");
        if(produced==total && q.empty() && logger.mainSyncedRows()==total && logger.phSyncedRows()==total) break;
#else
        if(produced==total && q.empty()) break;
#endif
        maxLoop=std::max(maxLoop,(unsigned)(HostTest::clockMs-started));
        HostTest::clockMs+=20;
    }
    check(produced==total && q.empty() && dropped==0,"all 2400 scheduled 20-Hz rows collected without queue loss");
    check(nextTx==121,"all 120 scheduled one-Hz TX events retained");
    check(logger.isReady(),"logger remains ready");
#ifdef USV_BUFFERED_SD
    check(logger.mainAcceptedRows()==total && logger.phAcceptedRows()==total,"both row acceptance counts complete");
    check(logger.mainWrittenRows()==total && logger.phWrittenRows()==total,"both file write counts complete");
    check(logger.mainSyncedRows()==total && logger.phSyncedRows()==total,"both files synced through final row");
    check(maxLoop<3000,"synthetic IO iteration returns before unchanged watchdog interval");
    if(profile=="fast") check(logger.syncDeadlineMisses()==0 && logger.maxSyncGapMs()<=1000,"normal synthetic backend completes sync cycles within one second");
    else check(logger.syncDeadlineMisses()>0,"injected 1.2-second SD stall is reported as a missed one-second deadline");
    std::printf("MAX_SERVICE_MS=%u MAX_SYNC_GAP_MS=%u SYNC_DEADLINE_MISSES=%u\n",logger.maxServiceMs(),logger.maxSyncGapMs(),logger.syncDeadlineMisses());
    for (const auto& f : HostTest::files()) {
        if (f.first == "/m_00001.csv" || f.first == "/m_00001.csv.ph.csv" || f.first == "/m_00001.csv.tx.csv") {
            if (profile=="fast") check(f.second->maxDurableLagMs<=SD_LOG_DURABILITY_MS,"every data row durably synced within one second in normal synthetic backend");
            std::printf("FILE=%s MAX_ROW_DURABILITY_MS=%lu\n",f.first.c_str(),f.second->maxDurableLagMs);
        }
    }
#endif
    std::printf("SCHEDULED_ROWS=%u DROPPED=%u MAX_QUEUE=%u MAX_IO_LOOP_MS=%u FILE_WRITE_CALLS=%u SYNC_CALLS=%u\n",produced,dropped,maxQueue,maxLoop,HostTest::writeCalls,HostTest::syncCalls);
    save(prefix);
#ifdef USV_BUFFERED_SD
    HostTest::reset(); DataLogger bad;
    check(bad.begin(3.01f,2.572f,2.184f,true,"HOST_TEST"),"fault logger startup");
    check(bad.logRecord(row(1,HostTest::clockMs)),"fault row buffered");
    HostTest::shortMainWrite=true; HostTest::clockMs+=300;
    check(!bad.service() && !bad.isReady() && bad.writeFailCount()==1,"short physical main write fails visibly and stops the logger");
    check(bad.mainPendingBytes()>0 && bad.mainWrittenRows()==0,"short write does not falsely acknowledge complete buffered rows");
    HostTest::reset(); DataLogger syncBad;
    check(syncBad.begin(3.01f,2.572f,2.184f,true,"HOST_TEST"),"sync fault logger startup");
    HostTest::failSync=true; HostTest::clockMs+=SD_LOG_SYNC_INTERVAL_MS;
    for(unsigned i=0;i<8 && syncBad.isReady();++i) { syncBad.service(); HostTest::clockMs+=20; }
    check(!syncBad.isReady() && syncBad.writeFailCount()==1,"sync error is checked and stops logging visibly");
    for (bool shortWrite : {true, false}) {
        HostTest::reset(); DataLogger phBad;
        check(phBad.begin(3.01f,2.572f,2.184f,true,"HOST_TEST"),"pH-only fault startup");
        auto first=row(1,HostTest::clockMs);
        check(phBad.logRecord(first) && phBad.logPhDiagnostic(first.boot_ms,1,first.ph_diagnostic),"pH fault pair accepted");
        HostTest::shortPhWrite=shortWrite; HostTest::failPhSync=!shortWrite;
        for(unsigned i=0;i<80 && !phBad.phDiagnosticStopped();++i) { phBad.service(); HostTest::clockMs+=20; }
        check(phBad.isReady() && phBad.writeFailCount()==0 && phBad.phDiagnosticWriteFailCount()==1,"pH-only write/sync error is isolated from main");
        check(phBad.phDiagnosticStopped() && phBad.phSyncedRows()==0,"failed pH rows never falsely reported synced");
        const size_t retained=phBad.phPendingBytes();
        for(unsigned i=2;i<=80;++i) {
            check(phBad.canAcceptLogPair(),"failed pH buffer cannot block main dequeue");
            auto next=row(i,HostTest::clockMs);
            check(phBad.logRecord(next),"main continues after pH fault");
            check(!phBad.logPhDiagnostic(next.boot_ms,i,next.ph_diagnostic),"future pH loss explicitly rejected/counted");
            phBad.service(); HostTest::clockMs+=50;
        }
        for(unsigned i=0;i<80 && phBad.mainSyncedRows()!=80;++i) { phBad.service(); HostTest::clockMs+=20; }
        check(phBad.mainSyncedRows()==80 && phBad.phSyncedRows()==0 && phBad.phDiagnosticWriteFailCount()==80,"main remains durable and pH losses counted");
        check(phBad.phPendingBytes()==retained,"failed pH bytes retained without retry/duplicate append");
    }
    HostTest::reset(); DataLogger txBad;
    check(txBad.begin(3.01f,2.572f,2.184f,true,"HOST_TEST"),"TX-only sync fault startup");
    auto first=row(1,HostTest::clockMs);
    check(txBad.logRecord(first) && txBad.logPhDiagnostic(first.boot_ms,1,first.ph_diagnostic) && txBad.logTxDiagnostic(tx(1,HostTest::clockMs)),"TX sync fault records accepted");
    HostTest::failTxSync=true;
    for(unsigned i=0;i<50;++i) { txBad.service(); HostTest::clockMs+=20; }
    check(txBad.isReady() && txBad.writeFailCount()==0 && txBad.txDiagnosticWriteFailCount()>0,"TX-only sync error counted without stopping main");
    check(txBad.mainSyncedRows()==1 && txBad.phSyncedRows()==1,"main and pH synced despite TX-only failure");
    HostTest::reset(); DataLogger timeBad;
    check(timeBad.begin(3.01f,2.572f,2.184f,true,"HOST_TEST"),"timestamp cache fault startup");
    check(timeBad.logRecord(row(1,HostTest::clockMs)),"timestamp fault row buffered");
    HostTest::failTimestampCache=true;
    for(unsigned i=0;i<50 && timeBad.isReady();++i) { timeBad.service(); HostTest::clockMs+=20; }
    check(!timeBad.isReady() && timeBad.writeFailCount()==1 && timeBad.mainSyncedRows()==0,"timestamp cache error stops main without false sync acknowledgment");
#endif
    std::printf("RATE_TEST_FAILURES=%u\n",failures);return failures?1:0;
}
