#include "storage/DataLogger.h"
#include "HostTestSupport.h"
#include <SdFat.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <algorithm>
#include <deque>
#include <set>

static unsigned failed=0;
static void verify(bool ok,const char* name) {
    std::printf("CASE %s %s\n",ok?"PASS":"FAIL",name);if(!ok)++failed;
}
static void advance(unsigned ms) { HostTest::clockMs=(uint32_t)(HostTest::clockMs+ms); }
static SdLogRecord row(unsigned seq) {
    SdLogRecord r{};r.boot_ms=(uint32_t)HostTest::clockMs;r.sequence_number=seq;
    r.gps_lat=20.047398;r.gps_lon=99.891441;r.current_mode=1;r.gps_sats=12;
    r.gps_hdop=.52;r.heading_angle=191.4;r.left_pwm=112;r.right_pwm=108;
    r.utc_valid=1;r.utc_year=2026;r.utc_month=10;r.utc_day=25;r.utc_hour=9;
    r.utc_minute=(seq/1200)%60;r.utc_second=(seq/20)%60;
    r.ph_diagnostic.samplePresent=true;r.ph_diagnostic.sampleId=seq/5;
    r.ph_diagnostic.calibrationSource="UNKNOWN";r.ph_diagnostic.utcSource="GPS";
    return r;
}
static DiagnosticFormatting::TxLogRecord tx(unsigned seq) {
    DiagnosticFormatting::TxLogRecord r{};r.sequence=seq;r.sendStartMs=(uint32_t)HostTest::clockMs;
    r.sendEndMs=r.sendStartMs+1;r.mode=1;return r;
}
static void pump(DataLogger& l,unsigned n=100) {for(unsigned i=0;i<n;++i){advance(20);l.service();}}
static void reset() {HostTest::reset();}
static bool begin(DataLogger& l) {return l.begin(3.01f,2.572f,2.184f,true,"HOST_TEST");}
static std::string mainData() { std::string out;for(auto& p:HostTest::files())if(p.first.size()>=4 && p.first.substr(p.first.size()-4)==".csv" && p.first.find(".ph.csv")==std::string::npos && p.first.find(".tx.csv")==std::string::npos)out+=p.second->bytes;return out; }
static bool completeLines(const std::string& s,unsigned columns) {
    if(s.empty() || s.back()!='\n')return false;
    unsigned commas=0;for(char c:s){if(c==',')++commas;if(c=='\n'){if(commas+1!=columns)return false;commas=0;}}return true;
}
static void baselineCases() {
    {reset();DataLogger l;bool ok=begin(l);for(unsigned n=1;n<=24000;++n){advance(50);auto r=row(n);ok=l.logRecord(r)&&ok;ok=l.logPhDiagnostic(r.boot_ms,n,r.ph_diagnostic)&&ok;for(unsigned j=0;j<3;++j)l.service();if(n%20==0)ok=l.logTxDiagnostic(tx(n/20))&&ok;}pump(l,100);
     verify(ok && l.isReady() && l.mainSyncedRows()==24000 && l.phSyncedRows()==24000,"healthy_20min_main_ph_tx");verify(completeLines(mainData(),55),"healthy_main_schema55_complete_lines");}
    for(const char* suffix:{".csv",".ph.csv",".tx.csv"}) {
        reset();HostTest::shortHeaderSuffix=suffix;DataLogger l;bool ok=begin(l);
        if(std::string(suffix)==".csv")verify(!ok,"reject_partial_main_header");
        else if(std::string(suffix)==".ph.csv")verify(l.phDiagnosticWriteFailCount()>0,"detect_partial_ph_header");
        else verify(l.txDiagnosticWriteFailCount()>0,"detect_partial_tx_header");
    }
    {reset();DataLogger l;begin(l);verify(HostTest::unmatchedSpiEnd==0 && HostTest::spiDepth==0,"balanced_explicit_spi_transactions");}
    {reset();DataLogger l;begin(l);HostTest::failNextMutexTake=true;bool result=l.checkIntegrity();verify(!result && l.isReady(),"integrity_mutex_busy_does_not_latch_fault");}
    {reset();DataLogger l;begin(l);auto r=row(1);l.logRecord(r);l.logPhDiagnostic(r.boot_ms,1,r.ph_diagnostic);HostTest::shortPhWrite=true;for(unsigned n=0;n<100&&!l.phDiagnosticStopped();++n)pump(l,1);HostTest::shortPhWrite=false;for(unsigned n=2;n<=40;++n){l.logRecord(row(n));pump(l,3);}pump(l,100);verify(l.isReady()&&l.mainSyncedRows()==40,"ph_fault_main_initially_continues");verify(l.checkIntegrity()&&l.isReady(),"old_ph_card_error_does_not_disable_main");}
    {reset();DataLogger l;begin(l);HostTest::failTxSync=true;pump(l,100);HostTest::failTxSync=false;l.logRecord(row(1));pump(l,600);verify(l.checkIntegrity()&&l.isReady(),"old_tx_sync_error_does_not_disable_main");}
    {reset();DataLogger l;begin(l);HostTest::shortTxWrite=true;l.logTxDiagnostic(tx(1));HostTest::shortTxWrite=false;auto before=HostTest::txBytes();bool ok=l.logTxDiagnostic(tx(2));auto after=HostTest::txBytes();verify((!ok && before==after)||completeLines(after,20),"partial_tx_row_not_joined_to_next_packet");}
    {reset();DataLogger l;begin(l);for(unsigned n=1;n<1000&&l.canAcceptLogPair();++n)l.logRecord(row(n));verify(!l.canAcceptLogPair()&&l.isReady(),"normal_buffer_backpressure_preserves_ready");pump(l,200);verify(l.isReady()&&l.canAcceptLogPair(),"buffer_backpressure_drains_without_fault");}
    for(unsigned kind=0;kind<4;++kind) {
        reset();DataLogger l;begin(l);l.logRecord(row(1));
        if(kind==0)HostTest::shortMainWrite=true;
        if(kind==1)HostTest::failSync=true;
        if(kind==2)HostTest::failTimestampCache=true;
        if(kind==3)HostTest::cardAvailable=false;
        pump(l,60);
        HostTest::shortMainWrite=HostTest::failSync=HostTest::failTimestampCache=false;HostTest::cardAvailable=true;
        pump(l,500);
        const char* names[]={"main_short_write_recovers","main_sync_fault_recovers","timestamp_fault_recovers","card_removed_then_restored_recovers"};
        verify(l.isReady(),names[kind]);verify(l.mainSyncedRows()==1,(std::string(names[kind])+"_retains_unconfirmed_row").c_str());
    }
    {reset();HostTest::clockMs=0xFFFFF000UL;DataLogger l;begin(l);bool ok=true;for(unsigned n=1;n<=160;++n){advance(50);auto r=row(n);ok=l.logRecord(r)&&ok;pump(l,2);}pump(l,100);verify(ok&&l.isReady()&&l.mainSyncedRows()==160,"healthy_millis_wrap");}
    {reset();DataLogger l;begin(l);HostTest::maxFileBytes=1100;for(auto& p:HostTest::files())HostTest::maxFileBytes+=p.second->bytes.size();for(unsigned n=1;n<=20&&l.isReady();++n){l.logRecord(row(n));pump(l,30);}verify(!l.isReady(),"persistent_disk_full_is_visible");}
}
static std::string currentMain(DataLogger& l) {
    auto it=HostTest::files().find(l.filename());return it==HostTest::files().end()?"":it->second->bytes;
}
static unsigned countIdentity(const std::string& data,unsigned column,unsigned identity) {
    unsigned result=0;size_t start=data.find('\n');if(start==std::string::npos)return 0;++start;
    while(start<data.size()) {
        size_t end=data.find('\n',start);if(end==std::string::npos)break;
        size_t pos=start;for(unsigned c=0;c<column && pos<end;++c){pos=data.find(',',pos);if(pos==std::string::npos||pos>=end){pos=end;break;}++pos;}
        if(pos<end && std::strtoul(data.c_str()+pos,nullptr,10)==identity)++result;
        start=end+1;
    }return result;
}
static void deepCases() {
    {reset();DataLogger l;begin(l);for(unsigned i=0;i<1000;++i)pump(l,1);verify(l.isReady()&&l.writeFailCount()==0,"empty_service_never_false_faults");}
    {reset();DataLogger l;begin(l);auto r=row(1);r.utc_valid=0;l.logRecord(r);pump(l,100);verify(l.isReady()&&completeLines(currentMain(l),55),"no_utc_keeps_main_schema");}
    for(const char* suffix:{".ph.csv",".tx.csv"}) {
        reset();HostTest::unavailableSuffix=suffix;DataLogger l;begin(l);bool accepted=true;
        for(unsigned n=1;n<=400;++n){advance(50);auto r=row(n);if(l.canAcceptLogPair()){accepted=l.logRecord(r)&&accepted;l.logPhDiagnostic(r.boot_ms,n,r.ph_diagnostic);}else accepted=false;for(unsigned k=0;k<3;++k)l.service();if(n%20==0)l.logTxDiagnostic(tx(n/20));}
        pump(l,200);verify(accepted&&l.isReady()&&l.mainSyncedRows()==400,(std::string("permanent_optional_missing_")+suffix+"_main_continues").c_str());
    }
    for(unsigned faultAt:{1U,7U,17U,31U}) {
        reset();DataLogger l;begin(l);unsigned accepted=0,confirmed=0;std::string oldFile=l.filename();
        for(unsigned n=1;n<=faultAt;++n){advance(50);if(l.logRecord(row(n)))++accepted;if(n<faultAt)l.service();}
        confirmed=l.mainSyncedRows();HostTest::failSync=true;pump(l,60);HostTest::failSync=false;
        for(unsigned n=faultAt+1;n<=faultAt+8;++n){if(l.logRecord(row(n)))++accepted;}
        pump(l,1000);bool identities=true;auto data=currentMain(l);
        for(unsigned n=confirmed+1;n<=accepted;++n)identities=identities&&countIdentity(data,3,n)==1;
        verify(l.isReady()&&l.mainSyncedRows()==accepted&&identities&&completeLines(data,55),
               (std::string("sync_recovery_replays_exact_unconfirmed_prefix_")+std::to_string(faultAt)).c_str());
        verify(std::string(l.filename())!=oldFile,(std::string("recovery_rotates_main_file_")+std::to_string(faultAt)).c_str());
    }
    {reset();DataLogger l;begin(l);l.logRecord(row(1));HostTest::shortMainWrite=true;pump(l,20);HostTest::shortMainWrite=false;
     for(unsigned i=0;i<800 && !l.isReady();++i)pump(l,1);
     l.logTxDiagnostic(tx(1));l.logTxDiagnostic(tx(2));pump(l,200);
     std::string data;for(auto& p:HostTest::files())if(p.first==std::string(l.filename())+".tx.csv")data=p.second->bytes;
     verify(l.isReady()&&completeLines(data,20)&&countIdentity(data,0,1)==1&&countIdentity(data,0,2)==1,"tx_packet_order_after_recovery");}
    {reset();DataLogger l;begin(l);l.logTxDiagnostic(tx(1));HostTest::shortTxWrite=true;l.logTxDiagnostic(tx(2));HostTest::shortTxWrite=false;
     l.logTxDiagnostic(tx(3));pump(l,1000);l.logTxDiagnostic(tx(4));pump(l,200);
     std::string data;for(auto& p:HostTest::files())if(p.first==std::string(l.filename())+".tx.csv")data=p.second->bytes;
     verify(l.isReady()&&completeLines(data,20)&&countIdentity(data,0,1)==1&&countIdentity(data,0,2)==1&&countIdentity(data,0,3)==1&&countIdentity(data,0,4)==1,"tx_partial_and_future_packets_replay_in_new_file");}
    {reset();DataLogger l;begin(l);auto r=row(1);l.logRecord(r);l.logPhDiagnostic(r.boot_ms,1,r.ph_diagnostic);HostTest::failPhSync=true;pump(l,50);HostTest::failPhSync=false;pump(l,1000);
     std::string data;for(auto& p:HostTest::files())if(p.first==std::string(l.filename())+".ph.csv")data=p.second->bytes;
     verify(l.isReady()&&l.phSyncedRows()==1&&completeLines(data,20)&&countIdentity(data,1,1)==1,"ph_sync_failure_retains_record_identity");}
    {reset();DataLogger first;begin(first);first.logRecord(row(1));pump(first,100);const std::string old=first.filename();const auto bytes=currentMain(first);
     HostTest::bootIndex=0;DataLogger second;begin(second);second.logRecord(row(1));pump(second,1000);
     auto it=HostTest::files().find(old);verify(second.isReady()&&std::string(second.filename())!=old&&it!=HostTest::files().end()&&it->second->bytes==bytes,"existing_run_file_not_overwritten_or_appended_after_counter_reuse");}
    {reset();HostTest::unavailableSuffix=".csv";DataLogger l;bool started=begin(l);HostTest::unavailableSuffix.clear();l.logRecord(row(1));pump(l,1000);verify(!started&&l.isReady()&&l.mainSyncedRows()==1,"startup_open_fault_recovers_without_reboot");}
    {reset();HostTest::cardAvailable=false;DataLogger l;bool started=begin(l);l.logRecord(row(1));HostTest::cardAvailable=true;pump(l,1000);verify(!started&&l.isReady()&&l.mainSyncedRows()==1,"startup_absent_card_later_recovers");}
    {reset();DataLogger l;begin(l);HostTest::cardAvailable=false;l.logRecord(row(1));pump(l,1000);unsigned attempts=HostTest::cardBeginCalls;
     verify(!l.isReady()&&attempts<=9,"persistent_card_absence_backoff_prevents_busy_loop");HostTest::cardAvailable=true;pump(l,1000);verify(l.isReady()&&l.mainSyncedRows()==1,"persistent_card_restoration_replays_pending");}
    {reset();HostTest::clockMs=0xFFFFF000UL;DataLogger l;begin(l);l.logRecord(row(1));HostTest::shortMainWrite=true;pump(l,20);HostTest::shortMainWrite=false;pump(l,1000);verify(l.isReady()&&l.mainSyncedRows()==1,"recovery_timer_wrap");}
    {reset();DataLogger l;begin(l);bool ok=true;for(unsigned cycle=1;cycle<=3;++cycle){l.logRecord(row(cycle));HostTest::shortMainWrite=true;pump(l,20);HostTest::shortMainWrite=false;pump(l,1000);ok=ok&&l.isReady()&&l.mainSyncedRows()==cycle;}
     verify(ok,"multiple_fault_recovery_cycles_preserve_counts");}
    {reset();DataLogger l;begin(l);HostTest::cardAvailable=false;l.logRecord(row(1));pump(l,30);unsigned accepted=1;for(unsigned n=2;n<=200;++n)if(l.canAcceptLogPair()&&l.logRecord(row(n)))++accepted;
     verify(!l.canAcceptLogPair()&&!l.isReady(),"outage_buffer_full_visible");HostTest::cardAvailable=true;pump(l,1000);verify(l.isReady()&&l.mainSyncedRows()==accepted,"full_buffer_drains_after_restore_without_false_ack");}
    {reset();HostTest::shortHeaderSuffix=".meta.txt";DataLogger l;begin(l);l.logRecord(row(1));pump(l,150);
     verify(l.isReady()&&l.mainSyncedRows()==1&&HostTest::serialText.find("metadata")!=std::string::npos&&HostTest::serialText.find("failed")!=std::string::npos,"partial_metadata_is_reported_without_stopping_main");}
    {reset();DataLogger l;begin(l);auto orphan=std::make_shared<HostTest::FileEntry>();orphan->path="/m_00001_r001.csv.ph.csv";orphan->bytes="do not overwrite\n";HostTest::files()[orphan->path]=orphan;
     l.logRecord(row(1));HostTest::shortMainWrite=true;pump(l,20);HostTest::shortMainWrite=false;pump(l,1000);
     verify(l.isReady()&&std::string(l.filename())!="/m_00001_r001.csv"&&orphan->bytes=="do not overwrite\n","orphan_sidecar_name_collision_preserved");}
    {reset();auto orphan=std::make_shared<HostTest::FileEntry>();orphan->path="/m_00001.csv.ph.csv";orphan->bytes="do not overwrite\n";HostTest::files()[orphan->path]=orphan;DataLogger l;begin(l);l.logRecord(row(1));pump(l,1000);
     verify(l.isReady()&&std::string(l.filename())!="/m_00001.csv"&&orphan->bytes=="do not overwrite\n","startup_orphan_sidecar_name_collision_preserved");}
    {reset();DataLogger l;begin(l);bool ok=true;
     for(unsigned cycle=1;cycle<=1001 && ok;++cycle){l.logRecord(row(cycle));HostTest::shortMainWrite=true;pump(l,20);HostTest::shortMainWrite=false;pump(l,1000);ok=l.isReady()&&l.mainSyncedRows()==cycle;}
     verify(ok,"more_than_999_recovery_sessions_no_terminal_name_limit");}
}
static void schedulingCases() {
    for(unsigned outage:{0U,600U,3000U,15000U}) {
        reset();DataLogger l;begin(l);const uint32_t start=(uint32_t)HostTest::clockMs;
        const unsigned total=2400;unsigned produced=0,dropped=0,maxQueue=0;std::deque<SdLogRecord> q;
        for(unsigned guard=0;guard<15000;++guard) {
            const uint32_t elapsed=(uint32_t)((uint32_t)HostTest::clockMs-start);
            while(produced<total && produced*50<=elapsed) {
                auto rec=row(++produced);rec.boot_ms=start+(produced-1)*50;
                if(q.size()<64)q.push_back(rec);else ++dropped;
            }
            maxQueue=std::max(maxQueue,(unsigned)q.size());
            HostTest::cardAvailable=!(outage && elapsed>=5000 && elapsed<5000+outage);
            // Independent FIFO contract: remove only after actual logger acceptance.
            for(unsigned n=0;n<8 && !q.empty() && l.canAcceptLogPair();++n) {
                const auto& rec=q.front();if(!l.logRecord(rec))break;
                l.logPhDiagnostic(rec.boot_ms,rec.sequence_number,rec.ph_diagnostic);q.pop_front();
            }
            l.service();advance(20);
            if(produced==total&&q.empty()&&l.isReady()&&l.mainSyncedRows()==total-dropped)break;
        }
        const bool rowsOk=produced==total&&q.empty()&&l.isReady()&&l.mainSyncedRows()==total-dropped;
        verify(rowsOk&&(outage<10000?dropped==0:dropped>0),
            (std::string("synthetic_fifo_outage_")+std::to_string(outage)+"ms").c_str());
        std::printf("SCHEDULE outage_ms=%u produced=%u dropped=%u accepted=%u synced=%u max_queue=%u\n",
                    outage,produced,dropped,l.mainAcceptedRows(),l.mainSyncedRows(),maxQueue);
    }
}

static std::string metadataParent(const std::string& file) {
    auto it=HostTest::files().find(file+".meta.txt");if(it==HostTest::files().end())return "";
    auto pos=it->second->bytes.find("parent_run=");if(pos==std::string::npos)return "";
    pos+=11;return it->second->bytes.substr(pos,it->second->bytes.find('\n',pos)-pos);
}
static void reviewCases() {
    for(const char* suffix:{"", ".ph.csv", ".tx.csv", ".meta.txt"}) {
        reset();auto old=std::make_shared<HostTest::FileEntry>();old->path=std::string("/m_00001.csv")+suffix;old->bytes="old session must survive\n";HostTest::files()[old->path]=old;
        HostTest::falseNegativeExists=true;DataLogger l;begin(l);l.logRecord(row(1));pump(l,1000);
        verify(old->bytes=="old session must survive\n",(std::string("false_negative_exists_preserves_old_")+suffix).c_str());
    }
    for(unsigned stream=0;stream<2;++stream)for(unsigned fault=0;fault<3;++fault) {
        reset();DataLogger l;begin(l);auto r=row(1);l.logRecord(r);l.logPhDiagnostic(r.boot_ms,1,r.ph_diagnostic);l.logTxDiagnostic(tx(1));
        HostTest::shortMainWrite=true;pump(l,20);HostTest::shortMainWrite=false;
        const std::string suffix=stream?".tx.csv":".ph.csv";
        if(fault==0)HostTest::unavailableSuffix=suffix;
        if(fault==1)HostTest::shortHeaderSuffix=suffix;
        if(fault==2){HostTest::failPhSync=!stream;HostTest::failTxSync=stream;}
        // Reach FINISH while the optional stream is still unavailable.
        for(unsigned n=0;n<800&&!l.isReady();++n)pump(l,1);
        const bool mainResumed=l.isReady();
        HostTest::unavailableSuffix.clear();HostTest::shortHeaderSuffix.clear();HostTest::failPhSync=HostTest::failTxSync=false;
        pump(l,1200);bool found=false;
        for(auto& f:HostTest::files())if(f.first.find(std::string(l.filename()))==0&&f.first.size()>=suffix.size()&&f.first.substr(f.first.size()-suffix.size())==suffix&&completeLines(f.second->bytes,20)&&countIdentity(f.second->bytes,stream?0:1,1)==1)found=true;
        verify(mainResumed&&l.isReady()&&found,(std::string("optional_restore_")+suffix+"_fault"+std::to_string(fault)).c_str());
    }
    for(unsigned stream=0;stream<2;++stream) {
        reset();HostTest::unavailableSuffix=stream?".tx.csv":".ph.csv";DataLogger l;begin(l);
        auto r=row(1);l.logRecord(r);l.logPhDiagnostic(r.boot_ms,1,r.ph_diagnostic);l.logTxDiagnostic(tx(1));pump(l,150);
        HostTest::unavailableSuffix.clear();pump(l,1200);bool found=false;
        const std::string suffix=stream?".tx.csv":".ph.csv";
        for(auto& f:HostTest::files())if(f.first.find(std::string(l.filename()))==0&&f.first.size()>=suffix.size()&&f.first.substr(f.first.size()-suffix.size())==suffix&&completeLines(f.second->bytes,20)&&countIdentity(f.second->bytes,stream?0:1,1)==1)found=true;
        verify(l.isReady()&&found,(std::string("startup_optional_restoration_")+suffix).c_str());
    }
    {reset();DataLogger a;begin(a);a.logRecord(row(1));pump(a,100);const auto oldParent=metadataParent(a.filename());
     HostTest::bootIndex=0;DataLogger b;begin(b);b.logRecord(row(1));pump(b,1000);const auto newParent=metadataParent(b.filename());
     verify(!oldParent.empty()&&!newParent.empty()&&oldParent!=newParent,"counter_reuse_new_boot_has_unique_parent");
     b.logRecord(row(2));HostTest::shortMainWrite=true;pump(b,20);HostTest::shortMainWrite=false;pump(b,1000);
     verify(!newParent.empty()&&metadataParent(b.filename())==newParent,"counter_reuse_recovery_keeps_new_parent");}
}

int main(){baselineCases();deepCases();schedulingCases();reviewCases();std::printf("AUDIT_FAILURES=%u\n",failed);return failed?1:0;}
