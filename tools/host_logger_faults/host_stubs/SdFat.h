#ifndef USV_HOST_SDFAT_H
#define USV_HOST_SDFAT_H

#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include "HostTestSupport.h"

#ifndef O_RDWR
#define O_RDWR 0x02
#endif
#ifndef O_CREAT
#define O_CREAT 0x40
#endif
#ifndef O_AT_END
#define O_AT_END 0x80
#endif
#ifndef O_EXCL
#define O_EXCL 0x100
#endif
#define T_ACCESS 0x01
#define T_WRITE 0x02
#define T_CREATE 0x04
#define SHARED_SPI 1

class FsFile {
public:
    FsFile() = default;
    explicit FsFile(std::shared_ptr<HostTest::FileEntry> entry)
        : _entry(std::move(entry)), _open(true) {}

    explicit operator bool() const { return _open && (bool)_entry; }

    size_t write(const uint8_t* text, size_t length) {
        if (!*this || !text) return 0;
        HostTest::physicalOperation(_entry->path, length, false);
        const bool shortWrite = (HostTest::isTxPath(_entry->path) && HostTest::shortTxWrite) ||
            (HostTest::shortPhWrite && _entry->path.find(".ph.csv") != std::string::npos) ||
            (HostTest::shortMainWrite && !HostTest::isTxPath(_entry->path) &&
             _entry->path.find(".ph.csv") == std::string::npos);
        const bool headerFault = !HostTest::shortHeaderSuffix.empty() && _entry->bytes.empty() &&
            _entry->path.size() >= HostTest::shortHeaderSuffix.size() &&
            _entry->path.compare(_entry->path.size()-HostTest::shortHeaderSuffix.size(), HostTest::shortHeaderSuffix.size(),HostTest::shortHeaderSuffix)==0;
        size_t allBytes=0; if(HostTest::maxFileBytes)for(auto& p:HostTest::files())allBytes+=p.second->bytes.size();
        const bool full = HostTest::maxFileBytes && allBytes+length > HostTest::maxFileBytes;
        const bool fault = shortWrite || headerFault || full || !HostTest::cardAvailable;
        const size_t written = full || !HostTest::cardAvailable ? 0 : (fault && length ? length - 1 : length);
        if (fault) HostTest::cardError=0x13;
        _entry->bytes.append((const char*)text, written);
        return written;
    }
    size_t print(const char* text) {
        return text ? write((const uint8_t*)text, std::strlen(text)) : 0;
    }
    bool sync() {
        if (!*this) return false;
        HostTest::physicalOperation(_entry->path, 0, true);
        if (!HostTest::cardAvailable || HostTest::failSync || (HostTest::failPhSync && _entry->path.find(".ph.csv") != std::string::npos) ||
            (HostTest::failTxSync && HostTest::isTxPath(_entry->path))) { HostTest::cardError=0x13; return false; }
        size_t pos = _entry->syncedBytes;
        while (pos < _entry->bytes.size()) {
            const size_t end = _entry->bytes.find('\n', pos);
            if (end == std::string::npos) break;
            if (_entry->bytes[pos] >= '0' && _entry->bytes[pos] <= '9') {
                const size_t timeStart = HostTest::isTxPath(_entry->path) ? _entry->bytes.find(',', pos) + 1 : pos;
                const unsigned long boot = std::strtoul(_entry->bytes.c_str() + timeStart, nullptr, 10);
                const unsigned long lag = HostTest::clockMs - boot;
                if (lag > _entry->maxDurableLagMs) _entry->maxDurableLagMs = lag;
            }
            pos = end + 1;
        }
        _entry->syncedBytes = pos;
        _entry->lastSyncMs = HostTest::clockMs;
        ++_entry->syncCount;
        return true;
    }

    int printf(const char* format, ...) {
        char buffer[2048];
        va_list args;
        va_start(args, format);
        const int length = std::vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        if (length < 0 || (size_t)length >= sizeof(buffer)) return 0;
        return (int)print(buffer);
    }

    size_t println(const char* text) {
        const size_t count = print(text);
        return count + print("\n");
    }
    void flush() { (void)sync(); }
    void close() { _open = false; }
    bool timestamp(uint8_t, uint16_t, uint8_t, uint8_t,
                   uint8_t, uint8_t, uint8_t) {
        if (!sync()) return false;
        HostTest::physicalOperation(_entry->path, 0, true); // Directory cacheSync.
        if (HostTest::failTimestampCache) HostTest::cardError=0x13;
        return !HostTest::failTimestampCache;
    }

private:
    std::shared_ptr<HostTest::FileEntry> _entry;
    bool _open = false;
};

class HostSdCard {
public:
    uint8_t errorCode() const { return HostTest::cardError; }
    uint32_t errorData() const { return HostTest::cardError ? 0xFF : 0; }
    void sdError(uint8_t code) { HostTest::cardError=code; }
};

class SdSpiConfig {
public:
    SdSpiConfig(int, int, unsigned long, void*) {}
};

class SdFat {
public:
    bool exists(const char* path) const {
        return !HostTest::falseNegativeExists && HostTest::files().find(path) != HostTest::files().end();
    }
    FsFile open(const char* path, int flags) {
        const std::string name(path);
        if (!HostTest::unavailableSuffix.empty() && name.size()>=HostTest::unavailableSuffix.size() && name.compare(name.size()-HostTest::unavailableSuffix.size(),HostTest::unavailableSuffix.size(),HostTest::unavailableSuffix)==0) return FsFile{};
        if (HostTest::isTxPath(name) && HostTest::txUnavailable) return FsFile{};
        auto found = HostTest::files().find(name);
        if (found != HostTest::files().end()) return (flags & O_EXCL) ? FsFile{} : FsFile(found->second);
        if (!(flags & O_CREAT)) return FsFile{};
        auto entry = std::make_shared<HostTest::FileEntry>();
        entry->path = name;
        HostTest::files()[name] = entry;
        return FsFile(entry);
    }
    void end() {}
    bool cardBegin(const SdSpiConfig&) { ++HostTest::cardBeginCalls; if (HostTest::cardAvailable) HostTest::cardError=0; return HostTest::cardAvailable; }
    bool volumeBegin() { return HostTest::volumeAvailable; }
    HostSdCard* card() { static HostSdCard value; return &value; }
};

#endif
