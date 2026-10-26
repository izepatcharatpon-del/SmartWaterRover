#ifndef USV_HOST_TEST_SUPPORT_H
#define USV_HOST_TEST_SUPPORT_H

#include <memory>
#include <string>
#include <unordered_map>

namespace HostTest {
struct FileEntry {
    std::string path;
    std::string bytes;
    size_t syncedBytes = 0;
    unsigned syncCount = 0;
    unsigned long lastSyncMs = 0;
    unsigned long maxDurableLagMs = 0;
};

inline std::unordered_map<std::string, std::shared_ptr<FileEntry>>& files() {
    static std::unordered_map<std::string, std::shared_ptr<FileEntry>> value;
    return value;
}
inline std::string serialText;
inline unsigned char cardError = 0;
inline bool cardAvailable = true, volumeAvailable = true;
inline std::string unavailableSuffix, shortHeaderSuffix;
inline unsigned cardBeginCalls = 0, spiDepth = 0, unmatchedSpiEnd = 0;
inline unsigned long maxFileBytes = 0;
inline bool falseNegativeExists = false;
inline bool txUnavailable = false;
inline bool shortTxWrite = false;
inline bool failNextMutexTake = false;
inline unsigned long clockMs = 1000;
inline unsigned int bootIndex = 0;
inline unsigned writeCalls = 0, syncCalls = 0;
inline unsigned writeDelayMs = 0, syncDelayMs = 0, switchDelayMs = 0, bytesPerMs = 0;
inline bool shortMainWrite = false, failSync = false;
inline bool shortPhWrite = false, failPhSync = false, failTxSync = false, failTimestampCache = false;
inline unsigned long oneShotAt = 0, oneShotDelay = 0;
inline std::string lastPath;
inline void physicalOperation(const std::string& path, size_t bytes, bool sync) {
    unsigned long delayMs = sync ? syncDelayMs : writeDelayMs;
    if (!lastPath.empty() && lastPath != path) delayMs += switchDelayMs;
    if (!sync && bytesPerMs) delayMs += (bytes + bytesPerMs - 1) / bytesPerMs;
    if (oneShotDelay && clockMs >= oneShotAt) { delayMs += oneShotDelay; oneShotDelay = 0; }
    lastPath = path;
    clockMs += delayMs;
    if (sync) ++syncCalls; else ++writeCalls;
}

inline void reset() {
    files().clear();
    serialText.clear(); cardError = 0; cardAvailable = volumeAvailable = true;
    unavailableSuffix.clear(); shortHeaderSuffix.clear(); cardBeginCalls = spiDepth = unmatchedSpiEnd = 0; maxFileBytes = 0;
    falseNegativeExists = false;
    txUnavailable = false;
    shortTxWrite = false;
    failNextMutexTake = false;
    clockMs = 1000;
    bootIndex = 0;
    writeCalls = syncCalls = 0;
    writeDelayMs = syncDelayMs = switchDelayMs = bytesPerMs = 0;
    shortMainWrite = failSync = false;
    shortPhWrite = failPhSync = failTxSync = failTimestampCache = false;
    oneShotAt = oneShotDelay = 0;
    lastPath.clear();
}

inline bool isTxPath(const std::string& path) {
    return path.size() >= 7 && path.compare(path.size() - 7, 7, ".tx.csv") == 0;
}

inline std::string txBytes() {
    for (const auto& item : files()) {
        if (isTxPath(item.first)) return item.second->bytes;
    }
    return std::string();
}
} // namespace HostTest

#endif
