#ifndef USV_HOST_PREFERENCES_H
#define USV_HOST_PREFERENCES_H

#include "HostTestSupport.h"

class Preferences {
public:
    bool begin(const char*, bool) { return true; }
    unsigned int getUInt(const char*, unsigned int fallback) const {
        return HostTest::bootIndex ? HostTest::bootIndex : fallback;
    }
    size_t putUInt(const char*, unsigned int value) {
        HostTest::bootIndex = value;
        return sizeof(value);
    }
    double getDouble(const char*, double fallback) const { return fallback; }
    unsigned char getUChar(const char*, unsigned char fallback) const { return fallback; }
    size_t putDouble(const char*, double) { return sizeof(double); }
    size_t putUChar(const char*, unsigned char) { return sizeof(unsigned char); }
    void end() {}
};

#endif
