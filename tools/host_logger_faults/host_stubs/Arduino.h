#ifndef USV_HOST_ARDUINO_H
#define USV_HOST_ARDUINO_H

#include <cstdint>
#include <cstdio>
#include "HostTestSupport.h"

#define F(value) value
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1

inline unsigned long millis() { return HostTest::clockMs; }
inline void delay(unsigned long ms) { HostTest::clockMs += ms; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}

class HostSerial {
public:
    template <typename... Args>
    void printf(const char* format, Args... args) const { char text[1024]; std::snprintf(text,sizeof(text),format,args...); HostTest::serialText += text; }
    void println(const char* text) const { HostTest::serialText += text; HostTest::serialText += "\n"; }
};
inline HostSerial Serial;

#endif
