#ifndef USV_HOST_SPI_H
#define USV_HOST_SPI_H

#define HSPI 1
#define MSBFIRST 1
#define SPI_MODE0 0

class SPISettings {
public:
    SPISettings(unsigned long, int, int) {}
};
class SPIClass {
public:
    explicit SPIClass(int) {}
    void end() {}
    void begin(int, int, int, int) {}
    void beginTransaction(const SPISettings&) {}
    void endTransaction() {}
    unsigned char transfer(unsigned char value) { return value; }
};

#endif
