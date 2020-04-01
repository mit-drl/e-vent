#ifndef __SDCARDDATALOGGER_H
#define __SDCARDDATALOGGER_H

#include "SdFat.h"

class SdCardDataLogger {
public:
    SdCardDataLogger(SdFat &sd, uint8_t csPin, SPISettings spiSettings);
    virtual ~SdCardDataLogger();

    void setup();

    void write(const char *msg);

private:
    const char *getName();
    bool openDataFile();
    void makeNewFile();

    SdFat &sd;
    uint8_t csPin;
    SPISettings spiSettings;

    char nameBuf[18];

    FatFile curFile;
};

#endif /* __SDCARDDATALOGGER_H */