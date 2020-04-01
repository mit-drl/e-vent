#include "SdCardDataLogger.h"

SdCardDataLogger::SdCardDataLogger(SdFat &sd, uint8_t csPin, SPISettings spiSettings) : sd(sd), csPin(csPin), spiSettings(spiSettings) {

}

SdCardDataLogger::~SdCardDataLogger() {
}

void SdCardDataLogger::setup() {
    // here, we initialize the SD card
    if (!this->sd.begin()) {
        Serial.println("SD Initialization failed");
    }

    Serial.println("Initialization complete");

    // get the name of the file
    const char *name = getName();

    Serial.print("Creating file: ");
    Serial.println(name);

    // create the file
    if (!sd.exists(name)) {
        makeNewFile();
    } else {
        Serial.println("File already exists...");
    }
}


void SdCardDataLogger::write(const char *msg) {
    // check to see if the file exists
    const char *name = getName();

    Serial.println(msg);

    // // check to see if the file exists; if not, create it
    // if (!sd.exists(name)) {
    //     makeNewFile();

    //     if (curFile.isOpen()) {
    //         curFile.close();
    //     }
    // }

    // if (!curFile.isOpen()) {
    //     openDataFile();
    // }

    // if (curFile.isOpen()) {
    //     int nBytes = curFile.write(msg);

    //     if (nBytes > 0) {
    //         curFile.sync();
    //     }
    // }
}

const char *SdCardDataLogger::getName() {
    int autoinc = 0;

    snprintf(nameBuf, sizeof(nameBuf), "DATA_%04d.csv", autoinc);

    return nameBuf;
}

void SdCardDataLogger::makeNewFile() {
    const char *name = getName();

    this->nameBuf[0] = "D";
    this->nameBuf[1] = "A";
    this->nameBuf[2] = "T";
    this->nameBuf[3] = "A";
    this->nameBuf[4] = "_";
    this->nameBuf[5] = "0";

    FatFile tmpFile;

    if (!tmpFile.open(name, O_CREAT | O_RDWR | O_APPEND)) {
        Serial.println("Failed to write/open SD card");
    }
    else {
        tmpFile.write("millis,state,motor_position,volume,bpm,ie_ratio,tin,tex,vin,vex,pressure");
        tmpFile.write("\n");

        tmpFile.close();
    }
}

bool SdCardDataLogger::openDataFile() {
    const char *name = getName();

    if (curFile.open(name, O_RDWR | O_AT_END | O_APPEND)) {
        return true;
    }
    else {
        return false;
    }
}
