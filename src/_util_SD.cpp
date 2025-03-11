#include <Arduino.h>

#ifdef ARDUINO_ARCH_AVR
#endif

#ifdef ESP32

// File system and ESP32 SPI SD lib
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <_util_SD.h>

// Preprocessor to choose different datalogging function 

void getSDsize(){
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

// SD initialization
void SD_SPI_init(int sd_sck, int sd_miso, int sd_mosi, int sd_cs) {

  // Initialize SD card
  SPI.begin(sd_sck, sd_miso, sd_mosi, sd_cs);
  if (!SD.begin(sd_cs)) {
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) 
    Serial.println("MMC");
  else if (cardType == CARD_SD) 
    Serial.println("SDSC");
  else if (cardType == CARD_SDHC) 
    Serial.println("SDHC");
  else 
    Serial.println("UNKNOWN");  
  
  getSDsize();

}

// SD card functions
void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    // Success - silent operation for performance
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

// delete file

// Read file
#endif