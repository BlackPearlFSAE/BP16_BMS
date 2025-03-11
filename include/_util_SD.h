
// Preprocessor to choose data log function in different architecture
#ifdef ARDUINO_ARCH_AVR
#endif


#ifdef ESP32
void getSDsize();
void SD_SPI_init(int sd_sck, int sd_miso, int sd_mosi, int sd_cs);

void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
#endif

