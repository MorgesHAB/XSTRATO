// SD card memory
#include <FS.h>
#include <SD.h>
#include <SD_MMC.h> // with CLK, CMD; D0-3

#define SD_CMD                  6
#define SD_CLK                  7
#define SD_D0                   15
#define SD_D1                   16
#define SD_D2                   4
#define SD_D3                   5

void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void createDir(fs::FS &fs, const char *path);
void removeDir(fs::FS &fs, const char *path);
void readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
void renameFile(fs::FS &fs, const char *path1, const char *path2);
void deleteFile(fs::FS &fs, const char *path);
void testFileIO(fs::FS &fs, const char *path);
void getFile(fs::FS &fs, const char *path, uint8_t* buf, uint8_t size);
void logAppendFile(fs::FS &fs, const char *path, const uint8_t *message, size_t length);
void save_image(fs::FS &fs, const char *path, const uint8_t *message, size_t length);

void test_SD_card();