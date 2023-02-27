#include "LogSD.h"


void test_SD_card() {

    uint8_t cardType = SD_MMC.cardType();

    if (cardType == CARD_NONE) {
        USBSerial.println("No SD card attached");
        return;
    }

    USBSerial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
        USBSerial.println("MMC");
    } else if (cardType == CARD_SD) {
        USBSerial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        USBSerial.println("SDHC");
    } else {
        USBSerial.println("UNKNOWN");
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    USBSerial.printf("SD Card Size: %lluMB\n", cardSize);

    listDir(SD_MMC, "/", 0);
    // createDir(SD_MMC, "/mydir");
    listDir(SD_MMC, "/", 0);
    removeDir(SD_MMC, "/mydir");
    listDir(SD_MMC, "/", 2);
    writeFile(SD_MMC, "/hello.txt", "Hello ");
    appendFile(SD_MMC, "/hello.txt", "World!\n");
    readFile(SD_MMC, "/hello.txt");
    // deleteFile(SD_MMC, "/foo.txt");
    // renameFile(SD_MMC, "/hello.txt", "/foo.txt");
    // readFile(SD_MMC, "/foo.txt");
    // testFileIO(SD_MMC, "/test.txt");
    USBSerial.printf("\r\nTotal space: %lluMB", SD_MMC.totalBytes() / (1024 * 1024));
    USBSerial.printf("\r\nUsed space: %lluMB", SD_MMC.usedBytes() / (1024 * 1024));
    USBSerial.printf("\r\nTotal space: %llu", SD_MMC.totalBytes());
    USBSerial.printf("\r\nUsed space: %llu", SD_MMC.usedBytes());
}



void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
    USBSerial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root) {
        USBSerial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        USBSerial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            USBSerial.print("  DIR : ");
            USBSerial.println(file.name());
            if (levels) {
                listDir(fs, file.name(), levels - 1);
            }
        } else {
            USBSerial.print("  FILE: ");
            USBSerial.print(file.name());
            USBSerial.print("  SIZE: ");
            USBSerial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char *path) {
    USBSerial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path)) {
        USBSerial.println("Dir created");
    } else {
        USBSerial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char *path) {
    USBSerial.printf("Removing Dir: %s\n", path);
    if (fs.rmdir(path)) {
        USBSerial.println("Dir removed");
    } else {
        USBSerial.println("rmdir failed");
    }
}

void getFile(fs::FS &fs, const char *path, uint8_t* buf, uint8_t size) {
    USBSerial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file) {
        USBSerial.println("Failed to open file for reading");
        return;
    }

    USBSerial.println("Reading" + String(file.size()) + "B");
    uint8_t i = 0;
    while (file.available() && i < size) {
        buf[i++] = file.read();
    }
    file.close();
}

size_t load_img_buffer(const char *path, uint8_t* imgbuf) {
    File file = SD_MMC.open(path);
    if (!file) {
        USBSerial.println("Failed to open image for reading");
        return false;
    }

    USBSerial.println("Reading" + String(file.size()) + "B");
    size_t size = 0;
    while (file.available() && size < IMG_BUFFER_SIZE) {
        imgbuf[size++] = file.read();
    }
    file.close();
    return size;
}

void readFile(fs::FS &fs, const char *path) {
    USBSerial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file) {
        USBSerial.println("Failed to open file for reading");
        return;
    }

    USBSerial.println("Read from file " + String(file.size()) + "B : ");
    while (file.available()) {
        USBSerial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
    USBSerial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file) {
        USBSerial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message)) {
        USBSerial.println("File written");
    } else {
        USBSerial.println("Write failed");
    }
    file.close();
}

void logAppendFile(fs::FS &fs, const char *path, const uint8_t *message, size_t length) {
    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        USBSerial.println("Failed to open file for appending");
        return;
    }
    if (file.write(message, length)) {
        USBSerial.println("LOG success to: " + String(path));
        //USBSerial.println("Message appended");
    } else {
        USBSerial.println("LOG FAILED ! :-(");
    }
    file.close();
}

void save_image(fs::FS &fs, const char *path, const uint8_t *message, size_t length) {
    USBSerial.println("Saving image: " + String(path));

    File file = fs.open(path, FILE_WRITE);
    if (!file) {
        USBSerial.println("Failed to open file for saving image");
        return;
    }
    if (file.write(message, length)) {
        USBSerial.println("Image saved");
    } else {
        USBSerial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
    USBSerial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        USBSerial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message)) {
        USBSerial.println("Message appended");
    } else {
        USBSerial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
    USBSerial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        USBSerial.println("File renamed");
    } else {
        USBSerial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char *path) {
    USBSerial.printf("Deleting file: %s\n", path);
    if (fs.remove(path)) {
        USBSerial.println("File deleted");
    } else {
        USBSerial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char *path) {
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if (file) {
        len = file.size();
        size_t flen = len;
        start = millis();
        while (len) {
            size_t toRead = len;
            if (toRead > 512) {
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        USBSerial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        USBSerial.println("Failed to open file for reading");
    }

    file = fs.open(path, FILE_WRITE);
    if (!file) {
        USBSerial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for (i = 0; i < 2048; i++) {
        file.write(buf, 512);
    }
    end = millis() - start;
    USBSerial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}
