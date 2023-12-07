#include "SD-card-API.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"

void mercatorFS::listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.printf("Failed to open directory: %s\n",root);
        return;
    }
    if(!root.isDirectory()){
        Serial.printf("Not a directory: %s\n",root);
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void mercatorFS::createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.printf("Dir created: %s\n",path);
    } else {
        Serial.printf("mkdir failed: %s\n",path);
    }
}

void mercatorFS::removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.printf("Dir removed: %s\n",path);
    } else {
        Serial.printf("rmdir failed: %s\n",path);
    }
}

void mercatorFS::readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.printf("Failed to open file for reading: %s\n",path);
        return;
    }

    Serial.printf("Read from file: %s\n",path);
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void mercatorFS::writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.printf("Failed to open file for writing: %s\n",path);
        return;
    }
    if(file.print(message)){
        Serial.printf("File written: %s\n",path);
    } else {
        Serial.printf("Write failed: %s\n",path);
    }
    file.close();
}

void mercatorFS::appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.printf("Failed to open file for appending: %s\n",path);
        return;
    }
    if(file.print(message)){
        Serial.printf("Message appended: %s\n",path);
    } else {
        Serial.printf("Append failed: %s\n",path);
    }
    file.close();
}

void mercatorFS::renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.printf("File renamed: %s to %s\n",path1, path2);
    } else {
        Serial.printf("Rename file failed: %s to %s\n",path1, path2);
    }
}

void mercatorFS::deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.printf("File deleted: %s\n",path);
    } else {
        Serial.printf("Delete failed: %s\n",path);
    }
}

void mercatorFS::testFileIO(uint8_t cardType)
{
    Serial.print("SD Card Type: ");
    
    if(cardType == CARD_MMC)
    {
        Serial.println("MMC");
    } 
    else if(cardType == CARD_SD)
    {
        Serial.println("SDSC");
    } 
    else if(cardType == CARD_SDHC)
    {
        Serial.println("SDHC");
    } 
    else 
    {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    listDir(SD, "/", 0);
    createDir(SD, "/mydir");
    listDir(SD, "/", 0);
    removeDir(SD, "/mydir");
    listDir(SD, "/", 2);
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, "/hello.txt", "World!\n");
    readFile(SD, "/hello.txt");
    deleteFile(SD, "/foo.txt");
    renameFile(SD, "/hello.txt", "/foo.txt");
    readFile(SD, "/foo.txt");
    testFlashFileIO(SD, "/test.txt");
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

void mercatorFS::testFlashFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms file: %s\n",flen, end, path);
        file.close();
    } else {
        Serial.printf("Failed to open file for reading: %s\n",path);
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.printf("Failed to open file for writing: %s\n",path);
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms file: %s\n", 2048 * 512, end, path);
    file.close(); 
}

