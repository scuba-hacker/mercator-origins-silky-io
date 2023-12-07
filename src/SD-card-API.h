#ifndef SD_CARD_API_H
#define SD_CARD_API_H

#include <stdint.h>

namespace fs
{
    class FS;
}
namespace mercatorFS
{
    void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
    void createDir(fs::FS &fs, const char * path);
    void removeDir(fs::FS &fs, const char * path);
    void readFile(fs::FS &fs, const char * path);
    void writeFile(fs::FS &fs, const char * path, const char * message);
    void appendFile(fs::FS &fs, const char * path, const char * message);
    void renameFile(fs::FS &fs, const char * path1, const char * path2);
    void deleteFile(fs::FS &fs, const char * path);
    void testFlashFileIO(fs::FS &fs, const char * path);
    void testFileIO(uint8_t cardType);
}

#endif
