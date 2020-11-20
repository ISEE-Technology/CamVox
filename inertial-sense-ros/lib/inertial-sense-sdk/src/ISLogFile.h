//
// Created by robb on 11/5/18.
//

#ifndef _IS_SDK_IS_LOG_FILE_H_
#define _IS_SDK_IS_LOG_FILE_H_

#include "ISLogFileBase.h"



class cISLogFile : public cISLogFileBase
{
public:
    cISLogFile();
    cISLogFile(const std::string& filePath, const char* mode);
    cISLogFile(const char* filePath, const char* mode);
    ~cISLogFile();

    bool open(const char* filePath, const char* mode) OVERRIDE;
    bool close() OVERRIDE;
    bool flush() OVERRIDE;
    bool good() OVERRIDE;
    bool isOpened() OVERRIDE;

    int putch(char ch) OVERRIDE;
    int puts(const char* str) OVERRIDE;
    std::size_t write(const void* bytes, std::size_t len) OVERRIDE;
    int lprintf(const char* format, ...) OVERRIDE;
    int vprintf(const char* format, va_list args) OVERRIDE;

    int getch() OVERRIDE;
    std::size_t read(void* bytes, std::size_t len) OVERRIDE;

private:
    FILE *m_file;
};


#endif //_IS_SDK_IS_LOG_FILE_H_
