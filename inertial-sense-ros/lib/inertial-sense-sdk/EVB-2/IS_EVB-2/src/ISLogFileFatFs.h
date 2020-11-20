/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_FAT_FS_LOG_FILE_H_
#define IS_FAT_FS_LOG_FILE_H_

#include "ISLogFileBase.h"
#include "ff.h"
#include <cstdarg>

class cISLogFileFatFs : public cISLogFileBase
{
public:
    cISLogFileFatFs();
    cISLogFileFatFs(const std::string& filePath, const char* mode);
    cISLogFileFatFs(const char* filePath, const char* mode);
    ~cISLogFileFatFs();

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
    FIL m_file;
    bool m_opened;
    std::uint32_t m_lastSync;
    CONST_EXPRESSION std::uint32_t SYNC_FREQ_MS = 1000; // sync every second
};


#endif //IS_FAT_FS_LOG_FILE_H_
