/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "drivers/d_time.h"

#include "ISLogFileFatFs.h"
#include "ISConstants.h"

cISLogFileFatFs::cISLogFileFatFs() : m_file{}, m_opened(false), m_lastSync(0)
{
}

cISLogFileFatFs::cISLogFileFatFs(const std::string& filePath, const char* mode)  : cISLogFileFatFs(filePath.c_str(), mode)
{

}

cISLogFileFatFs::cISLogFileFatFs(const char* filePath, const char* mode)  : cISLogFileFatFs()
{
    open(filePath, mode);
}

cISLogFileFatFs::~cISLogFileFatFs()
{
    close();
}

bool cISLogFileFatFs::open(const char* filePath, const char* mode)
{
    CONST_EXPRESSION std::size_t MAX_MODE_LEN = 3;
    CONST_EXPRESSION char DRIVE[] = "0:";
    BYTE flags = 0x00;
    bool append = false;
    char fullFilePath[256];
    strncpy(fullFilePath, DRIVE, sizeof(fullFilePath));
    strncat(fullFilePath, filePath, _MIN(strlen(fullFilePath), sizeof(fullFilePath)));
	
    if (strncmp(mode, "r", MAX_MODE_LEN) == 0)
    {
        flags = FA_READ;
    }
    else if (strncmp(mode, "r+", MAX_MODE_LEN) == 0)
    {
        flags = FA_READ | FA_WRITE;
    }
    else if (strncmp(mode, "w", MAX_MODE_LEN) == 0 ||
		strncmp(mode, "wb", MAX_MODE_LEN) == 0)
    {
        flags = FA_CREATE_ALWAYS | FA_WRITE;
    }
    else if (strncmp(mode, "w+", MAX_MODE_LEN) == 0 ||
		(strncmp(mode, "wb+", MAX_MODE_LEN) == 0))
    {
        flags = FA_CREATE_ALWAYS | FA_WRITE | FA_READ;
    }
    else if (strncmp(mode, "a", MAX_MODE_LEN) == 0)
    {
        // FatFs 0.09 does not support append mode.
        // If we upgrade, we can use instead of setting n append flag and seeking.
        //flags = FA_OPEN_APPEND | FA_WRITE;

        flags = FA_WRITE;
        append = true;
    }
    else if (strncmp(mode, "a+", MAX_MODE_LEN) == 0)
    {
        // FatFs 0.09 does not support append mode.
        // If we upgrade, we can use instead of setting n append flag and seeking.
        //flags = FA_OPEN_APPEND | FA_WRITE | FA_READ;

        flags = FA_WRITE | FA_READ;
        append = true;
    }
    else if (strncmp(mode, "wx", MAX_MODE_LEN) == 0)
    {
        flags = FA_CREATE_NEW | FA_WRITE;
    }
    else if (strncmp(mode, "w+x", MAX_MODE_LEN) == 0)
    {
        flags = FA_CREATE_NEW | FA_WRITE | FA_READ;
    }
	else
	{
		while (1) {} // not supported, fix this
	}

    FRESULT result = f_open(&m_file, filePath, flags);

    if (result != FR_OK)
    {
        return false;
    }

    if (append)
    {
        /* Move to end of the file to append data */
        result = f_lseek(&m_file, f_size(&m_file));
        if (result != FR_OK)
        {
            close();
            return false;
        }
    }

    m_opened = true;
    m_lastSync = time_msec();
    return true;
}

bool cISLogFileFatFs::close()
{
    if (m_opened)
    {
        FRESULT result = f_close(&m_file);
        if (result != FR_OK)
        {
            return false;
        }
        m_opened = false;
    }
    return true;
}

bool cISLogFileFatFs::flush()
{
    bool result = (f_sync(&m_file) == FR_OK);
    if (result)
    {
        m_lastSync = time_msec();
    }
    return result;
}

bool cISLogFileFatFs::good()
{
    return (f_error(&m_file) == FR_OK);
}

bool cISLogFileFatFs::isOpened()
{
    return m_opened;
}

int cISLogFileFatFs::putch(char ch)
{
    if (f_putc(ch, &m_file) == 1)
    {
        return static_cast<int>(ch);
    }
    return EOF;
}

int cISLogFileFatFs::puts(const char* str)
{
    return f_puts(str, &m_file);
}

std::size_t cISLogFileFatFs::write(const void* bytes, std::size_t len)
{
    std::size_t bytes_written = 0;
    f_write(&m_file, bytes, len, &bytes_written);
   
   // Indicate write to disk    
    extern int g_indicateFsWriteMs;
    g_indicateFsWriteMs = 50;
    
    if (time_msec() - m_lastSync > SYNC_FREQ_MS)
    {
        flush();
    }
    
    
    return bytes_written;
}

int cISLogFileFatFs::lprintf(const char* format, ...)
{
    int result;
    va_list args;
    va_start(args, format);
    result = vprintf(format, args);
    va_end(args);
    return result;
}

int cISLogFileFatFs::vprintf(const char* format, va_list args)
{
    return f_vprintf(&m_file, format, args);
}

int cISLogFileFatFs::getch()
{
    char buffer[1];
    CONST_EXPRESSION UINT bytesToRead = sizeof(buffer);
    UINT bytesRead;

    if (f_read(&m_file, buffer, bytesToRead, &bytesRead) == FR_OK && bytesRead == bytesToRead)
    {
        return static_cast<int>(buffer[0]);
    }
    return EOF;
}

std::size_t cISLogFileFatFs::read(void* bytes, std::size_t len)
{
    std::size_t bytes_read = 0;
    f_read(&m_file, bytes, len, &bytes_read);
    return bytes_read;
}