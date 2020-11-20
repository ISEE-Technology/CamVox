/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISLogFile.h"
#include "ISConstants.h"
#include <cstdarg>

cISLogFile::cISLogFile() : m_file(NULLPTR)
{
}

cISLogFile::cISLogFile(const char* filePath, const char* mode) : cISLogFile()
{
    open(filePath, mode);

}
cISLogFile::cISLogFile(const std::string& filePath, const char* mode) : cISLogFile(filePath.c_str(), mode)
{
}

cISLogFile::~cISLogFile()
{
    close();
}

bool cISLogFile::open(const char* filePath, const char* mode)
{
    m_file = fopen(filePath, mode);
    return m_file;
}

bool cISLogFile::close()
{
    if (m_file)
    {
        int result = fclose(m_file);
        if (result == 0)
        {
            m_file = NULLPTR;
            return true;
        }
    }
    return false;
}

bool cISLogFile::flush()
{
    if (m_file)
    {
        return (fflush(m_file) == 0);
    } else
    {
        return false;
    }
}

bool cISLogFile::good()
{
    if (m_file)
    {
        return (ferror(m_file) == 0);
    }
    else
    {
        return false;
    }
}

bool cISLogFile::isOpened()
{
    return m_file;
}

int cISLogFile::putch(char ch)
{
    if (m_file)
    {
        return fputc(ch, m_file);
    }
    else
    {
        return EOF;
    }
}

int cISLogFile::puts(const char* str)
{
    if (m_file)
    {
        return fputs(str, m_file);
    }
    else
    {
        return EOF;
    }
}

std::size_t cISLogFile::write(const void* bytes, std::size_t len)
{
    if (m_file)
    {
        return fwrite(bytes, 1, len, m_file);
    }
    else
    {
        return 0;
    }
}

int cISLogFile::lprintf(const char* format, ...)
{
    int result;
    va_list args;
    va_start(args, format);
    result = vprintf(format, args);
    va_end(args);
    return result;
}

int cISLogFile::vprintf(const char* format, va_list args)
{
    if (m_file) {
        return ::vfprintf(m_file,format, args);
    }
    else
    {
        return -1;
    }
}

int cISLogFile::getch()
{
    if (m_file)
    {
        return fgetc(m_file);
    }
    else
    {
        return EOF;
    }
}


std::size_t cISLogFile::read(void* bytes, std::size_t len)
{
    if (m_file)
    {
        return fread(bytes, 1, len, m_file);
    }
    else
    {
        return 0;
    }
}