/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _IS_SDK_IS_LOG_FILE_BASE_H_
#define _IS_SDK_IS_LOG_FILE_BASE_H_

#include "ISConstants.h"
#include <cstddef>
#include <cstdarg>
#include <string>


class cISLogFileBase
{
public:
    virtual ~cISLogFileBase() {}

    virtual bool open(const char* filePath, const char* mode) = 0;
    virtual bool close() = 0;
    virtual bool flush() = 0;
    virtual bool good() = 0;
    virtual bool isOpened() = 0;

    virtual int putch(char ch) = 0;
    virtual int puts(const char* str) = 0;
    virtual std::size_t write(const void* bytes, std::size_t len) = 0;
    virtual int lprintf(const char* format, ...) = 0;
    virtual int vprintf(const char* format, va_list args) = 0;

    virtual int getch() = 0;
    virtual std::size_t read(void* bytes, std::size_t len) = 0;
};


#endif //_IS_SDK_IS_LOG_FILE_BASE_H_
