/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_SDK_CREATE_IS_LOG_FILE_H_
#define IS_SDK_CREATE_IS_LOG_FILE_H_

#include "ISConstants.h"

#if PLATFORM_IS_EVB_2
#include "ISLogFileFatFs.h"
#else
#include "ISLogFile.h"
#endif


inline cISLogFileBase* CreateISLogFile()
{
#if PLATFORM_IS_EVB_2
    return new cISLogFileFatFs;
#else
    return new cISLogFile;
#endif
}

inline cISLogFileBase* CreateISLogFile(const char* filePath, const char* mode)
{
#if PLATFORM_IS_EVB_2
    return new cISLogFileFatFs(filePath, mode);
#else
    return new cISLogFile(filePath, mode);
#endif
}

inline cISLogFileBase* CreateISLogFile(const std::string& filePath, const char* mode)
{
#if PLATFORM_IS_EVB_2
    return new cISLogFileFatFs(filePath, mode);
#else
    return new cISLogFile(filePath, mode);
#endif
}

inline void CloseISLogFile(cISLogFileBase*& logFile)
{
    if (logFile != NULLPTR)
    {
        logFile->close();
        delete logFile;
        logFile = NULLPTR;
    }
}


#endif //IS_SDK_CREATE_IS_LOG_FILE_H_
