/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_SDK_IS_FILE_MANAGER_H_
#define IS_SDK_IS_FILE_MANAGER_H_

#include "ISConstants.h"
#include <string>
#include <vector>
#include <ctime>

namespace ISFileManager
{
    typedef struct
    {
        std::string name;
        uint64_t size;
        time_t lastModificationDate;
    } file_info_t;

    /**
     * Is this path directory?
     * @param path the path to check
     * @return true if the path is a directory, false otherwise
     */
    bool PathIsDir(const std::string& path);

    // get all files in a folder - files parameter is not cleared in this function
    // files contains the full path to the file
    // return false if no files found, true otherwise
    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, std::vector<std::string>& files);
    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, const std::string& regexPattern, std::vector<std::string>& files);

    bool DeleteFile(const std::string& fullFilePath);
    void DeleteDirectory(const std::string& directory, bool recursive = true);

    // get space used in a directory recursively - files is filled with all files in the directory, sorted by modification date, files is NOT cleared beforehand. sortByDate of false sorts by file name
    uint64_t GetDirectorySpaceUsed(const std::string& directory, bool recursive = true);
    uint64_t GetDirectorySpaceUsed(const std::string& directory, std::vector<file_info_t>& files, bool sortByDate = true, bool recursive = true);
    uint64_t GetDirectorySpaceUsed(const std::string& directory, std::string regexPattern, std::vector<file_info_t>& files, bool sortByDate = true, bool recursive = true);

    // get free space for the disk that the specified directory exists on
    uint64_t GetDirectorySpaceAvailable(const std::string& directory);

    // get just the file name from a path
    std::string GetFileName(const std::string& path);

    bool TouchFile(const std::string& path);

}

#endif //IS_SDK_IS_FILE_MANAGER_H_
