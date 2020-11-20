/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISFileManager.h"
#include <regex>


#if PLATFORM_IS_EVB_2
#include <ff.h>
#else
#include <sys/stat.h>
#include <cstdio>
#include <time.h>
#endif

#if PLATFORM_IS_LINUX || PLATFORM_IS_APPLE
#include <sys/statvfs.h>
#endif

namespace ISFileManager {

    bool PathIsDir(const std::string& path)
    {
#if PLATFORM_IS_EVB_2
        FILINFO info;
        FRESULT result = f_stat(path.c_str(), &info);
        return (result == FR_OK) && (info.fattrib & AM_DIR);
#else
        struct stat sb;
        return (stat(path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode));
#endif
    }

    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, const std::string& regexPattern, std::vector<std::string>& files)
	{
		size_t startSize = files.size();
		std::regex* rePtr = NULL;
		std::regex re;
		if (regexPattern.length() != 0)
		{
			re = std::regex(regexPattern, std::regex::icase);
			rePtr = &re;
		}

#if PLATFORM_IS_EVB_2

		{
			FRESULT result;
			FILINFO info;
			DIR dir;
			char *file_name;
#if _USE_LFN
			static char longFileName[_MAX_LFN + 1];
			info.lfname = longFileName;
			info.lfsize = sizeof(longFileName);
#endif


			result = f_opendir(&dir, directory.c_str());
			if (result == FR_OK) {
				while (true) {
					result = f_readdir(&dir, &info);
					if (result != FR_OK || info.fname[0] == 0)
					{
						break;
					}
#if _USE_LFN
					file_name = *info.lfname ? info.lfname : info.fname;
#else
					file_name = info.fname;
#endif
					std::string full_file_name = directory + "/" + file_name;


					if (file_name[0] == '.' || (rePtr != NULL && !regex_search(full_file_name, re)))
					{
						continue;
					}
					else if (info.fattrib & AM_DIR) {
						if (recursive)
						{
							GetAllFilesInDirectory(full_file_name, true, files);
						}
						continue;
					}
					files.push_back(full_file_name);
				}
			}

		}

#elif PLATFORM_IS_WINDOWS

		HANDLE dir;
		WIN32_FIND_DATAA file_data;
		if ((dir = FindFirstFileA((directory + "/*").c_str(), &file_data)) == INVALID_HANDLE_VALUE)
		{
			return false;
		}
		do
		{
			std::string file_name = file_data.cFileName;
			std::string full_file_name = directory + "/" + file_name;
			if (file_name[0] == '.' || (rePtr != NULL && !regex_search(full_file_name, re)))
			{
				continue;
			}
			else if ((file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0)
			{
				if (recursive)
				{
					GetAllFilesInDirectory(full_file_name, true, files);
				}
				continue;
			}
			files.push_back(full_file_name);
		} while (FindNextFileA(dir, &file_data));
		FindClose(dir);

#else // Linux

		class dirent* ent;
		class stat st;
		DIR* dir = opendir(directory.c_str());

		if (dir == NULL) {
			return false;
		}

		while ((ent = readdir(dir)) != NULL) {
			const std::string file_name = ent->d_name;
			const std::string full_file_name = directory + "/" + file_name;

			// if file is current path or does not exist (-1) then continue
			if (file_name[0] == '.' || stat(full_file_name.c_str(), &st) == -1 ||
				(rePtr != NULL && !regex_search(full_file_name, re))) {
				continue;
			}
			else if ((st.st_mode & S_IFDIR) != 0) {
				if (recursive) {
					GetAllFilesInDirectory(full_file_name, true, files);
				}
				continue;
			}

			files.push_back(full_file_name);
		}
		closedir(dir);

#endif

		return (files.size() != startSize);
	}

    bool GetAllFilesInDirectory(const std::string& directory, bool recursive, std::vector<std::string>& files)
    {
        return GetAllFilesInDirectory(directory, recursive, "", files);
    }

    bool DeleteFile(const std::string& fullFilePath)
    {
#if PLATFORM_IS_EVB_2
        return (f_unlink(fullFilePath.c_str()) == FR_OK);
#else
        return (std::remove(fullFilePath.c_str()) == 0);
#endif

    }

    void DeleteDirectory(const std::string& directory, bool recursive)
    {

#if PLATFORM_IS_EVB_2
        // Always recursive
        f_unlink(directory.c_str());
#elif PLATFORM_IS_WINDOWS

        HANDLE dir;
        WIN32_FIND_DATAA file_data;
        if ((dir = FindFirstFileA((directory + "/*").c_str(), &file_data)) == INVALID_HANDLE_VALUE)
        {
            return;
        }
        do
        {
            const std::string file_name = file_data.cFileName;
            const std::string full_file_name = directory + "/" + file_name;
            const bool is_directory = (file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
            if (file_name[0] == '.')
            {
                continue;
            }
            else if (is_directory)
            {
                if (recursive)
                {
                    DeleteDirectory(full_file_name, true);
                }
                continue;
            }
            std::remove(full_file_name.c_str());
        } while (FindNextFileA(dir, &file_data));
        FindClose(dir);

#else

        class dirent* ent;
        class stat st;

        DIR* dir;
        if ((dir = opendir(directory.c_str())) == NULL) {
            return;
        }

        while ((ent = readdir(dir)) != NULL) {
            const std::string file_name = ent->d_name;
            const std::string full_file_name = directory + "/" + file_name;

            // if file is current path or does not exist (-1) then continue
            if (file_name[0] == '.' || stat(full_file_name.c_str(), &st) == -1) {
                continue;
            } else if ((st.st_mode & S_IFDIR) != 0) {
                if (recursive) {
                    DeleteDirectory(full_file_name, true);
                }
                continue;
            }
            std::remove(full_file_name.c_str());
        }
        closedir(dir);

#endif

        _RMDIR(directory.c_str());
    }

    uint64_t GetDirectorySpaceUsed(const std::string& directory, bool recursive)
    {
        std::vector<file_info_t> files;
        return GetDirectorySpaceUsed(directory, "", files, true, recursive);
    }


    uint64_t GetDirectorySpaceUsed(const std::string& directory, std::vector<file_info_t>& files, bool sortByDate, bool recursive)
    {
        return GetDirectorySpaceUsed(directory, "", files, sortByDate, recursive);
    }

    uint64_t GetDirectorySpaceUsed(const std::string& directory, std::string regexPattern, std::vector<file_info_t>& files,
                                              bool sortByDate, bool recursive)
    {
#if PLATFORM_IS_EVB_2
        static_assert((sizeof(time_t) >= 2 * sizeof(WORD)), "time_t must be at least 2* DWORD, since we store the FatFs date/time (DWORD) in a time_t here.");
#endif

        std::vector<std::string> fileNames;
        ISFileManager::GetAllFilesInDirectory(directory, recursive, regexPattern, fileNames);
        uint64_t spaceUsed = 0;
        for (unsigned int i = 0; i < fileNames.size(); i++)
		{
            file_info_t info;
            info.name = fileNames[i];
#if PLATFORM_IS_EVB_2
            FILINFO filInfo;
            f_stat(info.name.c_str(), &filInfo);
            info.size = filInfo.fsize;
            info.lastModificationDate = static_cast<time_t>(filInfo.ftime);
            info.lastModificationDate |= static_cast<time_t>(filInfo.fdate) << (sizeof(filInfo.ftime) * 8);
#else
            struct stat st;
            stat(info.name.c_str(), &st);
            info.size = st.st_size;
            info.lastModificationDate = st.st_mtime;
#endif
            files.push_back(info);
            spaceUsed += info.size;
        }

        if (sortByDate) {
            struct
            {
                bool operator()(const file_info_t& a, const file_info_t& b)
                {
                    int compare = (a.lastModificationDate == b.lastModificationDate ? 0 : (a.lastModificationDate <
                                                                                           b.lastModificationDate ? -1
                                                                                                                  : 1));
                    if (compare == 0) {
                        compare = a.name.compare(b.name);
                    }
                    return compare < 0;
                }
            } customSortDate;
            sort(files.begin(), files.end(), customSortDate);
        } else {
            struct
            {
                bool operator()(const file_info_t& a, const file_info_t& b)
                {
                    return a.name < b.name;
                }
            } customSortName;
            sort(files.begin(), files.end(), customSortName);
        }

        return spaceUsed;
    }

    uint64_t GetDirectorySpaceAvailable(const std::string& directory)
    {
#if PLATFORM_IS_EVB_2
        FATFS *fs;
	DWORD free_clusters;
	DWORD sector_size;

	/* Get volume information and free clusters of drive 1 */
	FRESULT result = f_getfree("0:", &free_clusters, &fs);
	if (result != FR_OK)
	{
		return 0;
	}
#if _MAX_SS == 512
	sector_size = 512;
#else
	sector_size = fs->ssize;
#endif

	return free_clusters * fs->csize * sector_size;

#elif PLATFORM_IS_WINDOWS

        ULARGE_INTEGER space;
	memset(&space, 0, sizeof(space));
	char fullPath[MAX_PATH];
	GetFullPathNameA(directory.c_str(), MAX_PATH, fullPath, NULL);
	bool created = (_MKDIR(fullPath) == 0);
	GetDiskFreeSpaceExA(fullPath, &space, NULL, NULL);
	if (created)
	{
		_RMDIR(fullPath);
	}
	return (uint64_t)space.QuadPart;

#else

        struct statvfs stat;
        memset(&stat, 0, sizeof(stat));
        char fullPath[PATH_MAX];
        if (realpath(directory.c_str(), fullPath) == NULL)
        {
            return 0;
        }
        bool created = (_MKDIR(fullPath) == 0);
        statvfs(fullPath, &stat);
        if (created)
        {
            _RMDIR(fullPath);
        }
        return (uint64_t)stat.f_bsize * (uint64_t)stat.f_bavail;

#endif

    }

    std::string GetFileName(const std::string& path)
    {
        size_t lastSepIndex = path.find_last_of("\\/");
        if (lastSepIndex != std::string::npos)
        {
            return path.substr(lastSepIndex + 1);
        }
        return path;
    }

    bool TouchFile(const std::string& path)
    {
#if PLATFORM_IS_EVB_2
        DWORD datetime = get_fattime();
        FILINFO info;
        info.fdate = static_cast<WORD>(datetime >> sizeof(WORD));
        info.ftime = static_cast<WORD>(datetime);
        f_utime(path.c_str(), &info);
		return true;
#else
        _UTIMEBUF buf{time(NULL), time(NULL)};
        return (_UTIME(path.c_str(), &buf) == 0);
#endif
    }

} // namespace ISFileManager