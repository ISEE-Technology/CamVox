/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISStream.h"
#include "ISUtilities.h"

#ifndef _FILE_OFFSET_BITS
#define _FILE_OFFSET_BITS 64
#endif

cISStream::~cISStream()
{
	Close();
}

cISFileStream::cISFileStream()
{
	m_file = NULLPTR;
}

bool cISFileStream::Open(const std::string& path, const char* mode)
{
	Close();
	m_file = openFile(path.c_str(), mode);
	return (m_file != NULLPTR);
}

int cISFileStream::Read(void* buffer, int count)
{
	if (m_file != NULLPTR)
	{
		return (int)fread(buffer, 1, count, m_file);
	}
	return -1;
}

int cISFileStream::Write(const void* buffer, int count)
{
	if (m_file != NULLPTR)
	{
		return (int)fwrite(buffer, 1, count, m_file);
	}
	return -1;
}

int cISFileStream::Flush()
{
	if (m_file != NULLPTR)
	{
		return fflush(m_file);
	}
	return -1;
}

int cISFileStream::Close()
{
	int status = 0;
	if (m_file != NULLPTR)
	{
		status = fclose(m_file);
		m_file = NULLPTR;
	}
	return status;
}

long long cISFileStream::GetBytesAvailableToRead()
{
	if (m_file != NULLPTR)
	{

#if PLATFORM_IS_WINDOWS

		long long pos = _ftelli64(m_file);
		_fseeki64(m_file, 0, SEEK_END);
		long long fileSize = _ftelli64(m_file);
		_fseeki64(m_file, pos, SEEK_SET);
		return fileSize - pos;

#else

		long long pos = ftello(m_file);
		fseeko(m_file, 0, SEEK_END);
		long long fileSize = ftello(m_file);
		fseeko(m_file, pos, SEEK_SET);
		return fileSize - pos;

#endif

	}
	return 0;
}

bool cISFileStream::Eof()
{
	if (m_file != NULLPTR)
	{
		return (feof(m_file) != 0);
	}
	return true;
}
