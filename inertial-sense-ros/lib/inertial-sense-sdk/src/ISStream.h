/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __ISSTREAM_H__
#define __ISSTREAM_H__

#include <stdio.h>
#include <string>
#include <iostream>
#include <string>
#include <inttypes.h>

#include "ISConstants.h"

/**
* Interface to read and write data using stream interfaces
*/
class cISStream
{
public:
	/** Constructor */
	cISStream() {}

	/**
	* Destructor
	* If this instance owns the reader and/or writer streams, they are deleted
	*/
	virtual ~cISStream();

	/**
	* Read n bytes from the stream
	* @param buffer the buffer to read into
	* @param count the max count of bytes to read
	* @return the number of bytes read, -1 if reading is not supported, 0 if no bytes available
	*/
    virtual int Read(void* buffer, int count) { (void)buffer; (void)count; return -1; }

	/**
	* Write n bytes from the stream to buffer
	* @param buffer the buffer to write into
	* @param count the max count of bytes to write
	* @return the number of bytes written, -1 if writing is not supported, 0 if no bytes written
	*/
    virtual int Write(const void* buffer, int count) { (void)buffer; (void)count; return -1; }

	/**
	* Flush the stream
	* @return 0 if success, otherwise an error code
	*/
	virtual int Flush() { return -1; }

	/**
	* Close the stream
	* @return 0 if success, otherwise an error code
	*/
	virtual int Close() { return -1; }

	/**
	* Gets the number of bytes available to read
	* @return The number of bytes available to read or -1 if this feature is not supported
	*/
	virtual long long GetBytesAvailableToRead() { return -1; }

private:
	cISStream(const cISStream& copy); // Disable copy constructor
};

class cISFileStream : public cISStream
{
public:
	/** Constructor */
	cISFileStream();

	/**
	* Open a file. If the file is writeable and does not exist, it is created. If the file was already opened, it is closed first.
	* @param path the file path
	* @param mode the file open mode (i.e. r or rw, etc.)
	* @return true if success, false if failure
	*/
	bool Open(const std::string& path, const char* mode);

	/**
	* Read n bytes from the file
	* @param buffer the buffer to read into
	* @param count the max count of bytes to read
	* @return the number of bytes read, -1 if reading is not supported, 0 if no bytes available
	*/
	int Read(void* buffer, int count) OVERRIDE;

	/**
	* Write n bytes from the file to buffer
	* @param buffer the buffer to write into
	* @param count the max count of bytes to write
	* @return the number of bytes written, -1 if writing is not supported, 0 if no bytes written
	*/
	int Write(const void* buffer, int count) OVERRIDE;

	/**
	* Flush the file, causing any pending writes to be written to disk
	* @return 0 if success, otherwise an error code
	*/
	int Flush() OVERRIDE;

	/**
	* Close the file
	* @return 0 if success, otherwise an error code
	*/
	int Close() OVERRIDE;

	/**
	* Gets the number of bytes available to read
	* @return The number of bytes available to read or -1 if this feature is not supported
	*/
	long long GetBytesAvailableToRead() OVERRIDE;

	/**
	* Gets whether the file has hit end of file during read
	* @return true if end of file, false otherwise
	*/
	bool Eof();

private:
	cISFileStream(const cISFileStream& copy); // Disable copy constructor

	FILE* m_file;
};

#endif // __BASE_STREAM_H__
