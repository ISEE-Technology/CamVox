/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISConstants.h"

#if PLATFORM_IS_LINUX || PLATFORM_IS_APPLE

/* Assume that any non-Windows platform uses POSIX-style sockets instead. */
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <netdb.h>  /* Needed for getaddrinfo() and freeaddrinfo() */
#include <unistd.h> /* Needed for close() */
#include <fcntl.h>
#include <errno.h>

#endif

#include "ISTcpClient.h"
#include "ISUtilities.h"

#if PLATFORM_IS_WINDOWS

#include <mutex>

static uint32_t s_socketInitialized;
static mutex s_socketFrameworkMutex;

#endif

void ISSocketFrameworkInitialize()
{

#if PLATFORM_IS_WINDOWS

	lock_guard<mutex> locker(s_socketFrameworkMutex);
	if (++s_socketInitialized != 1)
	{
		return;
	}
	WSADATA wsa_data;
	WSAStartup(MAKEWORD(2, 2), &wsa_data);

#endif

}

void ISSocketFrameworkShutdown()
{

#if PLATFORM_IS_WINDOWS

	lock_guard<mutex> locker(s_socketFrameworkMutex);
	if (s_socketInitialized == 0)
	{
		return;
	}
	else if (--s_socketInitialized == 0)
	{
		WSACleanup();
	}

#endif

}

int ISSocketCanWrite(socket_t socket, int timeoutMilliseconds)
{
    struct timeval tv = { timeoutMilliseconds / 1000, (timeoutMilliseconds % 1000) * 1000 };
    fd_set ws;
    FD_ZERO(&ws);
    FD_SET(socket, &ws);
    int numberOfSocketsThatCanWrite = select((int)(socket + 1), NULL, &ws, NULL, &tv);
    return (numberOfSocketsThatCanWrite > 0);
}

int ISSocketCanRead(socket_t socket, int timeoutMilliseconds)
{
	struct timeval tv = { timeoutMilliseconds / 1000, (timeoutMilliseconds % 1000) * 1000 };
	fd_set rs;
	FD_ZERO(&rs);
	FD_SET(socket, &rs);
    int numberOfSocketsThatCanRead = select((int)(socket + 1), &rs, NULL, NULL, &tv);
	return (numberOfSocketsThatCanRead > 0);
}

int ISSocketWrite(socket_t socket, const uint8_t* data, int dataLength)
{
    int totalWriteCount = 0;
    int writeCount;

    while (totalWriteCount < dataLength)
	{
        if (!ISSocketCanWrite(socket))
        {
            break;
        }
        writeCount = send(socket, (const char*)data, dataLength, 0);
        if (writeCount < 0)
        {
            break;
        }
        totalWriteCount += writeCount;
	}

    return totalWriteCount;
}

int ISSocketRead(socket_t socket, uint8_t* data, int dataLength)
{
	int count = recv(socket, (char*)data, dataLength, 0);
	if (count < 0)
	{

#if PLATFORM_IS_WINDOWS

		DWORD err = GetLastError();
		if (err == SO_RCVTIMEO || err == WSAETIMEDOUT || err == WSAEWOULDBLOCK)
		{
			return 0;
		}

#else

		if (errno == EAGAIN || errno == EWOULDBLOCK)
		{
			return 0;
		}

#endif

	}
	return count;
}

int ISSocketSetBlocking(socket_t socket, bool blocking)
{

#if PLATFORM_IS_WINDOWS

	u_long blockingInt = (blocking ? 0 : 1);
	return ioctlsocket(socket, FIONBIO, &blockingInt);

#else

	int opts = fcntl(socket, F_GETFL);
	if (opts < 0)
	{
		return -1;
	}
	opts = (blocking ? opts & (~O_NONBLOCK) : opts | O_NONBLOCK);
	return fcntl(socket, F_SETFL, opts);

#endif

}

int ISSocketSetReadTimeout(socket_t socket, int timeoutMilliseconds)
{

#if PLATFORM_IS_WINDOWS

	DWORD timeout = timeoutMilliseconds;
	return setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));

#else

	struct timeval tv;
	tv.tv_sec = timeoutMilliseconds / 1000;
	tv.tv_usec = (timeoutMilliseconds % 1000) * 1000;
	return setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

#endif

}

int ISSocketClose(socket_t& socket)
{
	int status = 0;
	if (socket != 0)
	{

#if PLATFORM_IS_WINDOWS

		status = shutdown(socket, SD_BOTH);
		if (status == 0)
		{
			status = closesocket(socket);
		}

#else

		status = shutdown(socket, SHUT_RDWR);
		if (status == 0)
		{
			status = close(socket);
		}

#endif

	}

	socket = 0;

	return status;
}

cISTcpClient::cISTcpClient()
{
	m_socket = 0;
	m_port = 0;
	m_blocking = true;
	ISSocketFrameworkInitialize();
}

cISTcpClient::~cISTcpClient()
{
	Close();
	ISSocketFrameworkShutdown();
}

int cISTcpClient::Open(const string& host, int port, int timeoutMilliseconds)
{
	Close();
	int status;
	this->m_host = host;
	m_port = port;
	char portString[64];
	snprintf(portString, sizeof(portString), "%ld", (long)m_port);
	addrinfo* result = NULL;
	addrinfo hints = addrinfo();
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

    // attempt to get info about the host and port
	status = getaddrinfo(m_host.c_str(), portString, &hints, &result);
	if (status != 0)
	{
        // no info, fail
		Close();
		return status;
	}

    // create the socket
	m_socket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (m_socket == 0)
	{
        // no socket, fail
		freeaddrinfo(result);
		Close();
		return -1;
	}

    // make non-blocking socket
    SetBlocking(false);

    // because non-blocking, connect returns immediately
    connect(m_socket, result->ai_addr, (int)result->ai_addrlen);

    // in order to detect connection we determine whether the socket is writeable
    status = ISSocketCanWrite(m_socket, timeoutMilliseconds);

    // cleanup
	freeaddrinfo(result);
    if (status <= 0)
	{
        // no socket was writeable, fail
		Close();
        return -1;
	}

    return 0;
}

int cISTcpClient::Close()
{
	return ISSocketClose(m_socket);
}

int cISTcpClient::Read(void* data, int dataLength)
{
	int count = ISSocketRead(m_socket, (uint8_t*)data, dataLength);
	if (count < 0)
	{
		Close();
	}
	return count;
}

int cISTcpClient::Write(const void* data, int dataLength)
{
	int count = ISSocketWrite(m_socket, (const uint8_t*)data, dataLength);
	if (count < 0)
	{
		Close();
	}
	return count;
}

void cISTcpClient::HttpGet(const string& subUrl, const string& userAgent, const string& userName, const string& password)
{
	string msg = "GET /" + subUrl + " HTTP/1.1\r\n";
	msg += "User-Agent: " + userAgent + "\r\n";
	if (userName.size() != 0 && password.size() != 0)
	{
		string auth = userName + ":" + password;
		msg += "Authorization: Basic " + base64Encode((const unsigned char*)auth.data(), (int)auth.size()) + "\r\n";
	}
	msg += "Accept: */*\r\nConnection: close\r\n\r\n";
	Write((uint8_t*)msg.data(), (int)msg.size());
}

int cISTcpClient::SetBlocking(bool blocking)
{
	m_blocking = blocking;
	return ISSocketSetBlocking(m_socket, blocking);
}
