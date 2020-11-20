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

#include "ISTcpServer.h"
#include "ISUtilities.h"

cISTcpServer::cISTcpServer(iISTcpServerDelegate* delegate)
{
	ISSocketFrameworkInitialize();
	m_delegate = delegate;
	m_socket = 0;
	m_port = 0;
}

cISTcpServer::~cISTcpServer()
{
	Close();
	ISSocketFrameworkShutdown();
}

int cISTcpServer::Open(const string& ipAddress, int port)
{
	m_ipAddress = ipAddress;
	m_port = port;
	int status;
	char portString[64];
	snprintf(portString, sizeof(portString), "%ld", (long)m_port);
	addrinfo* result = NULL;
	addrinfo hints = addrinfo();
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;
	status = getaddrinfo(m_ipAddress.length() == 0 ? NULL : m_ipAddress.c_str(), portString, &hints, &result);
	if (status != 0)
	{
		Close();
		return status;
	}

	// setup socket
	m_socket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (m_socket == 0)
	{
		freeaddrinfo(result);
		Close();
		return -1;
	}

	int enable = 1;
	if (setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&enable, sizeof(enable)) < 0)
	{
		freeaddrinfo(result);
		Close();
		return -1;
	}

	// setup listener socket
	status = ::bind(m_socket, result->ai_addr, (int)result->ai_addrlen);
	if (status != 0)
	{
		freeaddrinfo(result);
		Close();
		return -1;
	}

	freeaddrinfo(result);

	status = listen(m_socket, SOMAXCONN);
	if (status != 0)
	{
		Close();
		return -1;
	}

	return status;
}

int cISTcpServer::Close()
{
	int status = ISSocketClose(m_socket);
	for (size_t i = 0; i < m_clients.size(); i++)
	{
		status |= ISSocketClose(m_clients[i]);
	}
	m_clients.clear();
	return status;
}

void cISTcpServer::Update()
{
	uint8_t readBuff[8192];

	// accept new sockets
    while (ISSocketCanRead(m_socket, 1))
	{
		if (m_delegate != NULLPTR)
		{
			m_delegate->OnClientConnecting(this);
		}
		socket_t socket = accept(m_socket, NULLPTR, NULLPTR);
		if (socket != 0)
		{
			ISSocketSetBlocking(socket, false);
			m_clients.push_back(socket);
			if (m_delegate != NULLPTR)
			{
				m_delegate->OnClientConnected(this, socket);
			}
		}
		else if (m_delegate != NULLPTR)
		{
			m_delegate->OnClientConnectFailed(this);
		}
	}

	for (size_t i = 0; i < m_clients.size(); i++)
	{
        if (ISSocketCanRead(m_clients[i], 1))
		{
			int count;
			if ((count = ISSocketRead(m_clients[i], readBuff, sizeof(readBuff))) < 0)
			{
				// remove the client
				if (m_delegate != NULLPTR)
				{
					m_delegate->OnClientDisconnected(this, m_clients[i]);
				}
				ISSocketClose(m_clients[i]);
				m_clients.erase(m_clients.begin() + i--);
			}
			else if (count > 0 && m_delegate != NULLPTR)
			{
				m_delegate->OnClientDataReceived(this, m_clients[i], readBuff, count);
			}
		}
	}
}

int cISTcpServer::Write(const uint8_t* data, int dataLength)
{
	for (size_t i = 0; i < m_clients.size(); i++)
	{
		int written = 0;
		int count;
		while (written < dataLength)
		{
			count = ISSocketWrite(m_clients[i], data + written, dataLength - written);
			if (count < 1)
			{
				// remove the client
				if (m_delegate != NULLPTR)
				{
					m_delegate->OnClientDisconnected(this, m_clients[i]);
				}
				ISSocketClose(m_clients[i]);
				m_clients.erase(m_clients.begin() + i--);
				break;
			}
			else
			{
				written += count;
				if (written == dataLength)
				{
					break;
				}
			}
		}
	}
	return dataLength; // TODO: Maybe be smarter about detecting difference in bytes written for each client
}
