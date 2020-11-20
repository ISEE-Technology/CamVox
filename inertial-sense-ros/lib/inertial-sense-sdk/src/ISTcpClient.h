/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __ISTCPCLIENT__H__
#define __ISTCPCLIENT__H__

#include <string>
#include <inttypes.h>

#include "ISStream.h"

#define IS_SOCKET_DEFAULT_TIMEOUT_MS 5000

using namespace std;

class cISTcpClient : public cISStream
{
public:
	/**
	* Constructor
	*/
	cISTcpClient();

	/**
	* Destructor
	*/
	virtual ~cISTcpClient();

	/**
	* Closes, then opens a tcp client
	* @param host the host or ip address to connect to
	* @param port the port to connect to on the host
    * @param timeoutMilliseconds the max milliseconds to wait for a successful connection before aborting
	* @return 0 if success, otherwise an error code
	*/
    int Open(const string& host, int port, int timeoutMilliseconds = IS_SOCKET_DEFAULT_TIMEOUT_MS);

	/**
	* Close the client
	* @return 0 if success, otherwise an error code
	*/
	int Close() OVERRIDE;

	/**
	* Read data from the client
	* @param data the buffer to read data into
	* @param dataLength the number of bytes available in data
	* @return the number of bytes read or less than 0 if error
	*/
	int Read(void* data, int dataLength) OVERRIDE;

	/**
	* Write data to the client
	* @param data the data to write
	* @param dataLength the number of bytes to write
	* @return the number of bytes written or less than 0 if error
	*/
	int Write(const void* data, int dataLength) OVERRIDE;

	/**
    * Send a GET http request to a url. You must then call Read to get the response. SSL is NOT supported.
	* @param subUrl the url to request, i.e. index.html or pages/page1.txt, etc.
	* @param userAgent the user agent to send
	* @param userName optional user name (basic authentication)
	* @param password optional password (basic authentication)
	*/
	void HttpGet(const string& subUrl, const string& userAgent, const string& userName, const string& password);

	/**
	* Get whether the connection is open
	* @return true if connection open, false if not
	*/
	bool IsOpen() { return m_socket != 0; }

	/**
	* Get whether the client socket is blocking - blocking reads do not return until the data is read or a timeout occurs. Default is false.
	* @return whether the client is a blocking socket
	*/
	bool GetBlocking() { return m_blocking; }

	/**
	* Sets whether the client socket is blocking. Default is false.
	* @return 0 if success otherwise an error code
	*/
	int SetBlocking(bool blocking);

private:
	cISTcpClient(const cISTcpClient& copy); // Disable copy constructor

	socket_t m_socket;
	string m_host;
	int m_port;
	bool m_blocking;
};

/**
* Initialize socket framework - called automatically by ISTcpClient and ISTcpServer
*/
void ISSocketFrameworkInitialize();

/**
* Shutdown socket framework - called automatically by ISTcpClient and ISTcpServer
*/
void ISSocketFrameworkShutdown();

/**
* Determines if a socket can be written to
* @param socket the socket to check for write capability
* @param timeoutMilliseconds the number of milliseconds to wait before aborting
* @return non-zero if the socket can be written to, otherwise zero
*/
int ISSocketCanWrite(socket_t socket, int timeoutMilliseconds = IS_SOCKET_DEFAULT_TIMEOUT_MS);

/**
* Determines if a socket can be read from
* @param socket the socket to check for read capability
* @param timeoutMilliseconds the number of milliseconds to wait before aborting
* @return non-zero if the socket can be read from, otherwise zero
*/
int ISSocketCanRead(socket_t socket, int timeoutMilliseconds = IS_SOCKET_DEFAULT_TIMEOUT_MS);

/**
* Write data to a socket
* @param socket the socket to write to
* @param data the data to write
* @param dataLength the number of bytes in data
* @return the number of bytes written or less than 0 if error, in which case the socket is probably disconnected
*/
int ISSocketWrite(socket_t socket, const uint8_t* data, int dataLength);

/**
* Read data from a socket
* @param socket the socket to read from
* @param data the buffer to read data into
* @param dataLength the number of bytes available in data
* @return the number of bytes read or less than 0 if error, in which case the socket is probably disconnected
*/
int ISSocketRead(socket_t socket, uint8_t* data, int dataLength);

/**
* Sets whether a socket is blocking. When reading, a blocking socket waits for the specified amount of data until the timeout is reached, a non-blocking socket returns immediately with the number of bytes read.
* @param socket the socket to set blocking for
* @param blocking whether the socket is blocking
* @return 0 if success otherwise an error code
*/
int ISSocketSetBlocking(socket_t socket, bool blocking);

/**
* Set a read timeout on a socket. This function is only useful for blocking sockets, where it is highly recommended.
* @param socket the socket to set the read timeout on
* @param timeoutMilliseconds the timeout in milliseconds
* @return 0 if success otherwise an error code
*/
int ISSocketSetReadTimeout(socket_t socket, int timeoutMilliseconds);

/**
* Close a socket and zero it out
* @param socket the socket to close, this will be zeroed out
* @return 0 if success otherwise an error code
*/
int ISSocketClose(socket_t& socket);

#endif // __ISTCPCLIENT__H__
