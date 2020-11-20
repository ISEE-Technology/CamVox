/*
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file mavlink_comm.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVLINK_COMM_H
#define MAVROSFLIGHT_MAVLINK_COMM_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <list>
#include <string>
#include <vector>

#include <stdint.h>

#define BUFFER_SIZE 2048

class SerialListener
{
public:
  virtual void handle_bytes(const uint8_t* bytes, uint8_t len) = 0;
};


class Serial
{
public:

  /**
   * \brief Instantiates the class and begins communication on the specified serial port
   * \param port Name of the serial port (e.g. "/dev/ttyUSB0")
   * \param baud_rate Serial communication baud rate
   */
  Serial(std::__cxx11::string port, int baud_rate);

  /**
   * \brief Stops communication and closes the serial port before the object is destroyed
   */
  ~Serial();

  /**
   * \brief Opens the port and begins communication
   */
  void open();

  /**
   * \brief Stops communication and closes the port
   */
  void close();

  /**
   * \brief write data
   * \param buffer The message to send
   * \param len The number of bytes
   */
  void write(const uint8_t *buffer, uint8_t len);

  /**
   * \brief Register a listener for received bytes
   * \param listener Pointer to an object that implements the SerialListener interface
   */
  void register_listener(SerialListener * const listener);

  boost::asio::io_service io_service_; //!< boost io service provider

private:

  //===========================================================================
  // definitions
  //===========================================================================

  /**
   * \brief Struct for buffering the contents of a mavlink message
   */
  struct WriteBuffer
  {
    uint8_t data[BUFFER_SIZE];
    size_t len;
    size_t pos;

    WriteBuffer() : len(0), pos(0) {}

    WriteBuffer(const uint8_t * buf, uint16_t len) : len(len), pos(0)
    {
      assert(len <= BUFFER_SIZE); //! \todo Do something less catastrophic here
      memcpy(data, buf, len);
    }

    const uint8_t * dpos() const { return data + pos; }

    size_t nbytes() const { return len - pos; }
  };

  /**
   * \brief Pointer to byte listener
   */
  SerialListener* listener_;

  boost::asio::serial_port serial_port_; //!< boost serial port object
  std::string port_;
  int baud_rate_;

  /**
   * \brief Convenience typedef for mutex lock
   */
  typedef boost::lock_guard<boost::recursive_mutex> mutex_lock;

  //===========================================================================
  // methods
  //===========================================================================

  /**
   * \brief Initiate an asynchronous read operation
   */
  void async_read();

  /**
   * \brief Handler for end of asynchronous read operation
   * \param error Error code
   * \param bytes_transferred Number of bytes received
   */
  void async_read_end(const boost::system::error_code& error, size_t bytes_transferred);

  /**
   * \brief Initialize an asynchronous write operation
   * \param check_write_state If true, only start another write operation if a write sequence is not already running
   */
  void async_write(bool check_write_state);

  /**
   * \brief Handler for end of asynchronous write operation
   * \param error Error code
   * \param bytes_transferred Number of bytes sent
   */
  void async_write_end(const boost::system::error_code& error, size_t bytes_transferred);

  //===========================================================================
  // member variables
  //===========================================================================

  boost::thread io_thread_; //!< thread on which the io service runs
  boost::recursive_mutex mutex_; //!< mutex for threadsafe operation

  uint8_t sysid_;
  uint8_t compid_;

  uint8_t read_buf_raw_[BUFFER_SIZE];

  std::list<WriteBuffer*> write_queue_; //!< queue of buffers to be written to the serial port
  bool write_in_progress_; //!< flag for whether async_write is already running
};

class SerialException : public std::exception
{
public:
  explicit SerialException(const char * const description)
  {
    init(description);
  }

  explicit SerialException(const std::string &description)
  {
    init(description.c_str());
  }

  explicit SerialException(const boost::system::system_error &err)
  {
    init(err.what());
  }

  SerialException(const SerialException &other) : what_(other.what_) {}

  ~SerialException() throw() {}

  virtual const char* what() const throw()
  {
    return what_.c_str();
  }

private:
  std::string what_;

  void init(const char * const description)
  {
    std::ostringstream ss;
    ss << "Serial Error: " << description;
    what_ = ss.str();
  }
};

#endif
