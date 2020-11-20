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
 * \file mavlink_comm.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 * \author James Jackson <superjax08@gmail.com>
 */

#include <serial.h>


using boost::asio::serial_port_base;

Serial::Serial(std::string port, int baud_rate) :
  io_service_(),
  write_in_progress_(false),
  serial_port_(io_service_),
  port_(port),
  baud_rate_(baud_rate)
{
  listener_ = NULL;
}

Serial::~Serial()
{
}

void Serial::open()
{
  // open the port
  try
  {
    serial_port_.open(port_);
    serial_port_.set_option(serial_port_base::baud_rate(baud_rate_));
    serial_port_.set_option(serial_port_base::character_size(8));
    serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
  }
  catch (boost::system::system_error e)
  {
    throw SerialException(e);
  }

  // start reading from the port
  async_read();
  io_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &this->io_service_));
}

void Serial::close()
{
  mutex_lock lock(mutex_);

  io_service_.stop();
  serial_port_.close();

  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void Serial::register_listener(SerialListener * const listener)
{
  if (listener == NULL || listener_ != NULL)
    return;
  else
    listener_ = listener;
}

void Serial::async_read()
{
  if (!serial_port_.is_open()) return;

  serial_port_.async_read_some(
        boost::asio::buffer(read_buf_raw_, BUFFER_SIZE),
        boost::bind(
          &Serial::async_read_end,
          this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}

void Serial::async_read_end(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (!serial_port_.is_open()) return;

  if (error)
  {
    close();
    return;
  }

  listener_->handle_bytes(read_buf_raw_, bytes_transferred);

  async_read();
}

void Serial::write(const uint8_t* bytes, uint8_t len)
{
  assert(len <= BUFFER_SIZE); //! \todo Do something less catastrophic here

  WriteBuffer *buffer = new WriteBuffer();
  buffer->len = len;
  for (int i = 0; i < len; i++)
  {
    buffer->data[i] = bytes[i];
  }

  {
    mutex_lock lock(mutex_);
    write_queue_.push_back(buffer);
  }

  async_write(true);
}

void Serial::async_write(bool check_write_state)
{
  if (check_write_state && write_in_progress_)
    return;

  mutex_lock lock(mutex_);
  if (write_queue_.empty())
    return;

  write_in_progress_ = true;
  WriteBuffer *buffer = write_queue_.front();
  serial_port_.async_write_some(
        boost::asio::buffer(buffer->dpos(), buffer->nbytes()),
        boost::bind(
          &Serial::async_write_end,
          this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));

}

void Serial::async_write_end(const boost::system::error_code &error, std::size_t bytes_transferred)
{
  if (error)
  {
    std::cerr << error.message() << std::endl;
    close();
    return;
  }

  mutex_lock lock(mutex_);
  if (write_queue_.empty())
  {
    write_in_progress_ = false;
    return;
  }

  WriteBuffer *buffer = write_queue_.front();
  buffer->pos += bytes_transferred;
  if (buffer->nbytes() == 0)
  {
    write_queue_.pop_front();
    delete buffer;
  }

  if (write_queue_.empty())
    write_in_progress_ = false;
  else
    async_write(false);
}
