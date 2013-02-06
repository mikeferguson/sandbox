/*
 * Copyright (c) 2013, Michael E. Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * based on http://www.boost.org/doc/libs/1_53_0/doc/html/boost_asio/tutorial.html
 *      and http://www.boost.org/doc/libs/1_53_0/doc/html/boost_asio/tutorial/tuttimer3.html
 */

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <std_msgs/Int32.h>

using boost::asio::ip::udp;

class udp_driver
{
public:
  udp_driver(boost::asio::io_service& io_service,
             ros::NodeHandle nh)
    : socket_(io_service),
      milliseconds_(10),
      timer_(io_service, boost::posix_time::milliseconds(milliseconds_))
  {
    timer_.async_wait(boost::bind(&udp_driver::updateCallback, this, _1));

    pub_ = nh.advertise<std_msgs::Int32>("count", 1);
    count_ = 0;

    // any socket will do
    socket_.open(udp::v4());
    start_receive();
  }

  void updateCallback(const boost::system::error_code& /*e*/)
  {
    // kick the hardware to send some stuff back
    udp::endpoint receiver_endpoint = udp::endpoint(boost::asio::ip::address::from_string("10.42.0.8"), 5048);
    try
    {
      boost::array<char, 1> send_buf  = {{ 0 }};
      socket_.send_to(boost::asio::buffer(send_buf), receiver_endpoint);
    } 
    catch (std::exception& e)
    {
      std::cerr << e.what() << std::endl;
    }
    ROS_INFO("sent commands");

    // need to set a new expiration time before calling async_wait again
    timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(milliseconds_));
    timer_.async_wait(boost::bind(&udp_driver::updateCallback, this, _1));
  }

private:
  void start_receive()
  {
    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        boost::bind(&udp_driver::handle_receive, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

  void handle_receive(const boost::system::error_code& error,
    std::size_t bytes_transferred)
  {
    // This catches all the data, just print it for demo
    if (!error || error == boost::asio::error::message_size)
    {
      recv_buffer_[bytes_transferred] = 0; // tiny hack
      std::string data(recv_buffer_.begin(), recv_buffer_.end());
      ROS_INFO("%s", data.c_str());
      if(ros::ok())
        start_receive();
      count_++;
      std_msgs::Int32 msg;
      msg.data = count_;
      pub_.publish(msg);
    }

    // ROS catches the ctrl-c, but we need to stop the io_service to exit
    //   now works after switching to a boost timer
    if(!ros::ok())
      socket_.get_io_service().stop();
  }

  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  boost::array<char, 255> recv_buffer_;

  // 100hz/1khz timer for sending new data request
  long milliseconds_;
  boost::asio::deadline_timer timer_;

  // publish number of message received
  ros::Publisher pub_;
  int count_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "asio_sim_driver");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  try
  {
    // io_service and "driver" class do all the heavy lifting
    boost::asio::io_service io_service;
    udp_driver driver(io_service, nh);

    // for ROS_INFO, pub, subs...
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // will block here until the end
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}

