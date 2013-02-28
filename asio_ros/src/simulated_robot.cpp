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
 */

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "asio_sim_robot");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  try
  {
    boost::asio::io_service io_service;

    /* open a upd socket on 5048 */
    udp::socket socket(io_service, udp::endpoint(udp::v4(), 5048));
    ROS_INFO("Robot started");

    /* stow endpoint for multiple returns */
    udp::endpoint remote_endpoint;
      
    while(ros::ok())
    {
      boost::array<char, 255> recv_buf;
      boost::system::error_code error;
      socket.receive_from(boost::asio::buffer(recv_buf),
          remote_endpoint, 0, error);

      if (error && error != boost::asio::error::message_size)
        throw boost::system::system_error(error);

      boost::system::error_code ignored_error;
      // send a message
      socket.send_to(boost::asio::buffer("robot_state"),
          remote_endpoint, 0, ignored_error);
      // now several more
      socket.send_to(boost::asio::buffer("mcb1"),
          remote_endpoint, 0, ignored_error);
      socket.send_to(boost::asio::buffer("mcb2"),
          remote_endpoint, 0, ignored_error);
      socket.send_to(boost::asio::buffer("mcb3"),
          remote_endpoint, 0, ignored_error);
      socket.send_to(boost::asio::buffer("mcb4"),
          remote_endpoint, 0, ignored_error);
      socket.send_to(boost::asio::buffer("mcb5"),
          remote_endpoint, 0, ignored_error);
      socket.send_to(boost::asio::buffer("mcb6"),
          remote_endpoint, 0, ignored_error);
      socket.send_to(boost::asio::buffer("mcb7"),
          remote_endpoint, 0, ignored_error);
      socket.send_to(boost::asio::buffer("mcb8"),
          remote_endpoint, 0, ignored_error);
      socket.send_to(boost::asio::buffer("mcb9"),
          remote_endpoint, 0, ignored_error);


      //ROS_INFO("recv message");
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
