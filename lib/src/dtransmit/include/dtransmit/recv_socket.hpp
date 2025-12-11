/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 */

/**
 * @file recv_socket.hpp
 * @author Yusu Pan
 * @version 2018
 * @date 2018-06-26
 */

#pragma once

#include <functional>

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>

// http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
#define UDPBUFFERSIZE 65535

namespace dtransmit {

typedef int PORT;

// TODO maybe use pair
/**
 * @brief Wrapper for socket transmit
 */
struct Foo {
  //! Socket instance
  boost::asio::ip::udp::socket *socket;
  //! handler for receiving messages
  std::function<void(const boost::system::error_code &, std::size_t)>
      readHandler;
  //! received buffer
  uint8_t recvBuffer[UDPBUFFERSIZE];
  //! remote endpoint address
  boost::asio::ip::udp::endpoint remoteEndpoint;

  /**
   * @brief Foo constructor (blank)
   */
  Foo() {}

  /**
   * @brief Foo constructor
   *
   * Wrapper of socket to listen on specified IP with reusable port.
   * https://stackoverflow.com/a/39665940
   *
   * @param service - io service
   * @param port - port
   */
  Foo(boost::asio::io_service &service, PORT port) {
    // construct the socket
    socket = new boost::asio::ip::udp::socket(service);

    // open it
    boost::asio::ip::udp::endpoint rx_endpoint_(boost::asio::ip::udp::v4(),
                                                port);
    boost::system::error_code error;
    socket->open(rx_endpoint_.protocol(), error);
    if (error) {
      ROS_ERROR("Can't open recv socket");
    } else {
      // then set it for reuse and bind it
      socket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
      socket->bind(rx_endpoint_, error);
      if (error) {
        ROS_ERROR("Can't bind recv socket");
      }
    }
  }

  /**
   * @brief Foo destructor
   */
  ~Foo() {}
};

} // namespace dtransmit

