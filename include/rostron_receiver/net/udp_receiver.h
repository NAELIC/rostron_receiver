#pragma once

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>
#include <queue>
#include <string>

#include "ssl_vision_wrapper.pb.h"

class UDPReceiver {
   public:
    UDPReceiver(std::string listen_addr, std::string multicast_addr,
              unsigned int port, boost::asio::io_context& io_context);
    void do_receive();

    // std::queue<grSim_Packet> packets;

   private:
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint sender_endpoint_;
    std::array<char, 2048> data_;
};