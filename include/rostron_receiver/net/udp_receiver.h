#pragma once

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>
#include <queue>
#include <string>

typedef void(*ProtoFunction)(std::size_t length, std::array<char, 2048> data);

class UDPReceiver {
   public:
    UDPReceiver(std::string listen_addr, std::string multicast_addr,
              unsigned int port, boost::asio::io_context& io_context, ProtoFunction func);
    void do_receive();

    // std::queue<grSim_Packet> packets;

   private:
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint sender_endpoint_;
    std::array<char, 2048> data_;
    ProtoFunction func_;
};