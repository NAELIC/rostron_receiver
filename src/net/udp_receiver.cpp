#include "rostron_receiver/net/udp_receiver.h"

using namespace boost;
using namespace boost::placeholders;

UDPReceiver::UDPReceiver(std::string listen_addr, std::string multicast_addr,
                         unsigned int port, asio::io_context &io_context, 
                         const std::function<void(std::size_t length, std::array<char, 2048> data)> &func)
    : socket_(io_context), func_(func)
{
    // Create the socket so that multiple may be bound to the same address.
    asio::ip::udp::endpoint listen_endpoint(asio::ip::make_address(listen_addr),
                                            port);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(asio::ip::udp::socket::reuse_address(true));
    socket_.bind(listen_endpoint);

    // Join the multicast group.
    socket_.set_option(asio::ip::multicast::join_group(
        asio::ip::make_address(multicast_addr)));

    do_receive();
}

void UDPReceiver::do_receive()
{
    socket_.async_receive_from(
        asio::buffer(data_), sender_endpoint_,
        [this](system::error_code ec, std::size_t length) {
            if (!ec)
            {
                // SSL_WrapperPacket ssl;
                // ssl.ParseFromArray(data_.data(), length);
                // std::cout << ssl.has_detection() << std::endl;
                // callback(length, data_);
                func_(length, data_);
                do_receive();
            }
        });
}