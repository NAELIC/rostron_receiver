#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rostron_receiver/net/udp_receiver.h"
boost::asio::io_context io;

class Vision : public rclcpp::Node
{
public:
  Vision() : Node("vision"),
             receiver_("224.5.23.2", "224.5.23.2", 10020, io,
                       Vision::parseSimPacket)
  {
  }

  static void parseSimPacket(std::size_t length, std::array<char, 2048> data)
  {
    SSL_WrapperPacket sim_packet;
    sim_packet.ParseFromArray(data.data(), length);
    std::cout << "Detection : " << sim_packet.has_detection() << std::endl;
  }

private:
  UDPReceiver receiver_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::on_shutdown([]() {
    io.stop();
  });
  auto node = std::make_shared<Vision>();
  io.run();
  rclcpp::shutdown();

  return 0;
}
