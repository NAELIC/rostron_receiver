#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rostron_receiver/net/udp_receiver.h"
boost::asio::io_context io;

class Vision : public rclcpp::Node
{
public:
  Vision() : Node("vision"),
             receiver_("224.5.23.2", "224.5.23.2", 10020, io)
  {
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
