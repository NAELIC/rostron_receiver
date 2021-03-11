#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rostron_receiver/net/udp_receiver.h"
#include "ssl_gc_referee_message.pb.h"

boost::asio::io_context io;

class GameController : public rclcpp::Node
{
public:
  GameController() : Node("vision"),
             receiver_("0.0.0.0", "224.5.23.1", 10003, io,
                       GameController::parseGCPacket)
  {
  }

  static void parseGCPacket(std::size_t length, std::array<char, 2048> data)
  {
    Referee gc_packet;
    gc_packet.ParseFromArray(data.data(), length);
    std::cout << "Detection : " << gc_packet.has_stage() << std::endl;
    if(gc_packet.has_stage()) {
        std::cout << "Stage :" << gc_packet.stage() << std::endl;
    }
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
  auto node = std::make_shared<GameController>();
  io.run();
  rclcpp::shutdown();

  return 0;
}
