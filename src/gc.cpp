#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rostron_receiver/net/udp_receiver.h"
#include "ssl_gc_referee_message.pb.h"

#include "rostron_interfaces/msg/referee.hpp"

boost::asio::io_context io;

class GameController : public rclcpp::Node
{
public:
  GameController() : Node("gamecontroller"),
                     receiver_("224.5.23.1", "224.5.23.1", 10003, io,
                               [this](std::size_t length, std::array<char, 2048> data) {
                                 Referee gc_packet;
                                 gc_packet.ParseFromArray(data.data(), length);
                                 parseGCPacket(gc_packet);
                               })
  {
    publisher_gc_ = this->create_publisher<rostron_interfaces::msg::Referee>("gc", 10);
  }

  void parseGCPacket(const Referee &frame)
  {
    auto message = rostron_interfaces::msg::Referee();
    message.set__stage(frame.stage());
    message.set__blue_team_on_positive_half(frame.blue_team_on_positive_half());
    message.set__command(frame.command());

    publisher_gc_->publish(message);
  }

private:
  UDPReceiver receiver_;
  rclcpp::Publisher<rostron_interfaces::msg::Referee>::SharedPtr publisher_gc_;
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
