#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rostron_receiver/net/udp_receiver.h"
#include "ssl_vision_wrapper.pb.h"

#include "rostron_interfaces/msg/detection_frame.hpp"

boost::asio::io_context io;

class Vision : public rclcpp::Node
{
public:
  Vision() : Node("vision"),
             receiver_("224.5.23.2", "224.5.23.2", 10020, io,
                       Vision::parseVisionPacket)
  {
    publisher_ = this->create_publisher<rostron_interfaces::msg::DetectionFrame>("topic", 10);
  }

  static void parseVisionPacket(std::size_t length, std::array<char, 2048> data)
  {
    SSL_WrapperPacket vision_packet;
    vision_packet.ParseFromArray(data.data(), length);
    auto message = rostron_interfaces::msg::DetectionFrame();

    if (vision_packet.has_detection())
    {
      auto &frame = vision_packet.detection();

      for (auto &ball : frame.balls())
      {
        rostron_interfaces::msg::DetectionBall ball_msg;
        ball_msg.confidence = ball.confidence();
        ball_msg.pixel.set__x(ball.pixel_x());
        ball_msg.pixel.set__y(ball.pixel_y());
        ball_msg.position.set__x(ball.x());
        ball_msg.position.set__y(ball.y());
        message.balls.push_back(ball_msg);
      }
    }

    // Vision::publisher_->publish(message);
  }

private:
  UDPReceiver receiver_;
  static rclcpp::Publisher<rostron_interfaces::msg::DetectionFrame>::SharedPtr publisher_;
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
