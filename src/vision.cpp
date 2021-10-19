#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rostron_receiver/net/udp_receiver.h"
#include "ssl_vision_wrapper.pb.h"

#include "rostron_interfaces/msg/detection_frame.hpp"
#include "rostron_interfaces/msg/field.hpp"

boost::asio::io_context io;

class Vision : public rclcpp::Node
{
public:
  Vision() : Node("vision"),
             receiver_("224.5.23.2", "224.5.23.2", 10020, io,
                       [this](std::size_t length, std::array<char, 2048> data)
                       {
                         SSL_WrapperPacket vision_packet;
                         vision_packet.ParseFromArray(data.data(), length);
                         if (vision_packet.has_detection())
                         {
                           this->parseVisionPacket(vision_packet.detection());
                         }

                         if (vision_packet.has_geometry())
                         {
                           this->parseGeometryPacket(vision_packet.geometry());
                         }
                       })
  {
    publisher_vision_ = this->create_publisher<rostron_interfaces::msg::DetectionFrame>("vision", 10);
    publisher_field_ = this->create_publisher<rostron_interfaces::msg::Field>("field", 10);
  }

  void parseVisionPacket(const SSL_DetectionFrame &frame)
  {
    auto message = rostron_interfaces::msg::DetectionFrame();
    message.set__camera_id(frame.camera_id());
    message.set__frame_id(frame.frame_number());

    for (auto &ball : frame.balls())
    {
      rostron_interfaces::msg::DetectionBall ball_msg;
      ball_msg.set__confidence(ball.confidence());
      ball_msg.pixel.set__x(ball.pixel_x());
      ball_msg.pixel.set__y(ball.pixel_y());
      ball_msg.position.set__x(ball.x() / 1000);
      ball_msg.position.set__y(ball.y() / 1000);
      ball_msg.position.set__z(ball.z() / 1000);
      message.balls.push_back(ball_msg);
    }

    for (auto &r : frame.robots_yellow())
    {
      rostron_interfaces::msg::DetectionRobot r_msg;
      r_msg.set__confidence(r.confidence());
      r_msg.set__id(r.robot_id());
      r_msg.pixel.set__x(r.pixel_x());
      r_msg.pixel.set__y(r.pixel_y());
      r_msg.pose.position.set__x(r.x() / 1000);
      r_msg.pose.position.set__y(r.y() / 1000);
      r_msg.pose.orientation.set__z(r.orientation());
      message.yellow.push_back(r_msg);
    }

    for (auto &r : frame.robots_blue())
    {
      rostron_interfaces::msg::DetectionRobot r_msg;
      r_msg.set__confidence(r.confidence());
      r_msg.set__id(r.robot_id());
      r_msg.pixel.set__x(r.pixel_x());
      r_msg.pixel.set__y(r.pixel_y());
      r_msg.pose.position.set__x(r.x() / 1000);
      r_msg.pose.position.set__y(r.y() / 1000);
      r_msg.pose.orientation.set__z(r.orientation());
      message.blue.push_back(r_msg);
    }

    publisher_vision_->publish(message);
  }

  void parseGeometryPacket(const SSL_GeometryData &data)
  {
    auto message = rostron_interfaces::msg::Field();
    message.set__width(data.field().field_width() / 1000.0);
    message.set__length(data.field().field_length() / 1000.0);

    message.set__goal_width(data.field().goal_width() / 1000.0);
    message.set__goal_depth(data.field().goal_depth() / 1000.0);
    message.set__boundary_width(data.field().boundary_width() / 1000.0);
    for (int i = 0; i < data.field().field_lines_size(); i++)
    {
      if (data.field().field_lines(i).name().compare("LeftFieldLeftPenaltyStretch"))
      {
        message.set__penalty_depth(std::abs(data.field().field_lines(i).p1().x() - data.field().field_lines(i).p2().x()) / 1000.0);
        message.set__penalty_width(std::abs(2 * data.field().field_lines(i).p1().y()) / 1000.0);
        break;
      }
    }

    publisher_field_->publish(message);
  }

private:
  UDPReceiver receiver_;
  rclcpp::Publisher<rostron_interfaces::msg::DetectionFrame>::SharedPtr publisher_vision_;
  rclcpp::Publisher<rostron_interfaces::msg::Field>::SharedPtr publisher_field_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::on_shutdown([]()
                      { io.stop(); });
  auto node = std::make_shared<Vision>();
  io.run();
  rclcpp::shutdown();

  return 0;
}
