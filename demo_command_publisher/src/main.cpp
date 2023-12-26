#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "jp200_msgs/msg/jp200.hpp"

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class DemoCommandPublisher : public rclcpp::Node
{
  public:
    DemoCommandPublisher()
    : Node("demo_command_publisher")
    {
      publisher_ = this->create_publisher<jp200_msgs::msg::JP200>("/cmd", 10);
      subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 0, std::bind(&DemoCommandPublisher::callback, this, _1));
    }

  private:
    void callback(const sensor_msgs::msg::Joy::SharedPtr get_msg)
    {
      auto message = jp200_msgs::msg::JP200();
      message.id = 1;
      message.id = 1;

      message.angle_cmd.enable = true;
      message.angle_cmd.value = std::atan(get_msg->axes[0] / get_msg->axes[1]);

      publisher_->publish(message);
    }

    rclcpp::Publisher<jp200_msgs::msg::JP200>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoCommandPublisher>());
  rclcpp::shutdown();
  return 0;
}