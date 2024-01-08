#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "jp200_msgs/msg/jp200.hpp"

using namespace std::chrono_literals;

class DemoCommandPublisher : public rclcpp::Node
{
  public:
    DemoCommandPublisher()
    : Node("demo_command_publisher")
    {
      publisher_ = this->create_publisher<jp200_msgs::msg::JP200>("/cmd", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&DemoCommandPublisher::callback, this));
    }

  private:
    void callback()
    {
      auto message = jp200_msgs::msg::JP200();
      message.id = 1;
      message.control_mode = 1;

      // set pwm(%)
      message.enable_pwm = true;
      message.pwm_cmd = 20;
      
      // set angle command
      message.angle_cmd.enable = true;
      message.angle_cmd.value = count;

      message.state.enable_get_angle = true;

      publisher_->publish(message);
      count += 10;
    }

    rclcpp::Publisher<jp200_msgs::msg::JP200>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoCommandPublisher>());
  rclcpp::shutdown();
  return 0;
}