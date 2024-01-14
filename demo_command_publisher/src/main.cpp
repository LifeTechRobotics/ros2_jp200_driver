#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "jp200_msgs/msg/jp200_multi_array.hpp"

using namespace std::chrono_literals;

class DemoCommandPublisher : public rclcpp::Node
{
  public:
    DemoCommandPublisher()
    : Node("demo_command_publisher")
    {
      publisher_ = this->create_publisher<jp200_msgs::msg::JP200MultiArray>("/cmd", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&DemoCommandPublisher::callback, this));
    }

  private:
    void callback()
    {
      auto message = jp200_msgs::msg::JP200MultiArray();
      message.servo_num = 1;
      message.servos[0].id = 1;
      message.servos[0].control_mode = 1;

      // set pwm(%)
      message.servos[0].enable_pwm = true;
      message.servos[0].pwm_cmd = 10;
      
      // set angle command
      message.servos[0].angle_cmd.enable = true;
      message.servos[0].angle_cmd.value = count;

      publisher_->publish(message);
      count += 10;
    }

    rclcpp::Publisher<jp200_msgs::msg::JP200MultiArray>::SharedPtr publisher_;
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