#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "jp200_msgs/msg/jp200_multi_array.hpp"
#include "jp200_msgs/msg/jp200.hpp"

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
      auto send_message = jp200_msgs::msg::JP200MultiArray();
      auto msg = jp200_msgs::msg::JP200();
      send_message.servo_num = 1;
      msg.id = 1;
      msg.control_mode = 1;

      // set pwm(%)
      msg.enable_pwm = true;
      msg.pwm_cmd = 10;
      
      // set angle command
      msg.angle_cmd.enable = true;
      msg.angle_cmd.value = count;

      send_message.servos.push_back(msg);

      publisher_->publish(send_message);
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