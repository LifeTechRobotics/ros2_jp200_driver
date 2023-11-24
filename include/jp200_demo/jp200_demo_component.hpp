#ifndef JP200_DEMO_COMPONENT_HPP_
#define JP200_DEMO_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <string>
#include <vector>
#include "termios.h"

namespace jp200_demo_component {
    class JP200DemoComp : rclcpp::Node{
        public:
            JP200DemoComp(
                const std::string &node_name="",
                const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
            );

            bool OpenPort();

        private:
            std::string port_name_;
            int baud_rate_;
            int fd_;
            double tx_time_per_bytes;
            std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> cmd_subscribers_;
            std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> state_publishers_;
    };
}

#endif