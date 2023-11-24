#ifndef JP200_DEMO_HPP_
#define JP200_DEMO_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
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
    };
}

#endif