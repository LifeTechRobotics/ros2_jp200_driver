#include <rclcpp/rclcpp.hpp>
#include "visibility.h"
#include <string>

namespace jp200_demo_component {
    class JP200DemoComp : rclcpp::Node{
        public:
            JP200DemoComp(
                const std::string &node_name="",
                const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
            );
    };
}