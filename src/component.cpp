#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "jp200_demo/jp200_demo.hpp"
#include <string>

namespace jp200_demo{
    JP200DemoNode::JP200DemoNode(
        const std::string &node_name,
        const rclcpp::NodeOptions& options
    ):Node("jo200_demo", node_name, options)
    {
        RCLCPP_INFO(this->get_logger(), "Start JP200 demo node");
    }
}