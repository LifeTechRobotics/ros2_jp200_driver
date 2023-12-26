#include "jp200_driver/component.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<jp200_driver::JP200Component>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}