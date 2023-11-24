#include "jp200_demo/jp200_demo_component.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<jp200_demo_component::JP200DemoComp>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}