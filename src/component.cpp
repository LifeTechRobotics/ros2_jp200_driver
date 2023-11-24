#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include "jp200_demo/jp200_demo_component.hpp"

#include <string>
#include <termios.h>
#include <fcntl.h>

namespace jp200_demo_component{
    JP200DemoComp::JP200DemoComp(
        const std::string &node_name,
        const rclcpp::NodeOptions& options
    ):Node("jo200_demo", node_name, options)
    {
        RCLCPP_INFO(this->get_logger(), "Start and Get Parameter");
        declare_parameter("serial_port", "/dev/ttyACM0");
        declare_parameter("baud_rate", 115200);
        get_parameter("serial_port", port_name_);
        get_parameter("baud_rate", baud_rate_);

        RCLCPP_INFO(this->get_logger(), "Open Serial port");
    }

    bool JP200DemoComp::OpenPort()
    {
        struct termios tio;

        fd_ = open(port_name_.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);

        if(fd_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("jp200_demo_serial"), "Failed to open serial port!");
        }

        bzero(&tio, sizeof(tio));

        tio.c_cflag =  CS8 | CLOCAL | CREAD;
        tio.c_iflag = IGNPAR;
        tio.c_oflag      = 0;
        tio.c_lflag      = 0;
        tio.c_cc[VTIME]  = 0;
        tio.c_cc[VMIN]   = 0;

        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSANOW, &tio);

        tx_time_per_bytes = (1000.0 / (double)baud_rate_) * 10.0;

        return true;
    }
}