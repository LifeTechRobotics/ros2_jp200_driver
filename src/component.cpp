#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include <jp200_demo/jp200_demo_component.hpp>

#include <string>
#include <termios.h>
#include <fcntl.h>

namespace jp200_demo_component{

    JP200DemoComp::JP200DemoComp(const rclcpp::NodeOptions& options)
    :Node("jo200_demo", options)
    {
        RCLCPP_INFO(this->get_logger(), "Start and Get Parameter");
        declare_parameter("serial_port", "/dev/ttyACM0");
        declare_parameter("baud_rate", 115200);
        get_parameter("serial_port", port_name_);
        get_parameter("baud_rate", baud_rate_);

        RCLCPP_INFO(this->get_logger(), "Open Serial port");
    }

    bool JP200DemoComp::openPort(int cflag_baud)
    {
        struct termios tio;

        fd_ = open(port_name_.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);

        if(fd_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("jp200_demo_serial"), "Failed to open serial port!");
            return false;
        }

        bzero(&tio, sizeof(tio));

        tio.c_cflag = cflag_baud |CS8 | CLOCAL | CREAD;
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

    void JP200DemoComp::closePort()
    {
        if(fd_ != -1)close(fd_);
        fd_ = -1;
    }
    void JP200DemoComp::clearPort()
    {
        tcflush(fd_, TCIFLUSH);
    }

    int JP200DemoComp::readPort(uint8_t *buffer, int length)
    {
        return read(fd_, buffer, length);
    }
    int JP200DemoComp::writePort(uint8_t *buffer, int length)
    {
        return write(fd_, buffer, length);
    }

    bool JP200DemoComp::setBaudRate(const int baud_rate)
    {
        int baud = getCFlagBaud(baud_rate);

        clearPort();

        if(baud <= 0)
        {
            openPort(B38400);
            baud_rate_ = baud_rate;
        }
        else
        {
            baud_rate_ = baud_rate;
            return openPort(baud);
        }
    }

    int JP200DemoComp::getCFlagBaud(int baudrate)
    {
        switch(baudrate)
        {
            case 9600:
            return B9600;
            case 19200:
            return B19200;
            case 38400:
            return B38400;
            case 57600:
            return B57600;
            case 115200:
            return B115200;
            case 230400:
            return B230400;
            case 460800:
            return B460800;
            case 500000:
            return B500000;
            case 576000:
            return B576000;
            case 921600:
            return B921600;
            case 1000000:
            return B1000000;
            case 1152000:
            return B1152000;
            case 1500000:
            return B1500000;
            case 2000000:
            return B2000000;
            case 2500000:
            return B2500000;
            case 3000000:
            return B3000000;
            case 3500000:
            return B3500000;
            case 4000000:
            return B4000000;
            default:
            return -1;
        }
    }

}