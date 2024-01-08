#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include <chrono>
#include <iostream>
#include <string>

#include "jp200_driver/component.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace jp200_driver{

    JP200Component::JP200Component(const rclcpp::NodeOptions& options)
    :Node("jp200_demo", options)
    {
        RCLCPP_INFO(this->get_logger(), "Set Parameter");

        declare_parameter("serial_port", "/dev/ttyACM0");
        declare_parameter("baud_rate", 115200);
        declare_parameter("enable_servo_response", true);
        declare_parameter("servo_num", 1);
        get_parameter("serial_port", port_name_);
        get_parameter("baud_rate", baud_rate_);
        get_parameter("enable_servo_response", enable_servo_response);
        get_parameter("servo_num", servo_num);

        // add subscriber
        RCLCPP_INFO(this->get_logger(), "Initialize node");
        // cmd_subscriber_ = this->create_subscription<jp200_msgs::msg::JP200>("/jp200_cmd", 0, std::bind(&JP200Component::callback, this, _1));
        if(enable_servo_response)
        {
            read_timer_ = this->create_wall_timer(50ms, std::bind(&JP200Component::read_serial, this));
            // state_publisher_ = this->create_publisher<jp200_msgs::msg::Response>("/state", 0);
        }
        RCLCPP_INFO(this->get_logger(), "Open Serial port");
        RCLCPP_INFO(this->get_logger(), "port:%s, baud rate:%d, enable servo response:%s", port_name_.c_str(), baud_rate_, std::to_string(enable_servo_response).c_str());

        fd_ = open_port();
        if(fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger() , "Failed to open port");
            this->close_port();
        }else{
            RCLCPP_INFO(this->get_logger(), "Serial port was connected <%d>", fd_);
        }
    }

    void JP200Component::add_subscriber()
    {
        for(int i = 0; i < servo_num; i++)
        {
            commands_.push_back(jp200_driver::JP200Utils::JP200Cmd());
            std::string topic_name = "servo_command/" + i;
            auto sub = this->create_subscription<jp200_msgs::msg::JP200>(topic_name, 0,
                [this, i](const jp200_msgs::msg::JP200 msg)
                {
                    commands_[i].id = msg.id;
                    commands_[i].control_mode = msg.control_mode;

                    commands_[i].angle.enable = msg.angle_cmd.enable;
                    commands_[i].angle.value = msg.angle_cmd.value;

                    commands_[i].velocity.enable = msg.velocity_cmd.enable;
                    commands_[i].velocity.value = msg.velocity_cmd.value;

                    commands_[i].current.enable = msg.current_cmd.enable;
                    commands_[i].current.value = msg.current_cmd.value;

                    commands_[i].pwm_enable = msg.enable_pwm;
                    commands_[i].pwm_rate = msg.pwm_cmd;

                    commands_[i].angle.get_state = msg.state.enable_get_angle;
                    commands_[i].velocity.get_state = msg.state.enable_get_velocity;
                    commands_[i].current.get_state = msg.state.enable_get_current;
                    commands_[i].get_pwm = msg.state.enable_get_pwm;
                    commands_[i].get_mpu_temp = msg.state.enable_get_mpu_temp;
                    commands_[i].get_amp_temp = msg.state.enable_get_amp_temp;
                    commands_[i].get_motor_temp = msg.state.enable_get_motor_temp;
                    commands_[i].get_status = msg.state.enable_get_status;
                    commands_[i].get_voltage = msg.state.enable_get_voltage;

                    commands_[i].position_gain.enable = msg.position_gain.enable;
                    commands_[i].position_gain.p = msg.position_gain.p;
                    commands_[i].position_gain.i = msg.position_gain.i;
                    commands_[i].position_gain.d = msg.position_gain.d;
                    commands_[i].position_gain.f = msg.position_gain.f;

                    commands_[i].velocity_gain.enable = msg.velocity_gain.enable;
                    commands_[i].velocity_gain.p = msg.velocity_gain.p;
                    commands_[i].velocity_gain.i = msg.velocity_gain.i;
                    commands_[i].velocity_gain.d = msg.velocity_gain.d;
                    commands_[i].velocity_gain.f = msg.velocity_gain.f;

                    commands_[i].current_gain.enable = msg.current_gain.enable;
                    commands_[i].current_gain.p = msg.current_gain.p;
                    commands_[i].current_gain.i = msg.current_gain.i;
                    commands_[i].current_gain.d = msg.current_gain.d;
                    commands_[i].current_gain.f = msg.current_gain.f;
                }
            );

            cmd_subscribers_.push_back(sub);
            
        }
    }

    void JP200Component::add_publisher()
    {
        
    }

    int JP200Component::open_port()
    {
        int fd=open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        fcntl(fd, F_SETFL,0);
        struct termios conf_tio;
        tcgetattr(fd,&conf_tio);
        speed_t BAUDRATE = get_baud_rate();
        cfsetispeed(&conf_tio, BAUDRATE);
        cfsetospeed(&conf_tio, BAUDRATE);
        conf_tio.c_lflag &= ~(ECHO | ICANON);
        conf_tio.c_cc[VMIN]=0;
        conf_tio.c_cc[VTIME]=0;
        tcsetattr(fd,TCSANOW,&conf_tio);
        return fd;
    }

    void JP200Component::close_port()
    {
        RCLCPP_INFO(this->get_logger(), "Close port");
        close(fd_);
    }

    int JP200Component::write_serial()
    {
        if(fd_ < 0)
        {
            return -1;
        }
        const char *packet = tx_packet_.c_str();
        return write(fd_, packet, strlen(packet));
    }

    int JP200Component::read_serial()
    {
        if(fd_ < 0)
        {
            return -1;
        }
        char buf[100];
        ssize_t bytes_read = read(fd_, buf, sizeof(buf) - 1);
        if(bytes_read < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read port");
            return bytes_read;
        }
        else
        {
            buf[bytes_read] = '\0';
            rx_packet_ = buf;
            RCLCPP_INFO(this->get_logger(), "read %s from {%d}", rx_packet_.c_str(), fd_);
            return bytes_read;
        }
    }

    speed_t JP200Component::get_baud_rate()
    {
        switch(baud_rate_)
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