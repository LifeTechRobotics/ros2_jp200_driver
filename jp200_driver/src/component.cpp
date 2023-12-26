#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include <chrono>
#include <iostream>
#include <string>

#include <jp200_driver/component.hpp>

using std::placeholders::_1;

namespace jp200_driver{

    JP200Component::JP200Component(const rclcpp::NodeOptions& options)
    :Node("jp200_demo", options)
    {
        RCLCPP_INFO(this->get_logger(), "Set Parameter");
        declare_parameter("serial_port", "/dev/ttyACM0");
        declare_parameter("baud_rate", 115200);
        get_parameter("serial_port", port_name_);
        get_parameter("baud_rate", baud_rate_);

        RCLCPP_INFO(this->get_logger(), "get Utils instance");

        // add subscriber
        RCLCPP_INFO(this->get_logger(), "create subscriber");
        cmd_subscriber_ = this->create_subscription<jp200_msgs::msg::JP200>("/jp200_cmd", 0, std::bind(&JP200Component::callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Open Serial port");

        fd_ = open_port();
        if(fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger() , "Failed to open port");
        }else{
            RCLCPP_INFO(this->get_logger(), "Serial port was connected");
        }
    }

    void JP200Component::callback(const jp200_msgs::msg::JP200 msg)
    {
        command_.id = msg.id;
        command_.control_mode = msg.control_mode;

        command_.angle.enable = msg.angle_cmd.enable;
        command_.angle.value = msg.angle_cmd.value;

        command_.velocity.enable = msg.velocity_cmd.enable;
        command_.velocity.value = msg.velocity_cmd.value;

        command_.current.enable = msg.current_cmd.enable;
        command_.current.value = msg.current_cmd.value;

        command_.pwm_enable = msg.enable_pwm;
        command_.pwm_rate = msg.pwm_cmd;

        command_.angle.get_state = msg.state.enable_get_angle;
        command_.velocity.get_state = msg.state.enable_get_velocity;
        command_.current.get_state = msg.state.enable_get_current;
        command_.get_pwm = msg.state.enable_get_pwm;
        command_.get_mpu_temp = msg.state.enable_get_mpu_temp;
        command_.get_amp_temp = msg.state.enable_get_amp_temp;
        command_.get_motor_temp = msg.state.enable_get_motor_temp;
        command_.get_status = msg.state.enable_get_status;
        command_.get_voltage = msg.state.enable_get_voltage;

        command_.position_gain.enable = msg.position_gain.enable;
        command_.position_gain.p = msg.position_gain.p;
        command_.position_gain.i = msg.position_gain.i;
        command_.position_gain.d = msg.position_gain.d;
        command_.position_gain.f = msg.position_gain.f;

        command_.velocity_gain.enable = msg.velocity_gain.enable;
        command_.velocity_gain.p = msg.velocity_gain.p;
        command_.velocity_gain.i = msg.velocity_gain.i;
        command_.velocity_gain.d = msg.velocity_gain.d;
        command_.velocity_gain.f = msg.velocity_gain.f;

        command_.current_gain.enable = msg.current_gain.enable;
        command_.current_gain.p = msg.current_gain.p;
        command_.current_gain.i = msg.current_gain.i;
        command_.current_gain.d = msg.current_gain.d;
        command_.current_gain.f = msg.current_gain.f;

        tx_packet_ = utils.createJp200Cmd(command_);

        write_serial();

    }

    int JP200Component::open_port()
    {
        int fd=open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        fcntl(fd, F_SETFL,0);
        struct termios conf_tio;
        tcgetattr(fd,&conf_tio);
        speed_t BAUDRATE = B115200;
        cfsetispeed(&conf_tio, BAUDRATE);
        cfsetospeed(&conf_tio, BAUDRATE);
        conf_tio.c_lflag &= ~(ECHO | ICANON);
        conf_tio.c_cc[VMIN]=0;
        conf_tio.c_cc[VTIME]=0;
        tcsetattr(fd,TCSANOW,&conf_tio);
        return fd;
    }

    void JP200Component::write_serial()
    {
        int error = write(fd_, tx_packet_.data(), tx_packet_.size());

        if(error < 0){
            RCLCPP_ERROR(this->get_logger(), "Failed to write");
        }
    }

}