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
    :Node("jp200_driver_node", options)
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

        RCLCPP_INFO(this->get_logger(), "Get JP200 Utils instance");
        utils = jp200_driver::JP200Utils();

        RCLCPP_INFO(this->get_logger(), "Initialize node");
        if(servo_num == 1)
        {
            auto cmd = JP200Utils::JP200Cmd();
            commands_.push_back(cmd);
            cmd_subscriber_0 = this->create_subscription<jp200_msgs::msg::JP200>(
                "servo/_0", 
                0, 
                std::bind(&JP200Component::single_motor_callback, this, _1)
            );
        }
        

        timer_ = this->create_wall_timer(1000ms, std::bind(&JP200Component::timer_callback, this));

        for(int i = 0; i < servo_num; i++)
        {
            std::string topic_name = "servo_state/_" + std::to_string(i);
            state_publishers_.push_back(this->create_publisher<jp200_msgs::msg::Response>(topic_name, 0));
            RCLCPP_INFO(this->get_logger(), "Add publisher topic name: %s", topic_name.c_str()); 
        }
        RCLCPP_INFO(this->get_logger(), "Open Serial port");
        int fd_ = utils.open_port(port_name_, baud_rate_);
        RCLCPP_INFO(this->get_logger(), "port:%s, baud rate:%d, enable servo response:%s", port_name_.c_str(), baud_rate_, std::to_string(enable_servo_response).c_str());

        if(fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger() , "Failed to open port");
            utils.close_port(fd_);
        }else{
            RCLCPP_INFO(this->get_logger(), "Serial port was connected <%d>", fd_);
        }

    }

    void JP200Component::timer_callback()
    {
        tx_packet_ = utils.createJp200Cmd(commands_, enable_servo_response);
        int write = utils.write_serial(fd_, tx_packet_);
        if(write >= 0)
        {
            RCLCPP_INFO(this->get_logger(), "Write %s to %d", tx_packet_.c_str(), fd_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Failed to write : %s size: %ld", tx_packet_.c_str(), strlen(tx_packet_.c_str()));
        }

        // rx_packet_ = utils.read_serial(fd_);
        // RCLCPP_INFO(this->get_logger(), "Read packet : %s", rx_packet_.c_str());
    }

    void JP200Component::single_motor_callback(const jp200_msgs::msg::JP200 msg)
    {
        commands_[0].id = msg.id;
        commands_[0].control_mode = msg.control_mode;

        commands_[0].angle.enable = msg.angle_cmd.enable;
        commands_[0].angle.value = msg.angle_cmd.value;

        commands_[0].velocity.enable = msg.velocity_cmd.enable;
        commands_[0].velocity.value = msg.velocity_cmd.value;

        commands_[0].current.enable = msg.current_cmd.enable;
        commands_[0].current.value = msg.current_cmd.value;

        commands_[0].pwm_enable = msg.enable_pwm;
        commands_[0].pwm_rate = msg.pwm_cmd;

        commands_[0].angle.get_state = msg.state.enable_get_angle;
        commands_[0].velocity.get_state = msg.state.enable_get_velocity;
        commands_[0].current.get_state = msg.state.enable_get_current;
        commands_[0].get_pwm = msg.state.enable_get_pwm;
        commands_[0].get_mpu_temp = msg.state.enable_get_mpu_temp;
        commands_[0].get_amp_temp = msg.state.enable_get_amp_temp;
        commands_[0].get_motor_temp = msg.state.enable_get_motor_temp;
        commands_[0].get_status = msg.state.enable_get_status;
        commands_[0].get_voltage = msg.state.enable_get_voltage;

        commands_[0].position_gain.enable = msg.position_gain.enable;
        commands_[0].position_gain.p = msg.position_gain.p;
        commands_[0].position_gain.i = msg.position_gain.i;
        commands_[0].position_gain.d = msg.position_gain.d;
        commands_[0].position_gain.f = msg.position_gain.f;

        commands_[0].velocity_gain.enable = msg.velocity_gain.enable;
        commands_[0].velocity_gain.p = msg.velocity_gain.p;
        commands_[0].velocity_gain.i = msg.velocity_gain.i;
        commands_[0].velocity_gain.d = msg.velocity_gain.d;
        commands_[0].velocity_gain.f = msg.velocity_gain.f;

        commands_[0].current_gain.enable = msg.current_gain.enable;
        commands_[0].current_gain.p = msg.current_gain.p;
        commands_[0].current_gain.i = msg.current_gain.i;
        commands_[0].current_gain.d = msg.current_gain.d;
        commands_[0].current_gain.f = msg.current_gain.f;

        tx_packet_ = utils.createJp200Cmd(commands_, enable_servo_response);
        int error = utils.write_serial(fd_, tx_packet_);
        if(error > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Write %s", tx_packet_.c_str());
        }
    }
}