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
        add_subscriber();
        write_timer_ = this->create_wall_timer(50ms, std::bind(&JP200Component::write_serial));

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

    
}