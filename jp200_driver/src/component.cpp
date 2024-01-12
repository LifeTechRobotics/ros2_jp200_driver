#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <unistd.h>

#include "jp200_driver/component.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace jp200_driver;

    JP200Component::JP200Component(const rclcpp::NodeOptions& options)
    :Node("jp200_driver_node", options)
    {   
        RCLCPP_INFO(this->get_logger(), "Set Parameter");

        declare_parameter("serial_port", "/dev/ttyACM0");
        declare_parameter("baud_rate", 115200);
        declare_parameter("enable_servo_response", true);
        declare_parameter("servo_num", 1);
        get_parameter("serial_port", port_name);
        get_parameter("baud_rate", baud_rate);
        get_parameter("enable_servo_response", enable_servo_response);
        get_parameter("servo_num", servo_num);

        RCLCPP_INFO(this->get_logger(), "Get JP200 Utils instance");
        utils = std::shared_ptr<jp200_driver::JP200Utils>(jp200_driver::JP200Utils::getJP200Utils(port_name, baud_rate));

        RCLCPP_INFO(this->get_logger(), "Init commands");
        for(int i = 0; i < servo_num; i++)
        {
            auto cmd = JP200Utils::JP200Cmd();
            commands_.push_back(cmd);
        }

        RCLCPP_INFO(this->get_logger(), "Initialize node");
        cmd_subscriber_ = this->create_subscription<jp200_msgs::msg::MultiJP200>(
        "/jp200_servo", 10, std::bind(&JP200Component::topic_callback, this, _1));
        read_timer_ = this->create_wall_timer(
            100ms, std::bind(&JP200Component::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Open Serial port");
        utils->open_port();

        if(utils->get_fd() < 0)
        {
            RCLCPP_ERROR(this->get_logger() , "Failed to open port");
            utils->close_port();
        }else{
            RCLCPP_INFO(this->get_logger(), "Serial port was connected <%d>", utils->get_fd());
            RCLCPP_INFO(this->get_logger(), "port:%s, baud rate:%d, enable servo response:%s", utils->get_port_name().c_str(), utils->get_baud_rate(), std::to_string(enable_servo_response).c_str());
        }

        sleep(1);
    }

    void JP200Component::topic_callback(const jp200_msgs::msg::MultiJP200 msg)
    {
        for(int i = 0; i < servo_num; i++)
        {
            commands_[i].id = msg.servos[i].id;
            commands_[i].control_mode = msg.servos[i].control_mode;

            commands_[i].angle.enable = msg.servos[i].angle_cmd.enable;
            commands_[i].angle.value = msg.servos[i].angle_cmd.value;

            commands_[i].velocity.enable = msg.servos[i].velocity_cmd.enable;
            commands_[i].velocity.value = msg.servos[i].velocity_cmd.value;
            commands_[i].current.enable = msg.servos[i].current_cmd.enable;
            commands_[i].current.value = msg.servos[i].current_cmd.value;

            commands_[i].pwm_enable = msg.servos[i].enable_pwm;
            commands_[i].pwm_rate = msg.servos[i].pwm_cmd;

            commands_[i].position_gain.enable = msg.servos[i].position_gain.enable;
            commands_[i].position_gain.p = msg.servos[i].position_gain.p;
            commands_[i].position_gain.i = msg.servos[i].position_gain.i;
            commands_[i].position_gain.d = msg.servos[i].position_gain.d;
            commands_[i].position_gain.f = msg.servos[i].position_gain.f;

            commands_[i].velocity_gain.enable = msg.servos[i].velocity_gain.enable;
            commands_[i].velocity_gain.p = msg.servos[i].velocity_gain.p;
            commands_[i].velocity_gain.i = msg.servos[i].velocity_gain.i;
            commands_[i].velocity_gain.d = msg.servos[i].velocity_gain.d;
            commands_[i].velocity_gain.f = msg.servos[i].velocity_gain.f;

            commands_[i].current_gain.enable = msg.servos[i].current_gain.enable;
            commands_[i].current_gain.p = msg.servos[i].current_gain.p;
            commands_[i].current_gain.i = msg.servos[i].current_gain.i;
            commands_[i].current_gain.d = msg.servos[i].current_gain.d;
            commands_[i].current_gain.f = msg.servos[i].current_gain.f;
        }

        utils->createJp200Cmd(commands_, enable_servo_response);
        int error = utils->write_serial();
        if(error > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Write %s", utils->get_tx_packet().c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write");
        }
    }

    void JP200Component::timer_callback()
    {
        if(enable_servo_response)
        {
            int error = utils->read_serial();
            if(error > 0)
            {
                RCLCPP_INFO(this->get_logger(), "Read %s", utils->get_rx_packet().c_str());
            }
        }
    }