#ifndef COMPONENT_HPP_
#define COMPONENT_HPP_

#include "jp200_driver/utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <jp200_msgs/msg/jp200.hpp>

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <string>
#include <vector>

#define LATENCY_TIMER 16

namespace jp200_driver {

    class JP200Component : public rclcpp::Node{
        public:
            JP200Component(
                const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
            );

            void callback(const jp200_msgs::msg::JP200 msg);

            int open_port();
            void close_port();
            void read_serial();
            void write_serial();

        private:
            std::string port_name_;
            int baud_rate_;
            int fd_;
            int servo_num_;
            double tx_time_per_bytes;
            double packet_timeout_;
            double packet_start_time_;
            std::string tx_packet_;
            std::vector<uint8_t> rx_packet_;
            JP200Utils::JP200Cmd command_;
            JP200Utils utils;
            rclcpp::Subscription<jp200_msgs::msg::JP200>::SharedPtr cmd_subscriber_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_publisher_;
    };
}

#endif