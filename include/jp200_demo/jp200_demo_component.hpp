#ifndef JP200_DEMO_COMPONENT_HPP_
#define JP200_DEMO_COMPONENT_HPP_

#include "jp200_demo/jp200_utils.hpp"

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

#define LATENCY_TIMER 16

namespace jp200_demo_component {

    class JP200DemoComp : public rclcpp::Node{
        public:
            JP200DemoComp(
                const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
            );

            bool openPort(int cflag_baud);
            void closePort();
            void clearPort();
            bool setBaudRate(const int baud_rate);
            int readPort(std::string *rx_packet);
            int writePort(std::string tx_packet);
            void setPacketTimeOut(uint16_t packet_langth);
            void setPacketTimeOut(double msec);
            bool isPacketTimeOut();
            double getCurrentTime();
            double getTimeSinceStart();
            int getCFlagBaud(int baud_rate);

        private:
            std::string port_name_;
            int baud_rate_;
            int fd_;
            int servo_num_;
            double tx_time_per_bytes;
            double packet_timeout_;
            double packet_start_time_;
            std::string tx_packet_;
            std::string rx_packet_;
            std::vector<JP200Utils::JP200Cmd> commands_;
            std::vector<rclcpp::Subscription<jp200_msgs::msg::JP200>::SharedPtr> cmd_subscribers_;
            std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> state_publishers_;
    };
}

#endif