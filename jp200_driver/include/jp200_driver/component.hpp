#ifndef COMPONENT_HPP_
#define COMPONENT_HPP_

#include "jp200_driver/utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <jp200_msgs/msg/jp200.hpp>
#include <jp200_msgs/msg/response.hpp>


namespace jp200_driver {

    class JP200Component : public rclcpp::Node{
        public:
            JP200Component(
                const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
            );

            void add_subscriber();
            void add_publisher();
            void callback(const jp200_msgs::msg::JP200 msg);

            int open_port();
            void close_port();
            int read_serial();
            int write_serial();
            speed_t get_baud_rate();

        private:
            std::string port_name_;
            int baud_rate_;
            int servo_num;
            int fd_;
            bool enable_servo_response;
            std::string tx_packet_;
            std::string rx_packet_;
            std::vector<JP200Utils::JP200Cmd> commands_;
            JP200Utils utils;

            std::vector<rclcpp::Subscription<jp200_msgs::msg::JP200>::SharedPtr> cmd_subscribers_;
            std::vector<rclcpp::Publisher<jp200_msgs::msg::Response>::SharedPtr> state_publishers_;
            rclcpp::TimerBase::SharedPtr read_timer_, write_timer_;
    };
}

#endif