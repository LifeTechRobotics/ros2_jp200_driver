#ifndef COMPONENT_HPP_
#define COMPONENT_HPP_

#include "jp200_driver/utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <jp200_msgs/msg/jp200_multi_array.hpp>
#include <jp200_msgs/msg/jp200_responses.hpp>
#include <jp200_msgs/msg/response.hpp>


namespace jp200_driver {

    class JP200Component : public rclcpp::Node{
        public:
            explicit JP200Component(
                const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
            );
            void topic_callback(const jp200_msgs::msg::JP200MultiArray msg);
            void timer_callback();

        private:
            std::string port_name;
            int baud_rate;
            int servo_num;
            bool enable_servo_response;
            std::vector<JP200Utils::JP200Cmd> commands_;
            std::vector<JP200Utils::Response> resps_;
            std::shared_ptr<jp200_driver::JP200Utils> utils;

            rclcpp::Subscription<jp200_msgs::msg::JP200MultiArray>::SharedPtr cmd_subscriber_;
            rclcpp::Publisher<jp200_msgs::msg::JP200Responses>::SharedPtr resp_publisher_;
            rclcpp::TimerBase::SharedPtr read_timer_;
    };
}

#endif