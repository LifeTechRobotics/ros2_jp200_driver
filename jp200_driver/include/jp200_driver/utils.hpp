#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <vector>
#include <string>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>

namespace jp200_driver
{
    class JP200Utils{
        public:
            struct Gains
            {
                bool enable;
                float p;
                float i;
                float d;
                float f;
            };

            struct Cmd
            {
                bool enable;
                float value;
                int trajectory;
                float transition_time;
                bool get_state;
            };

            struct JP200Cmd
            {
                int id;
                bool enable_control_mode;
                uint8_t control_mode;
                Cmd angle;
                Cmd velocity;
                Cmd current;
                bool pwm_enable;
                double pwm_rate;
                bool get_pwm;
                bool get_mpu_temp;
                bool get_amp_temp;
                bool get_motor_temp;
                bool get_voltage;
                bool get_status;
                Gains position_gain;
                Gains velocity_gain;
                Gains current_gain; 

                bool error_checker;
            };

            struct Response
            {
                int id;
                bool control_mode;
                bool target_angle;
                bool target_velocity;
                bool target_current;
                bool target_pwm;
                bool target_position_gain;
                bool target_velocity_gain;
                bool target_current_gain;
                float angle_feedback;
                float velocity_feedback;
                float current_feedback;
                float pwm_feedback;
                float mpu_temp_feedback;
                float amp_temp_feedback;
                float motor_temp_feedback;
                float voltage_feedback;
                float status_feedback;
            };

            static JP200Utils *getJP200Utils(std::string port_name, int baud_rate);
            JP200Utils(std::string port_name, int baud_rate);
            
            void createJp200Cmd(std::vector<JP200Cmd> cmd, bool enable_response);
            std::vector<Response> getResponse(int servo_num);
            

            void open_port();
            void close_port();
            int read_serial();
            int write_serial();
            speed_t set_baud_rate(int baud_rate);

            std::string get_port_name();
            int get_fd();
            int get_baud_rate();
            std::string get_tx_packet();
            std::string get_rx_packet();

        private:
            std::string port_name_;
            int baud_rate_;
            int fd_;
            std::string _tx_packet_;
            std::string _rx_packet_;

    };
}

#endif