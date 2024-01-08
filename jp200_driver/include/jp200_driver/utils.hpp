#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <vector>
#include <string>
#include <cstdint>
#include <cstring>
#include <memory>

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
            
            std::string createJp200Cmd(JP200Cmd cmd, bool enable_response);
    };
}

#endif