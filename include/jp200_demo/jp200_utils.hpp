#ifndef JP200_UTILS_HPP_
#define JP200_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float32.hpp>

#include <vector>
#include <string>

#define CONTROL_MODE "EX="
#define TARGET_ANGLE "TA="
#define TARGET_VELOCITY "TV="
#define TARGET_CURRENT "TC="
#define TARGET_PWM "TP="
#define GET_ANGLE "CA"
#define GET_VELOCITY "CV"
#define GET_CURRENT "CC"
#define GET_PWM "CP"
#define GET_MPU_TEMP "CT0"
#define GET_AMP_TEMP "CT1"
#define GET_MOTOR_TEMP "CT2"
#define GET_VOLTAGE "CB"
#define GET_STATUS "ST"
#define POSITION_GAIN "SG0="
#define VELOCITY_GAIN "SG1="
#define CURRENT_GAIN "SG2="

namespace jp200_demo_component
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
                const uint8_t id;
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
            };
            
            std::string createJp200Cmd(std::vector<JP200Cmd> cmds);
            void setTargetAngle(JP200Cmd cmd, std::string *packet);
            void setTargetVelocity(JP200Cmd cmd, std::string *packet);
            void setTargetCurrent(JP200Cmd cmd, std::string *packet);
            void setPWM(JP200Cmd cmd, std::string *packet);
            void setGetStateEnable(JP200Cmd cmd, std::string *packet);
            void setPositionGain(JP200Cmd cmd, std::string *packet);
            void setVelocityGain(JP200Cmd cmd, std::string *packet);
            void setCurrentGain(JP200Cmd cmd, std::string *packet);
            bool getCmdResult(std::string rx_packet, std::string cmd_name);
            double getState(std::string rx_packet, std::string cmd_name);

    };
}

#endif