#ifndef JP200_UTILS_HPP_
#define JP200_UTILS_HPP_

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

namespace jp200_demo_component
{
    class JP200Utils{
        public:
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
    };
}

#endif