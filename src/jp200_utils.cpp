#include "jp200_demo/jp200_utils.hpp"

namespace jp200_demo_component{
    std::string JP200Utils::createJp200Cmd(std::vector<JP200Cmd> cmds)
    {
        std::string packet = "<";

        for(JP200Cmd cmd :cmds)
        {
            packet += "#" + cmd.id;
            packet += CONTROL_MODE + cmd.control_mode;
            if(cmd.angle.enable)setTargetAngle(cmd, &packet);
            if(cmd.velocity.enable)setTargetVelocity(cmd, &packet);
            if(cmd.current.enable)setTargetCurrent(cmd, &packet);
            if(cmd.pwm_enable)setPWM(cmd, &packet);
            setGetStateEnable(cmd, &packet);
            if(cmd.position_gain.enable)setPositionGain(cmd, &packet);
            if(cmd.velocity.enable)setVelocityGain(cmd, &packet);
            if(cmd.current_gain.enable)setCurrentGain(cmd, &packet);
        }

        packet += ">";

        return packet;
    }

    void JP200Utils::setTargetAngle(JP200Cmd cmd, std::string *packet)
    {
        std::string header = cmd.id + "/";
        auto target_angle = std::to_string(cmd.angle.value * 100.0);
        *packet += TARGET_ANGLE + target_angle;
        if(cmd.angle.trajectory != 0)
        {
            auto target_tajectory = std::to_string(cmd.angle.trajectory);
            *packet += ";" + target_tajectory;
        }
        if(cmd.angle.transition_time != 0.0)
        {
            auto target_transition_time = std::to_string(cmd.angle.transition_time);
            *packet += ";" + target_transition_time;
        }
    }
    void JP200Utils::setTargetVelocity(JP200Cmd cmd, std::string *packet)
    {
        auto target_velocity = std::to_string(cmd.velocity.value * 1000.0);
        *packet += TARGET_VELOCITY + target_velocity;
        if(cmd.velocity.trajectory != 0)
        {
            auto target_trajectory = std::to_string(cmd.velocity.trajectory);
            *packet += ";" + target_trajectory;
        }
        if(cmd.velocity.transition_time != 0.0)
        {
            auto target_transition_time = std::to_string(cmd.velocity.transition_time);
            *packet += ";" + target_transition_time;
        }
    }
    void JP200Utils::setTargetCurrent(JP200Cmd cmd, std::string *packet)
    {
        auto target_current = std::to_string(cmd.current.value);
        *packet += TARGET_CURRENT + target_current;
        if(cmd.current.trajectory != 0)
        {
            auto target_trajectory = std::to_string(cmd.current.trajectory);
            *packet += ";" + target_trajectory;
        }
        if(cmd.current.transition_time != 0.0)
        {
            auto target_transition_time = std::to_string(cmd.current.transition_time);
            *packet += ";" + target_transition_time;
        }
    }
    void JP200Utils::setPWM(JP200Cmd cmd, std::string *packet)
    {
            auto target_pwm = std::to_string(cmd.pwm_rate);
            *packet += TARGET_PWM + target_pwm;
    }

    void JP200Utils::setGetStateEnable(JP200Cmd cmd, std::string *packet)
    {
        if(cmd.angle.get_state)*packet += GET_ANGLE;
        if(cmd.velocity.get_state)*packet += GET_VELOCITY;
        if(cmd.current.get_state)*packet += GET_CURRENT;
        if(cmd.get_pwm)*packet += GET_PWM;
        if(cmd.get_mpu_temp)*packet += GET_MPU_TEMP;
        if(cmd.get_amp_temp)*packet += GET_AMP_TEMP;
        if(cmd.get_motor_temp)*packet += GET_MOTOR_TEMP;
        if(cmd.get_voltage)*packet += GET_VOLTAGE;
        if(cmd.get_status)*packet += GET_STATUS;
    }

    void JP200Utils::setPositionGain(JP200Cmd cmd, std::string *packet)
    {
        auto gain = cmd.position_gain;
        auto p_str = std::to_string(gain.p);
        *packet += POSITION_GAIN + p_str;
        
        if(gain.i != 0.0)
        {
            auto i_str = std::to_string(gain.i);
            *packet += ";" + i_str;
        }
        if(gain.d != 0.0)
        {
            auto d_str = std::to_string(gain.d);
            *packet += ";" + d_str;
        }
        if(gain.f != 0.0)
        {
            auto f_str = std::to_string(gain.f);
            *packet += ";" + f_str;
        }
    }
    void JP200Utils::setVelocityGain(JP200Cmd cmd, std::string *packet)
    {
        auto gain = cmd.velocity_gain;
        auto p_str = std::to_string(gain.p);
        *packet += VELOCITY_GAIN + p_str;
        
        if(gain.i != 0.0)
        {
            auto i_str = std::to_string(gain.i);
            *packet += ";" + i_str;
        }
        if(gain.d != 0.0)
        {
            auto d_str = std::to_string(gain.d);
            *packet += ";" + d_str;
        }
        if(gain.f != 0.0)
        {
            auto f_str = std::to_string(gain.f);
            *packet += ";" + f_str;
        }
    }
    void JP200Utils::setCurrentGain(JP200Cmd cmd, std::string *packet)
    {
        auto gain = cmd.current_gain;
        auto p_str = std::to_string(gain.p);
        *packet += CURRENT_GAIN + p_str;
        
        if(gain.i != 0.0)
        {
            auto i_str = std::to_string(gain.i);
            *packet += ";" + i_str;
        }
        if(gain.d != 0.0)
        {
            auto d_str = std::to_string(gain.d);
            *packet += ";" + d_str;
        }
        if(gain.f != 0.0)
        {
            auto f_str = std::to_string(gain.f);
            *packet += ";" + f_str;
        }
    }
}