#include "jp200_demo/jp200_utils.hpp"

namespace jp200_demo_component{
    std::string JP200Utils::createJp200Cmd(std::vector<JP200Cmd> cmds)
    {
        std::string packet = "<";

        for(JP200Cmd cmd :cmds)
        {
            packet += "#" + cmd.id;
            packet += "EX=" + cmd.control_mode;
            setTargetAngle(cmd, &packet);
            setTargetVelocity(cmd, &packet);
            setTargetCurrent(cmd, &packet);
            setPWM(cmd, &packet);
            setGetStateEnable(cmd, &packet);
            setPositionGain(cmd, &packet);
        }

        return packet;
    }

    void JP200Utils::setTargetAngle(JP200Cmd cmd, std::string *packet)
    {
        auto target_angle = std::to_string(cmd.angle.value * 100.0);
        *packet += "TA=" + target_angle;
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
        *packet += "TV=" + target_velocity;
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
        *packet += "TC=" + target_current;
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
            *packet += "TP=" + target_pwm;
    }

    void JP200Utils::setGetStateEnable(JP200Cmd cmd, std::string *packet)
    {
        if(cmd.angle.get_state)*packet += "CA";
        if(cmd.velocity.get_state)*packet += "CV";
        if(cmd.current.get_state)*packet += "CC";
        if(cmd.get_pwm)*packet += "CP";
        if(cmd.get_mpu_temp)*packet += "CT0";
        if(cmd.get_amp_temp)*packet += "CT1";
        if(cmd.get_motor_temp)*packet += "CT2";
        if(cmd.get_voltage)*packet += "CB";
        if(cmd.get_status)*packet += "ST";
    }

    void JP200Utils::setPositionGain(Gains gain, std::string *packet)
    {
        auto p_str = std::to_string(gain.p);
        *packet += "SG0=" + p_str;
        
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
    void JP200Utils::setVelocityGain(Gains gain, std::string *packet)
    {
        auto p_str = std::to_string(gain.p);
        *packet += "SG1=" + p_str;
        
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
    void JP200Utils::setCurrentGain(Gains gain, std::string *packet)
    {
        auto p_str = std::to_string(gain.p);
        *packet += "SG2=" + p_str;
        
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