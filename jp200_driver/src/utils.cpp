#include "jp200_driver/utils.hpp"
#include <vector>
#include <cstdint>

namespace jp200_driver
{
    std::string JP200Utils::createJp200Cmd(JP200Cmd cmd)
    {
        std::string send = "#";
        send += std::to_string(cmd.id);

        send.push_back('E');
        send.push_back('X');
        send.push_back('=');
        send += std::to_string(cmd.control_mode);

        if(cmd.angle.enable)
        {
            send.push_back('T');
            send.push_back('A');
            send.push_back('=');

            auto target = (int)(cmd.angle.value);
            send += std::to_string(target);
        }
        if(cmd.velocity.enable)
        {
            send.push_back('T');
            send.push_back('V');
            send.push_back('=');

            auto target = (int)(cmd.velocity.value);
            send += std::to_string(target);
        }
        if(cmd.current.enable)
        {
            send.push_back('T');
            send.push_back('C');
            send.push_back('=');

            auto target = (int)(cmd.current.value);
            send += std::to_string(target);
        }
        if(cmd.pwm_enable)
        {
            send.push_back('T');
            send.push_back('P');
            send.push_back('=');

            auto target = (int)(cmd.pwm_rate);
            send += std::to_string(target);
        }
        if(cmd.angle.get_state)
        {
            send.push_back('C');
            send.push_back('A');
        }
        if(cmd.velocity.get_state)
        {
            send.push_back('C');
            send.push_back('V');
        }
        if(cmd.current.get_state)
        {
            send.push_back('C');
            send.push_back('C');
        }
        if(cmd.get_pwm)
        {
            send.push_back('C');
            send.push_back('P');
        }
        if(cmd.get_mpu_temp)
        {
            send.push_back('C');
            send.push_back('T');
            send.push_back('0');
        }
        if(cmd.get_amp_temp)
        {
            send.push_back('C');
            send.push_back('T');
            send.push_back('1');
        }
        if(cmd.get_motor_temp)
        {
            send.push_back('C');
            send.push_back('T');
            send.push_back('2');
        }
        if(cmd.get_voltage)
        {
            send.push_back('C');
            send.push_back('B');
        }
        if(cmd.get_status)
        {
            send.push_back('S');
            send.push_back('T');
        }
        if(cmd.position_gain.enable)
        {
            send.push_back('S');
            send.push_back('G');
            send.push_back('0');
            send.push_back('=');
            
            auto target = (int)(cmd.position_gain.p);
            send += std::to_string(target);
            if(cmd.position_gain.i != 0.0)
            {
                target = (int)(cmd.position_gain.i);
                send += std::to_string(target);
                send.push_back(';');
            }
            if(cmd.position_gain.d != 0.0)
            {
                target = (int)(cmd.position_gain.d);
                send += std::to_string(target);
                send.push_back(';');
            }
            if(cmd.position_gain.f != 0.0)
            {
                target = (int)(cmd.position_gain.f);
                send += std::to_string(target);
            }
        }
        if(cmd.velocity_gain.enable)
        {
            send.push_back('S');
            send.push_back('G');
            send.push_back('1');
            send.push_back('=');
            
            auto target = (int)(cmd.velocity_gain.p);
            send += std::to_string(target);
            if(cmd.velocity_gain.i != 0.0)
            {
                target = (int)(cmd.velocity_gain.i);
                send += std::to_string(target);
                send.push_back(';');
            }
            if(cmd.velocity_gain.d != 0.0)
            {
                target = (int)(cmd.velocity_gain.d);
                send += std::to_string(target);
                send.push_back(';');
            }
            if(cmd.velocity_gain.f != 0.0)
            {
                target = (int)(cmd.velocity_gain.f);
                send += std::to_string(target);
            }
        }
        if(cmd.current_gain.enable)
        {
            send.push_back('S');
            send.push_back('G');
            send.push_back('2');
            send.push_back('=');
            
            auto target = (int)(cmd.current_gain.p);
            send += std::to_string(target);
            if(cmd.current_gain.i != 0.0)
            {
                target = (int)(cmd.current_gain.i);
                send += std::to_string(target);
                send.push_back(';');
            }
            if(cmd.current_gain.d != 0.0)
            {
                target = (int)(cmd.current_gain.d);
                send += std::to_string(target);
                send.push_back(';');
            }
            if(cmd.current_gain.f != 0.0)
            {
                target = (int)(cmd.current_gain.f);
                send += std::to_string(target);
            }
        }

        send.insert(send.begin(), '<');
        send.push_back('>');

        return send;
    }
}