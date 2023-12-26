#include "jp200_driver/utils.hpp"
#include <vector>
#include <cstdint>

namespace jp200_driver
{
    std::vector<uint8_t> JP200Utils::createJp200Cmd(JP200Cmd cmd)
    {
        auto id = serialize(cmd.id);
        std::vector<uint8_t> send = {'#'};
        for(size_t i = 0; i < id.size(); i++)
            {
                send.push_back(id[0]);
                id.erase(id.begin());
            }

        send.push_back('E');
        send.push_back('X');
        send.push_back('=');
        send.push_back(cmd.control_mode);

        if(cmd.angle.enable)
        {
            send.push_back('T');
            send.push_back('A');
            send.push_back('=');

            std::vector<uint8_t> data = serialize(cmd.angle.value);
            for(size_t i = 0; i < data.size(); i++)
            {
                send.push_back(data[0]);
                data.erase(data.begin());
            }
        }
        if(cmd.velocity.enable)
        {
            send.push_back('T');
            send.push_back('V');
            send.push_back('=');

            std::vector<uint8_t> data = serialize(cmd.velocity.value);
            for(size_t i = 0; i < data.size(); i++)
            {
                send.push_back(data[0]);
                data.erase(data.begin());
            }
        }
        if(cmd.current.enable)
        {
            send.push_back('T');
            send.push_back('C');
            send.push_back('=');

            std::vector<uint8_t> data = serialize(cmd.current.value);
            for(size_t i = 0; i < data.size(); i++)
            {
                send.push_back(data[0]);
                data.erase(data.begin());
            }
        }
        if(cmd.pwm_enable)
        {
            send.push_back('T');
            send.push_back('P');
            send.push_back('=');

            std::vector<uint8_t> data = serialize(cmd.pwm_rate);
            for(size_t i = 0; i < data.size(); i++)
            {
                send.push_back(data[0]);
                data.erase(data.begin());
            }
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
            
            auto p_data = serialize(cmd.position_gain.p);
            for(size_t i = 0; i < p_data.size(); i++)
            {
                send.push_back(p_data[0]);
                p_data.erase(p_data.begin());
            }
            if(cmd.position_gain.i != 0.0)
            {
                auto data = serialize(cmd.position_gain.i);
                for(size_t i = 0; i < data.size(); i++)
                {
                    send.push_back(data[0]);
                    data.erase(data.begin());
                }
                send.push_back(';');
            }
            if(cmd.position_gain.d != 0.0)
            {
                auto data = serialize(cmd.position_gain.d);
                for(size_t i = 0; i < data.size(); i++)
                {
                    send.push_back(data[0]);
                    data.erase(data.begin());
                }
                send.push_back(';');
            }
            if(cmd.position_gain.f != 0.0)
            {
                auto data = serialize(cmd.position_gain.f);
                for(size_t i = 0; i < data.size(); i++)
                {
                    send.push_back(data[0]);
                    data.erase(data.begin());
                }
            }
        }
        if(cmd.velocity_gain.enable)
        {
            send.push_back('S');
            send.push_back('G');
            send.push_back('1');
            
            auto p_data = serialize(cmd.velocity_gain.p);
            for(size_t i = 0; i < p_data.size(); i++)
            {
                send.push_back(p_data[0]);
                p_data.erase(p_data.begin());
            }
            if(cmd.velocity_gain.i != 0.0)
            {
                auto data = serialize(cmd.velocity_gain.i);
                for(size_t i = 0; i < data.size(); i++)
                {
                    send.push_back(data[0]);
                    data.erase(data.begin());
                }
                send.push_back(';');
            }
            if(cmd.velocity_gain.d != 0.0)
            {
                auto data = serialize(cmd.velocity_gain.d);
                for(size_t i = 0; i < data.size(); i++)
                {
                    send.push_back(data[0]);
                    data.erase(data.begin());
                }
                send.push_back(';');
            }
            if(cmd.velocity_gain.f != 0.0)
            {
                auto data = serialize(cmd.velocity_gain.f);
                for(size_t i = 0; i < data.size(); i++)
                {
                    send.push_back(data[0]);
                    data.erase(data.begin());
                }
            }
        }
        if(cmd.current_gain.enable)
        {
            send.push_back('S');
            send.push_back('G');
            send.push_back('2');
            
            auto p_data = serialize(cmd.current_gain.p);
            for(size_t i = 0; i < p_data.size(); i++)
            {
                send.push_back(p_data[0]);
                p_data.erase(p_data.begin());
            }
            if(cmd.current_gain.i != 0.0)
            {
                auto data = serialize(cmd.current_gain.i);
                for(size_t i = 0; i < data.size(); i++)
                {
                    send.push_back(data[0]);
                    data.erase(data.begin());
                }
                send.push_back(';');
            }
            if(cmd.current_gain.d != 0.0)
            {
                auto data = serialize(cmd.current_gain.d);
                for(size_t i = 0; i < data.size(); i++)
                {
                    send.push_back(data[0]);
                    data.erase(data.begin());
                }
                send.push_back(';');
            }
            if(cmd.current_gain.f != 0.0)
            {
                auto data = serialize(cmd.current_gain.f);
                for(size_t i = 0; i < data.size(); i++)
                {
                    send.push_back(data[0]);
                    data.erase(data.begin());
                }
            }
        }

        send.insert(send.begin(), '<');
        send.push_back('>');

        return send;
    }
}