#include "jp200_driver/utils.hpp"
#include <vector>
#include <cstdint>

using namespace jp200_driver;

    JP200Utils *JP200Utils::getJP200Utils(std::string port_name, int baud_rate)
    {
        return (JP200Utils *)(new JP200Utils(port_name, baud_rate));
    }

    JP200Utils::JP200Utils(std::string port_name, int baud_rate)
    {
        port_name_ = port_name;
        baud_rate_ = baud_rate;
    }

    void JP200Utils::createJp200Cmd(std::vector<JP200Cmd> cmds, bool enable_response)
    {
        std::string tx_packet_ = "";
        for(auto cmd : cmds)
        {
            tx_packet_.push_back('#');
            tx_packet_ += std::to_string(cmd.id);

            tx_packet_.push_back('E');
            tx_packet_.push_back('X');
            tx_packet_.push_back('=');
            tx_packet_ += std::to_string(cmd.control_mode);

            if(cmd.angle.enable)
            {
                tx_packet_.push_back('T');
                tx_packet_.push_back('A');
                tx_packet_.push_back('=');

                auto target = (int)(cmd.angle.value*100);
                tx_packet_ += std::to_string(target);
            }
            if(cmd.velocity.enable)
            {
                tx_packet_.push_back('T');
                tx_packet_.push_back('V');
                tx_packet_.push_back('=');

                auto target = (int)(cmd.velocity.value);
                tx_packet_ += std::to_string(target);
            }
            if(cmd.current.enable)
            {
                tx_packet_.push_back('T');
                tx_packet_.push_back('C');
                tx_packet_.push_back('=');

                auto target = (int)(cmd.current.value);
                tx_packet_ += std::to_string(target);
            }
            if(cmd.pwm_enable)
            {
                tx_packet_.push_back('T');
                tx_packet_.push_back('P');
                tx_packet_.push_back('=');

                auto target = (int)(cmd.pwm_rate*100);
                tx_packet_ += std::to_string(target);
            }
                
            tx_packet_.push_back('C');
            tx_packet_.push_back('A');
            
            tx_packet_.push_back('C');
            tx_packet_.push_back('V');
            
            tx_packet_.push_back('C');
            tx_packet_.push_back('C');

            tx_packet_.push_back('C');
            tx_packet_.push_back('P');
            
            tx_packet_.push_back('C');
            tx_packet_.push_back('T');
            tx_packet_.push_back('0');

            tx_packet_.push_back('C');
            tx_packet_.push_back('T');
            tx_packet_.push_back('1');
            
            tx_packet_.push_back('C');
            tx_packet_.push_back('T');
            tx_packet_.push_back('2');
            
            
            
            tx_packet_.push_back('C');
            tx_packet_.push_back('B');
            
            
            
            tx_packet_.push_back('S');
            tx_packet_.push_back('T');
            
            if(cmd.position_gain.enable)
            {
                tx_packet_.push_back('S');
                tx_packet_.push_back('G');
                tx_packet_.push_back('0');
                tx_packet_.push_back('=');
                
                auto target = (int)(cmd.position_gain.p);
                tx_packet_ += std::to_string(target);
                if(cmd.position_gain.i != 0.0)
                {
                    target = (int)(cmd.position_gain.i);
                    tx_packet_ += std::to_string(target);
                    tx_packet_.push_back(';');
                }
                if(cmd.position_gain.d != 0.0)
                {
                    target = (int)(cmd.position_gain.d);
                    tx_packet_ += std::to_string(target);
                    tx_packet_.push_back(';');
                }
                if(cmd.position_gain.f != 0.0)
                {
                    target = (int)(cmd.position_gain.f);
                    tx_packet_ += std::to_string(target);
                }
            }
            if(cmd.velocity_gain.enable)
            {
                tx_packet_.push_back('S');
                tx_packet_.push_back('G');
                tx_packet_.push_back('1');
                tx_packet_.push_back('=');
                
                auto target = (int)(cmd.velocity_gain.p);
                tx_packet_ += std::to_string(target);
                if(cmd.velocity_gain.i != 0.0)
                {
                    target = (int)(cmd.velocity_gain.i);
                    tx_packet_ += std::to_string(target);
                    tx_packet_.push_back(';');
                }
                if(cmd.velocity_gain.d != 0.0)
                {
                    target = (int)(cmd.velocity_gain.d);
                    tx_packet_ += std::to_string(target);
                    tx_packet_.push_back(';');
                }
                if(cmd.velocity_gain.f != 0.0)
                {
                    target = (int)(cmd.velocity_gain.f);
                    tx_packet_ += std::to_string(target);
                }
            }
            if(cmd.current_gain.enable)
            {
                tx_packet_.push_back('S');
                tx_packet_.push_back('G');
                tx_packet_.push_back('2');
                tx_packet_.push_back('=');
                
                auto target = (int)(cmd.current_gain.p);
                tx_packet_ += std::to_string(target);
                if(cmd.current_gain.i != 0.0)
                {
                    target = (int)(cmd.current_gain.i);
                    tx_packet_ += std::to_string(target);
                    tx_packet_.push_back(';');
                }
                if(cmd.current_gain.d != 0.0)
                {
                    target = (int)(cmd.current_gain.d);
                    tx_packet_ += std::to_string(target);
                    tx_packet_.push_back(';');
                }
                if(cmd.current_gain.f != 0.0)
                {
                    target = (int)(cmd.current_gain.f);
                    tx_packet_ += std::to_string(target);
                }
            }
        }

        if(enable_response)
        {
            tx_packet_.insert(tx_packet_.begin(), '<');
            tx_packet_.push_back('>');
        }
        else
        {
            tx_packet_.insert(tx_packet_.begin(), '[');
            tx_packet_.push_back(']');
        }

        _tx_packet_ = tx_packet_;
    }

    std::vector<JP200Utils::Response> JP200Utils::getResponse(int servo_num)
    {
        std::vector<JP200Utils::Response> resps;
        std::string left_over_packet = "";
        std::string one_motor_packet = "";
        int start_index = 0;
        int end_index = 0;

        left_over_packet = _rx_packet_;

        for(int i = 0; i < servo_num; i++)
        {
            auto resp = JP200Utils::Response();
            start_index = left_over_packet.find_first_of('(');
            end_index = left_over_packet.find_first_of(')');

            one_motor_packet = left_over_packet.substr(start_index, end_index);
            left_over_packet = left_over_packet.substr(end_index);

            // id
            int id_index = one_motor_packet.find("#");
            int id_end_index = one_motor_packet.find("EX");
            int id_range = id_index - id_end_index;
            std::string id_str = one_motor_packet.substr(id_index, id_range);
            resp.id = atoi(id_str.c_str());

            // EX(control mode)
            int ex_index = one_motor_packet.find("EX=");
            std::string ex_str = one_motor_packet.substr(ex_index + 2, 2);
            if(ex_str == "OK")
            {
                resp.control_mode = true;
            }
            else if(ex_str == "NG")
            {
                resp.control_mode = false;
            }

            // TA
            int _index = one_motor_packet.find("TA=");
            if(_index != std::string::npos)
            {
                std::string ta_str = one_motor_packet.substr(_index + 2, 2);
                if(ta_str == "OK")
                {
                    resp.target_angle = true;
                }
                else if(ta_str == "NG")
                {
                    resp.target_angle = false;
                }
            }

            // TA
            int _index = one_motor_packet.find("TA=");
            if(_index != std::string::npos)
            {
                std::string ta_str = one_motor_packet.substr(_index + 2, 2);
                if(ta_str == "OK")
                {
                    resp.target_angle = true;
                }
                else if(ta_str == "NG")
                {
                    resp.target_angle = false;
                }
            }


            resps.push_back(resp);
        }

        return resps;
    }

    void JP200Utils::open_port()
    {
        fd_ =open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        struct termios conf_tio;
        tcgetattr(fd_,&conf_tio);
        speed_t BAUDRATE = set_baud_rate(baud_rate_);
        cfsetispeed(&conf_tio, BAUDRATE);
        cfsetospeed(&conf_tio, BAUDRATE);
        conf_tio.c_lflag &= ~(ECHO | ICANON);
        conf_tio.c_cc[VMIN]=0;
        conf_tio.c_cc[VTIME]=10;
        conf_tio.c_cflag &= ~PARENB;
        conf_tio.c_cflag &= ~CSTOPB;
        conf_tio.c_cflag |= CS8;
        tcsetattr(fd_,TCSANOW,&conf_tio);
    }

    void JP200Utils::close_port()
    {
        close(fd_);
    }

    int JP200Utils::write_serial()
    {
        if(fd_ < 0)
        {
            return -1;
        }
        return write(fd_, _tx_packet_.c_str(), _tx_packet_.size());
    }

    int JP200Utils::read_serial()
    {
        if(fd_ < 0)
        {
            return -1;
        }
        char buf[100];
        ssize_t bytes_read = read(fd_, buf, sizeof(buf));
        if(bytes_read < 0)
        {
            return -1;
        }
        else
        {
            buf[bytes_read] = '\0';
            _rx_packet_ = buf;
            return 1;
        }
    }

    speed_t JP200Utils::set_baud_rate(int baud_rate)
    {
        switch(baud_rate)
        {
            case 9600:
            return B9600;
            case 19200:
            return B19200;
            case 38400:
            return B38400;
            case 57600:
            return B57600;
            case 115200:
            return B115200;
            case 230400:
            return B230400;
            case 460800:
            return B460800;
            case 500000:
            return B500000;
            case 576000:
            return B576000;
            case 921600:
            return B921600;
            case 1000000:
            return B1000000;
            case 1152000:
              return B1152000;
            case 1500000:
            return B1500000;
            case 2000000:
            return B2000000;
            case 2500000:
            return B2500000;
            case 3000000:
            return B3000000;
            case 3500000:
            return B3500000;
            case 4000000:
            return B4000000;
            default:
            return -1;
        }
    }

    int JP200Utils::get_fd()
    {
        return fd_;
    }

    int JP200Utils::get_baud_rate()
    {
        return baud_rate_;
    }

    std::string JP200Utils::get_port_name()
    {
        return port_name_;
    }

    std::string JP200Utils::get_tx_packet()
    {
        return _tx_packet_;
    }

    std::string JP200Utils::get_rx_packet()
    {
        return _rx_packet_;
    }
