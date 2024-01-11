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
            if(cmd.angle.get_state)
            {
                tx_packet_.push_back('C');
                tx_packet_.push_back('A');
            }
            if(cmd.velocity.get_state)
            {
                tx_packet_.push_back('C');
                tx_packet_.push_back('V');
            }
            if(cmd.current.get_state)
            {
                tx_packet_.push_back('C');
                tx_packet_.push_back('C');
            }
            if(cmd.get_pwm)
            {
                tx_packet_.push_back('C');
                tx_packet_.push_back('P');
            }
            if(cmd.get_mpu_temp)
            {
                tx_packet_.push_back('C');
                tx_packet_.push_back('T');
                tx_packet_.push_back('0');
            }
            if(cmd.get_amp_temp)
            {
                tx_packet_.push_back('C');
                tx_packet_.push_back('T');
                tx_packet_.push_back('1');
            }
            if(cmd.get_motor_temp)
            {
                tx_packet_.push_back('C');
                tx_packet_.push_back('T');
                tx_packet_.push_back('2');
            }
            if(cmd.get_voltage)
            {
                tx_packet_.push_back('C');
                tx_packet_.push_back('B');
            }
            if(cmd.get_status)
            {
                tx_packet_.push_back('S');
                tx_packet_.push_back('T');
            }
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
    }

    JP200Utils::Response JP200Utils::getResponse(std::string rx_packet, int motor_id)
    {
        std::string str_motor_id = "#" + std::to_string(motor_id);
        
        return JP200Utils::Response();
    }

    void JP200Utils::open_port()
    {
        fd_ =open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        struct termios conf_tio;
        tcgetattr(fd_,&conf_tio);
        speed_t BAUDRATE = get_baud_rate(baud_rate_);
        cfsetispeed(&conf_tio, BAUDRATE);
        cfsetospeed(&conf_tio, BAUDRATE);
        conf_tio.c_lflag &= ~(ECHO | ICANON);
        conf_tio.c_cc[VMIN]=0;
        conf_tio.c_cc[VTIME]=0;
        tcsetattr(fd_,TCSANOW,&conf_tio);
    }

    void JP200Utils::close_port()
    {
        close(fd_);
    }

    int JP200Utils::get_fd()
    {
        return fd_;
    }

    int JP200Utils::write_serial()
    {
        if(fd_ < 0)
        {
            return -1;
        }
        return write(fd_, tx_packet_.c_str(), tx_packet_.length());
    }

    std::string JP200Utils::read_serial()
    {
        if(fd_ < 0)
        {
            return READ_EMPTY;
        }
        char buf[100];
        ssize_t bytes_read = read(fd_, buf, sizeof(buf) - 1);
        if(bytes_read < 0)
        {
            return READ_ERROR;
        }
        else
        {
            buf[bytes_read] = '\0';
            std::string rx_packet_ = buf;
            return rx_packet_;
        }
    }

    speed_t JP200Utils::get_baud_rate(int baud_rate)
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
