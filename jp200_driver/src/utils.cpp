#include "jp200_driver/utils.hpp"
#include <vector>
#include <cstdint>

namespace jp200_driver
{
    JP200Utils::JP200Utils()
    {

    }

    std::string JP200Utils::createJp200Cmd(std::vector<JP200Cmd> cmds, bool enable_response)
    {
        std::string send;
        for(auto cmd : cmds)
        {
            send.push_back('#');
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

                auto target = (int)(cmd.angle.value*100);
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

                auto target = (int)(cmd.pwm_rate*100);
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
        }
        

        if(enable_response)
        {
            send.insert(send.begin(), '<');
            send.push_back('>');
        }
        else
        {
            send.insert(send.begin(), '[');
            send.push_back(']');
        }

        return send;
    }

    int JP200Utils::open_port(std::string port_name, int baud_rate)
    {
        int fd=open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        fcntl(fd, F_SETFL,0);
        struct termios conf_tio;
        tcgetattr(fd,&conf_tio);
        speed_t BAUDRATE = get_baud_rate(baud_rate);
        cfsetispeed(&conf_tio, BAUDRATE);
        cfsetospeed(&conf_tio, BAUDRATE);
        conf_tio.c_lflag &= ~(ECHO | ICANON);
        conf_tio.c_cc[VMIN]=0;
        conf_tio.c_cc[VTIME]=0;
        tcsetattr(fd,TCSANOW,&conf_tio);
        return fd;
    }

    void JP200Utils::close_port(int fd)
    {
        close(fd);
    }

    int JP200Utils::write_serial(int fd, std::string tx_packet)
    {
        if(fd < 0)
        {
            return -1;
        }
        const char *packet = tx_packet.c_str();
        return write(fd, packet, strlen(packet));
    }

    std::string JP200Utils::read_serial(int fd)
    {
        if(fd < 0)
        {
            return "";
        }
        char buf[100];
        ssize_t bytes_read = read(fd, buf, sizeof(buf) - 1);
        if(bytes_read < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read port");
            return bytes_read;
        }
        else
        {
            buf[bytes_read] = '\0';
            rx_packet_ = buf;
            RCLCPP_INFO(this->get_logger(), "read %s from {%d}", rx_packet_.c_str(), fd_);
            return bytes_read;
        }
    }

    speed_t JP200Utils::get_baud_rate()
    {
        switch(baud_rate_)
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
}