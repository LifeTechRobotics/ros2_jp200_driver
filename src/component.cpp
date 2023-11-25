#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include <jp200_demo/jp200_demo_component.hpp>

#include <string>
#include <termios.h>
#include <fcntl.h>

namespace jp200_demo_component{

    JP200DemoComp::JP200DemoComp(const rclcpp::NodeOptions& options)
    :Node("jo200_demo", options)
    {
        RCLCPP_INFO(this->get_logger(), "Start and Get Parameter");
        declare_parameter("serial_port", "/dev/ttyACM0");
        declare_parameter("baud_rate", 115200);
        declare_parameter("servo_num", 1);
        get_parameter("serial_port", port_name_);
        get_parameter("baud_rate", baud_rate_);
        get_parameter("servo_num", servo_num_);

        // get ros2 parameter from launch or yaml
        for(int i = 0; i < servo_num_; i++)
        {
            commands_.push_back(get_jp200_parameter(i));
        }

        // add subscriber
        for(auto cmd:commands_)
        {
            if(cmd.angle.enable)cmd_subscribers_.push_back(create_subscription<std_msgs::msg::Float32>(cmd.id + "/target_angle", 1, sub_cmd_callback));
            if(cmd.velocity.enable)cmd_subscribers_.push_back(create_subscription<std_msgs::msg::Float32>(cmd.id + "/target_velocity", 1, sub_cmd_callback));
            if(cmd.current.enable)cmd_subscribers_.push_back(create_subscription<std_msgs::msg::Float32>(cmd.id + "/target_current", 1, sub_cmd_callback));
        }

        RCLCPP_INFO(this->get_logger(), "Open Serial port");
        if(!openPort(baud_rate_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
        }


    }

    JP200Utils::JP200Cmd JP200DemoComp::get_jp200_parameter(int num)
    {
        JP200Utils::JP200Cmd cmd = {};
        std::string header = num + "/";
        declare_parameter(header +"id", 0);
        declare_parameter(header +"control_mode", 1);
        declare_parameter(header +"enable/target_angle", false);
        declare_parameter(header +"enable/target_velocity", false);
        declare_parameter(header +"enable/target_current", false);
        declare_parameter(header +"enable/target_pwm", false);
        declare_parameter(header +"enable/get_angle", false);
        declare_parameter(header +"enable/get_velocity", false);
        declare_parameter(header +"enable/get_current", false);
        declare_parameter(header +"enable/get_pwm", false);
        declare_parameter(header +"enable/get_mpu", false);
        declare_parameter(header +"enable/get_amp", false);
        declare_parameter(header +"enable/get_motor", false);
        declare_parameter(header +"enable/get_voltage", false);
        declare_parameter(header +"enable/get_status", false);
        declare_parameter(header +"enable/gain_position", false);
        declare_parameter(header +"enable/gain_velocity", false);
        declare_parameter(header +"enable/gain_current", false);

        get_parameter(header +"id", cmd.id);
        get_parameter(header +"control_mode", cmd.control_mode);
        get_parameter(header +"enable/target_angle", cmd.angle.enable);
        get_parameter(header +"enable/target_velocity", cmd.velocity.enable);
        get_parameter(header +"enable/target_current", cmd.current.enable);
        get_parameter(header +"enable/target_pwm", cmd.pwm_enable);
        get_parameter(header +"enable/get_angle", cmd.angle.get_state);
        get_parameter(header +"enable/get_velocity", cmd.velocity.get_state);
        get_parameter(header +"enable/get_current", cmd.current.get_state);
        get_parameter(header +"enable/get_pwm", cmd.get_pwm);
        get_parameter(header +"enable/get_mpu", cmd.get_mpu_temp);
        get_parameter(header +"enable/get_amp", cmd.get_amp_temp);
        get_parameter(header +"enable/get_motor", cmd.get_motor_temp);
        get_parameter(header +"enable/get_voltage", cmd.get_voltage);
        get_parameter(header +"enable/get_status", cmd.get_status);
        get_parameter(header +"enable/gain_position", cmd.position_gain.enable);
        get_parameter(header +"enable/gain_velocity", cmd.velocity_gain.enable);
        get_parameter(header +"enable/gain_current", cmd.current_gain.enable);
        
    }

    void JP200DemoComp::sub_cmd_callback(
        const std_msgs::msg::Float32::SharedPtr msg,
        float *get_value
    )
    {
        *get_value = msg->data;
    }


    bool JP200DemoComp::openPort(int cflag_baud)
    {
        struct termios tio;

        fd_ = open(port_name_.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);

        if(fd_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("jp200_demo_serial"), "Failed to open serial port!");
            return false;
        }

        bzero(&tio, sizeof(tio));

        tio.c_cflag = cflag_baud |CS8 | CLOCAL | CREAD;
        tio.c_iflag = IGNPAR;
        tio.c_oflag      = 0;
        tio.c_lflag      = 0;
        tio.c_cc[VTIME]  = 0;
        tio.c_cc[VMIN]   = 0;

        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSANOW, &tio);

        tx_time_per_bytes = (1000.0 / (double)baud_rate_) * 10.0;

        return true;
    }

    void JP200DemoComp::closePort()
    {
        if(fd_ != -1)close(fd_);
        fd_ = -1;
    }
    void JP200DemoComp::clearPort()
    {
        tcflush(fd_, TCIFLUSH);
    }

    int JP200DemoComp::readPort(std::string *rx_packet)
    {
        uint8_t checksum = 0;
        uint8_t rx_length = 0;
        uint8_t wait_length = 6;
    }

    int JP200DemoComp::writePort(std::string tx_packet)
    {
        const char *send_msg = tx_packet.c_str();

        int wrriten_bytes = write(fd_, send_msg, strlen(send_msg));

        return wrriten_bytes;
    }

    void JP200DemoComp::setPacketTimeOut(uint16_t packet_length)
    {
        packet_start_time_ = getCurrentTime();
        packet_timeout_ = (tx_time_per_bytes * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
    }

    void JP200DemoComp::setPacketTimeOut(double msec)
    {
        packet_start_time_ = getCurrentTime();
        packet_timeout_ = msec;
    }

    bool JP200DemoComp::isPacketTimeOut()
    {
        if(getTimeSinceStart() > packet_timeout_)
        {
            packet_timeout_ = 0;
            return true;
        }
        return false;
    }

    // return milli sec
    double JP200DemoComp::getCurrentTime()
    {
        struct timespec tv;
        clock_gettime(CLOCK_REALTIME, &tv);
        return ((double)tv.tv_sec * 1000.0 + (double)tv.tv_nsec * 0.001 * 0.001);
    }

    double JP200DemoComp::getTimeSinceStart()
    {
        double time;
        time = getCurrentTime() - packet_start_time_;
        if(time < 0.0)
        {
            packet_start_time_ = getCurrentTime();
        }

        return time;
    }

    bool JP200DemoComp::setBaudRate(const int baud_rate)
    {
        int baud = getCFlagBaud(baud_rate);

        clearPort();

        if(baud <= 0)
        {
            openPort(B38400);
            baud_rate_ = baud_rate;
        }
        else
        {
            baud_rate_ = baud_rate;
            return openPort(baud);
        }
    }

    int JP200DemoComp::getCFlagBaud(int baudrate)
    {
        switch(baudrate)
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