#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <vector>
#include <string>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <std_msgs/Int32MultiArray.h>

using namespace std;

class RS485Controller
{
private:
    int serial_port;

    // 串口初始化
    bool setup_serial(const std::string &port, int baudrate)
    {
        serial_port = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (serial_port < 0)
        {
            ROS_ERROR("Error opening serial port %s", port.c_str());
            return false;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));

        if (tcgetattr(serial_port, &tty) != 0)
        {
            ROS_ERROR("Error getting terminal attributes");
            return false;
        }

        // 设置波特率
        cfsetospeed(&tty, baudrate);
        cfsetispeed(&tty, baudrate);

        // 8位数据，无校验，1停止位
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;       // 禁用硬件流控
        tty.c_cflag |= CREAD | CLOCAL; // 启用接收，忽略调制解调器控制线

        // 非规范模式
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        // 原始输出
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        // 设置超时：1秒
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10; // 1秒超时

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
        {
            ROS_ERROR("Error setting terminal attributes");
            return false;
        }

        return true;
    }

public:
    RS485Controller(const std::string &port = "/dev/ttyUSB0", int baudrate = B115200)
    {
        if (!setup_serial(port, baudrate))
        {
            throw std::runtime_error("Failed to initialize serial port");
        }
        ROS_INFO("RS485 controller initialized on %s", port.c_str());
    }

    ~RS485Controller()
    {
        if (serial_port >= 0)
        {
            close(serial_port);
            ROS_INFO("RS485 port closed");
        }
    }

    // 构建18字节命令帧
    std::vector<uint8_t> build_command(uint8_t address, uint8_t mode,
                                       const std::vector<uint8_t> &data)
    {

        if(address==0xA3)
        {
            
        }
      
        std::vector<uint8_t> command = {
            0xFF, 0xFF,       // 帧头
            address, mode,    // 地址和模式
            data[0], 0xff,    // id
            0x64,             // 加速度
            data[2], 0x00,    // 位置
            0x00, 0x00,       // 时间
            data[3], data[4], // 速度
            0xAC, 0x02,       // 转矩
            0x00,
            0xFF, 0xFF // 帧尾
        };


        return command;
    }

    // 发送命令并接收响应
    bool send_command(const std::vector<uint8_t> &command, std::vector<uint8_t> &response)
    {
        // 发送完整18字节帧
        ssize_t written = write(serial_port, command.data(), command.size());
        if (written != command.size())
        {
            ROS_ERROR("Failed to write full command");
            return false;
        }

        // 等待数据发送完成
        tcdrain(serial_port);

        // 读取响应（假设也是18字节）
        response.resize(18);
        // ssize_t read_bytes = read(serial_port, response.data(), response.size());
        ssize_t read_bytes = 0;

        if (read_bytes == 18)
        {
            ROS_WARN("Incomplete response: %zd bytes", read_bytes);
            return false;
        }

        ROS_DEBUG("Command sent, response received");
        return true;
    }

    // 设置舵机位置（实用函数）
    bool set_servo_position(uint8_t Command, const std_msgs::Int32MultiArray::ConstPtr &msg, uint16_t speed = 950)
    {

        // 构建完整命令
        if (Command == 1)
        {

            std::vector<uint8_t> cmd = {
                0xFF, 0xFF, // 帧头
                0x03, 0xa3, // 地址和模式
                0x5a, 0x00, // 数据1 + 填充
                0x5a, 0x00, // 数据2 + 填充
                0x5a, 0x00, // 数据3 + 填充
                0x5a, 0x00, // 数据4 + 填充
                0x00, 0x00, // 数据5 + 填充
                0x00, 0x00, // 数据6 + 填充
                0xFF, 0xFF  // 帧尾
            };

            // 发送命令
            std::vector<uint8_t> response;
            bool success = send_command(cmd, response);

            
            usleep(2500000);

            std::vector<uint8_t> cmd2 = {
                0xFF, 0xFF, // 帧头
                0x03, 0xa3, // 地址和模式
                0x5a, 0x00, // 数据1 + 填充
                0x5a, 0x00, // 数据2 + 填充
                0x5a, 0x00, // 数据3 + 填充
                0x5a, 0x00, // 数据4 + 填充
                0x00, 0x00, // 数据5 + 填充
                0x5a, 0x00, // 数据6 + 填充
                0xFF, 0xFF  // 帧尾
            };

            // 发送命令

            success = send_command(cmd2, response);

            

            return success;
        }

        else if (Command == 2)
        {


            std::vector<uint8_t> cmd1 = {
                0xFF, 0xFF, // 帧头
                0x03, 0xa3, // 地址和模式
                0x5a, 0x00, // 数据1 + 填充
                0x5a, 0x00, // 数据2 + 填充
                0x5a, 0x00, // 数据3 + 填充
                0x5a, 0x00, // 数据4 + 填充
                0x00, 0x00, // 数据5 + 填充
                0x00, 0x00, // 数据6 + 填充
                0xFF, 0xFF  // 帧尾
            };

           

            // 发送命令
            std::vector<uint8_t> response1;
            bool success1 = send_command(cmd1, response1);


             usleep(2500000);
            // sleep(10);


            std::vector<uint8_t> cmd = {
                0xFF, 0xFF, // 帧头
                0x03, 0xa3, // 地址和模式
                0x00, 0x00, // 数据1 + 填充
                0x00, 0x00, // 数据2 + 填充
                0x00, 0x00, // 数据3 + 填充
                0x00, 0x00, // 数据4 + 填充
                0x00, 0x00, // 数据5 + 填充
                0x00, 0x00, // 数据6 + 填充
                0xFF, 0xFF  // 帧尾
            };

            // 发送命令
            std::vector<uint8_t> response;
            bool success = send_command(cmd, response);

            
        }
        else if (Command == 3)
        {
            speed = 1000;
            
            uint8_t speedlow_byte = speed & 0xFF; // 获取低字节
            uint8_t speedhigh_byte = (speed >> 8) & 0xFF;

            // 构建数据字段
            for (int i = 1; i < 7; i++)
            {
                
                int position1 = static_cast<uint8_t>(msg->data[i]);

                uint8_t position=static_cast<uint8_t>(position1) ;
              //  cout << "msmsg->data[" << i << "]" << "= " << position << " "<< position1 <<endl;
                
                std::vector<uint8_t> data = {
                    uint8_t(i), // ID
                    0x01,     // 写操作
                    position, // 位置
                    speedlow_byte,
                    speedhigh_byte, // 速度
                    0         // 时间
                };

                // 构建完整命令
                auto cmd = build_command(0x03, 0xA1, data); // 地址0x03，模式0xA3

                // 发送命令
                std::vector<uint8_t> response;
                if (!send_command(cmd, response))
                {
                    return false;
                }
                
                usleep(5000);

                // 这里可以添加响应验证逻辑
                
            }
           
        }
        return true;
    }
};

// 全局控制器实例
RS485Controller *g_controller = nullptr;

void commandCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (!g_controller)
        return;

    int command = msg->data[0];
    ROS_DEBUG("Received command: %d", command);

    if (command >= 1 && command <= 6)
    {

        g_controller->set_servo_position(command,  msg, 950);
    }
}



int main(int argc, char **argv)
{
   // cout << "11111111111111111111" << endl;
    ros::init(argc, argv, "hand_control_node");
    ros::NodeHandle nh;

    try
    {
        // 初始化RS485控制器
        RS485Controller controller("/dev/ttyUSB0");
        g_controller = &controller;

        // 订阅命令话题
        ros::Subscriber sub = nh.subscribe<std_msgs::Int32MultiArray>(
            "hand_command", 10, commandCallback);

        ROS_INFO("Hand control node ready. Waiting for commands...");
        ros::spin();
    }

    catch (const std::exception &e)
    {
        ROS_FATAL("Initialization failed: %s", e.what());
        return 1;
    }

    g_controller = nullptr;
    return 0;
}