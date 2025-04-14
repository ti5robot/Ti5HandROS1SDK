#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include <sstream>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("hand_command", 10);

    ROS_INFO("Enter 7 numbers separated by spaces, then press Enter to publish (q to quit)");

    while (ros::ok())
    {
        std::string input;
        std::getline(std::cin, input); // 读取整行输入

        if (input == "q" || input == "Q")
        {
            break;
        }
            

        std::istringstream iss(input);
        std::vector<int> numbers;
        int num;

        // 解析输入的7个数字
        while (iss >> num && numbers.size() < 7)
        {
            numbers.push_back(num);
        }

        if (numbers.size() == 7)
        {
            std_msgs::Int32MultiArray msg;
            msg.data = numbers;
            pub.publish(msg);
            ROS_INFO("Published: %d %d %d %d %d %d %d",
                     numbers[0], numbers[1], numbers[2],
                     numbers[3], numbers[4], numbers[5],
                     numbers[6]);
        }
        else
        {
            ROS_WARN("Please enter exactly 7 numbers separated by spaces");
        }
    }

    return 0;
}