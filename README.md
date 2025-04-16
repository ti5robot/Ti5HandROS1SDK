# Ti5HandROS1SDK
钛虎五指灵巧手

# Ti5Hand ROS1 SDK 开发文档

## 一、环境准备
- **操作系统**: Ubuntu 20.04
- **ROS版本**: ROS Noetic
- **依赖安装**:
  sudo apt-get update
  sudo apt-get install git ros-noetic-serial ros-noetic-roscpp

二、完整使用流程
1. 获取代码

git clone https://github.com/ti5robot/Ti5HandROS1SDK.git
cd Ti5HandROS1SDK/hand_ws

2. 编译工作空间


# 清理旧编译文件（首次或出错时执行）
rm -rf build/ devel/

# 编译工作空间
catkin_make

# 配置环境变量
source devel/setup.bash

3. 设备权限设置


# 临时解决方案（每次重启后需重新执行）
sudo chmod 666 /dev/ttyUSB0

# 永久解决方案（推荐）
sudo usermod -a -G dialout $USER && sudo reboot

4. 运行控制节点

需要开启两个终端窗口：

终端1 - 运行机械手控制节点:

cd ~/Ti5HandROS1SDK/hand_ws
source devel/setup.bash
rosrun hand_control hand_control_node

终端2 - 运行键盘控制节点:

cd ~/Ti5HandROS1SDK/hand_ws
source devel/setup.bash
rosrun hand_control keyboard_publisher

三、键盘控制指令手册
指令格式说明

所有指令均由7个数字组成，第一个数字为模式选择，后6个为参数：
模式	示例指令	功能描述
1 0 0 0 0 0 0	执行预设握手动作
2 0 0 0 0 0 0	执行预设松手动作
3 0 0 0 0 0 0	全手指松开
3 45 45 45 45 45 45	全手指半握
3 90 90 90 90 0 0	四指全握（大拇指松开）
角度控制范围

    每个手指角度范围：0（完全伸直）~90（完全弯曲）

    特殊说明：大拇指最后一个参数可以不为0，

四、故障排除指南

    编译失败

        确保在hand_ws目录下执行编译

        检查ROS环境是否配置正确：echo $ROS_PACKAGE_PATH

        完整清理后重新编译：

        rm -rf build/ devel/
        catkin_make

    设备连接问题

        检查设备是否识别：ls /dev/ttyUSB*

        确认用户权限：groups | grep dialout

        临时解决方案：

        sudo chmod 666 /dev/ttyUSB0

    节点通信异常

        确认roscore已运行：roscore

        检查节点列表：rosnode list

        查看话题通信：rostopic list
