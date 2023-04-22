# XTDrone 记录

> [XTDrone文档](https://www.yuque.com/xtdrone/manual_cn/basic_config_13)
> 
> 无人机仿真环境配置十分困难，甚至比开发困难。
> 已知的仿真环境有如下：
> 
> 1. XTDrone 用过。有多种任务的示例代码
> 2. Prometheus 阿木实验室开发 必须要买他的手柄，不愧是商家
> 3. RflySim 北航可靠飞行控制组 基于Matlab和FlightGear，但我不是Matlab选手
> 4. PX4 SITL + ROS2 + Ignition Gazebo  需要肝
> 5. APM SITL + Gazebo Garden 需要肝
> 
> 万能安装工具 aptitude
> 
> 环境：
> 
> Ubuntu 20.04
> 
> ROS noetic
> 
> Gazebo11
> 
> PX4-Autopilot v1.13

安装过程中将包都下载在`~/`下，减少错误！有些脚本的路径没有自适应！

# 1 环境配置

## 1.1 安重新装gazebo

按官网要求 step-by-step 安装 gazebo11

> 重装是为了编译Gazebo Plugin

## 1.2 编译Gazebo Plugin

安装依赖

```sh
sudo apt-get install ros-noetic-moveit-msgs ros-noetic-object-recognition-msgs ros-noetic-octomap-msgs ros-noetic-camera-info-manager  ros-noetic-control-toolbox ros-noetic-polled-camera ros-noetic-controller-manager ros-noetic-transmission-interface ros-noetic-joint-limits-interface
```

下载源码，编译

```sh
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git
cd ~/catkin_ws
catkin build
```

编译报错，安装库

```shell
sudo apt-get install libgazebo11-dev
```

解决rosrun问题。运行rosrun，提示缺少rosbash，直接下载会失败。
解决方案：包名错误，下载`ros-noetic-rosbash`和`ros-noetic-catkin`

## 1.3 安装MAVROS

```shell
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
wget https://gitee.com/robin_shaun/XTDrone/raw/master/sitl_config/mavros/install_geographiclib_datasets.sh
sudo chmod a+x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

## 1.4 安装PX4固件

```shell
git clone https://github.com/PX4/PX4-Autopilot.git
mv PX4-Autopilot PX4_Firmware
cd PX4_Firmware
git checkout -b xtdrone/dev v1.13.2
git submodule update --init --recursive
make px4_sitl_default gazebo
```

编译完成后，会启动gazebo，里面有一个无人机，可以用QGC控制。

解决QGC Qt问题。

```shell
export LD_LIBRARY_PATH=/opt/qt514/lib/
```

修改.bashrc

```shell
source ~/catkin_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
```

之后通过launch文件启动

```shell
roslaunch px4 mavros_posix_sitl.launch
```

## 1.5 XTDrone 下载与配置

```shell
git clone https://gitee.com/robin_shaun/XTDrone.git
cd XTDrone
git checkout 1_13_2
git submodule update --init --recursive
# 修改启动脚本文件
cp sitl_config/init.d-posix/* ~/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/
# 添加launch文件
cp -r sitl_config/launch/* ~/PX4_Firmware/launch/
# 添加世界文件
cp sitl_config/worlds/* ~/PX4_Firmware/Tools/sitl_gazebo/worlds/
# 修改部分模型文件
cp -r sitl_config/models/* ~/PX4_Firmware/Tools/sitl_gazebo/models/ 
# 替换同名文件
cd ~/.gazebo/models/
rm -r stereo_camera/ 3d_lidar/ 3d_gpu_lidar/ hokuyo_lidar/
```

键盘控制

```shell
cd ~/PX4_Firmware
roslaunch px4 indoor1.launch
```

```shell
cd ~/XTDrone/communication/
python3 multirotor_communication.py iris 0
```

```shell
cd ~/XTDrone/control/keyboard
python3 multirotor_keyboard_control.py iris 1 vel
```

强行关闭gazebo

```shell
killall -9 gzclient
killall -9 gzserver
```

# 2 源码分析

我需要借助XTDrone平台完成无人机飞行任务。需要用到SLAM；最好可以有动态路径规划、视觉伺服。

为此，必须能通过MAVLink发送指令，控制无人机飞行，需要控制飞行速度，而不仅仅是顶点；在无人机上安装摄像头，获取图像信息，并进行处理。

可能有用的工具：

1. ROS 提供了 MAVLink 的接口 <mark>MAVROS</mark>。

2. <mark>ORB SLAM</mark> 的调用，通过ROS或不通过ROS。
