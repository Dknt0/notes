# XTDrone 记录

> [XTDrone文档](https://www.yuque.com/xtdrone/manual_cn/basic_config_13)
> 
> 无人机仿真环境配置十分困难，甚至比开发困难。
> 已知的开源仿真环境有如下：
> 
> 1. XTDrone 有多种任务的示例代码
> 2. Prometheus 阿木实验室开发 必须要买他的手柄，不愧是商家
> 3. RflySim 北航可靠飞行控制组 基于Matlab和FlightGear，但我不是Matlab选手
> 4. [Hello Sky - GAAS](https://gaas.gitbook.io/guide/)
> 5. PX4 SITL + ROS2 + Ignition Gazebo  需要肝，之后会自己逐步搭平台
> 6. APM SITL + Gazebo Garden 需要肝，不如 PX4
> 
> 万能安装工具 aptitude
> 
> 教程中编译使用 cakin build，和 cakin_make 是两种工具，推荐用前者
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
> 
> 依赖：
> 
> OpenCV 4.2 —— 图像库
> 
> Eigen ? —— C++ 线性代数库
> 
> Ceres 1.14.x —— 最小二乘优化库
> 
> g2o ? —— 图优化库
> 
> PCL ? —— 点云库
> 
> Qt ? —— GUI 库
> 
> Protobuf ? —— 消息序列化库
> 
> Dknt 2023.4

安装过程中将包都下载在`~/`下，和文档里一致，减少错误！有些脚本的路径没有自适应！

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

# 2 基本使用

## 2.1 键盘控制

键盘控制

```shell
# 启动 sitl
cd ~/PX4_Firmware
roslaunch px4 indoor1.launch
# 通信程序
cd ~/XTDrone/communication/
python3 multirotor_communication.py iris 0
# 键盘控制程序
cd ~/XTDrone/control/keyboard
python3 multirotor_keyboard_control.py iris 1 vel
```

强行关闭 gazebo

```shell
killall -9 gzclient
killall -9 gzserver
```

## 2.2 EKF 配置

如果使用 VO，需要配置 EKF。

打开文件

```shell
gedit ~/PX4_Firmware/build/px4_sitl_default/etc/init.d-posix/rcS
```

修改参数

```
# GPS used
#param set EKF2_AID_MASK 1
# Vision used and GPS denied
param set EKF2_AID_MASK 24

# Barometer used for hight measurement
#param set EKF2_HGT_MODE 0
# Barometer denied and vision used for hight measurement
param set EKF2_HGT_MODE 3
```

修改完成后，需要删除旧的 eeprom 数据

```shell
rm ~/.ros/eeprom/parameters*
rm -rf ~/.ros/sitl*
```

## 2.3 VIO VINS-Fusion

> 必须用 OpenCV **4.2.0** 才能运行

VINS-Fusion可以进行单目 + IMU, 双目 + IMU 或只有双目的多传感器融合定位，另外也可以融进GPS信息。

优势：VINS-Fusion 不需要运动初始化，ORB SLAM 需要。在仿真中，无人机飞行在 Offboard 模式下实现，没有位置信息是无法切换 Offboard 模式的，这会造成矛盾，因此使用 VINS-Fusion 作为 VIO。

> VINS-Fusion with Ubuntu 20.04 + ROS Noetic + OpenCV 4.2.0 + Ceres 1.4.x 配置方法：
> 
> https://blog.csdn.net/xiaojinger_123/article/details/121517771?utm_source=app&app_version=4.20.0

默认的仿真使用 iris_0 + 双目相机，<mark>配置文件</mark>如下

```shell
gedit ~/catkin_ws/src/VINS-Fusion/config/xtdrone_sitl/px4_sitl_stereo_imu_config.yaml
```

运行

```shell
# 运行 SITL + Gazebo
roslaunch px4 indoor1.launch
# 运行 VINS-Fusion
cd ~/catkin_ws
bash scripts/xtdrone_run_vio.sh
# 将视觉定位信息发送给 PX4，类似于机载电脑与固件间基于MAVROS的通信
cd ~/XTDrone/sensing/slam/vio
python vins_transfer.py iris 0
```

> 手写 MAVSDK keyboard 程序需要用端口 udp://:14030

## 2.4 VINS-Fusion + RTABMap

需要修改仿真 launch 文件中的模型，使用 realsence 相机。

需要更改  rtabmap_vins.launch 中静态TF发布器的坐标为`/depth_camera_base`（去掉命名空间）。

下载并配置

```shell
sudo apt install ros-noetic-rtabmap*
sudo apt install ros-noetic-octomap*
# 复制配置文件
sudo cp ~/XTDrone/sensing/slam/vio/VINS-Fusion/config/xtdrone_sitl/rgbd.rviz /opt/ros/melodic/share/rtabmap_ros/launch/config/
```

运行：按 2.3 节 运行仿真和 VINS-Fusion

> 使用 GPS + 气压计 定位

之后运行 RTABMap

```shell
roslaunch vins rtabmap_vins.launch
```

可以正常运行，稠密建图、八叉树地图。但是无人机位姿会有很大漂移，不知道为什么，之后可以直接用真值试试。

## 2.5 ego_planner

> 注意，ego_planner 需要使用 catkin_make 编译，catkin build 会产生一些链接问题。可以在其他工作空间用 catkin_make 编译 ego_planner。

基于视觉的三维路径规划。

<mark>先运行 VINS-Fusion，再运行 ego_planner</mark>：

```shell
// 切换相机位姿的坐标系方向
cd ~/XTDrone/motion_planning/3d
python ego_transfer.py iris 0
// 启动 ego_planner
roslaunch ego_planner single_uav.launch 
```

目标点切换：

```xml

```

一塌糊涂...

## 2.6 2D Laser SLAM

对于简单的场景，2D SLAM 稳定性更强。使用 HectorSLAM。

安装：

```shell
sudo apt install ros-noetic-hector-slam*
sudo apt install ros-noetic-map-server
```

laser_2d 坐标系命名空间错误

# 3 源码分析

我需要借助XTDrone平台完成无人机飞行任务。需要用到SLAM；最好可以有动态路径规划、视觉伺服。

为此，必须能通过MAVLink发送指令，控制无人机飞行，需要控制飞行速度，而不仅仅是定点；在无人机上安装摄像头，获取图像信息，并进行处理。

可能有用的工具：

1. ROS 提供了 MAVLink 的接口 <mark>MAVROS</mark>。

2. 我可以写 MAVSDK，与 MAVROS 相比谁更有优势？

3. <mark>ORB SLAM</mark> 的调用，通过ROS或不通过ROS。

XTDrone 对 PX4, ROS 源码的改动：

1. 修改了 gazebo_ros 包，用处未知

2. 修改了 mavlink_sitl_gazebo 包，扩充了地图。这个包位于 `PX4_Firmware/Tools/sitl_gazebo`

3. 修改了 PX4 源码中生成无人机的 launch 文件，添加了集群控制

4. 修改了 gazebo 的模型库，更改了一些传感器

## 3.1 键盘控制源码

命令行运行一个 launch 文件，两个 py 脚本。

`launch` 文件启动 gazebo、运行 sitl 并生成无人机模型、启动 MAVROS。

`multirotor_communication.py`将期望控制指令按照 MAVROS 格式转发

`multirotor_keyboard_control.py`读取键盘输入，发布期望控制话题

> MAVROS 的活好像都能用 MAVSDK 解决，但也看一下 MAVROS 吧
