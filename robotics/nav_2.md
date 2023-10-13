# Nav2

> ROS2 下平面机器人导航包
> 
> Dknt 2023.10
> 
> 参考：
> 
> [Getting Started &mdash; Nav2 1.0.0 documentation](https://navigation.ros.org/getting_started/index.html)
> 
> [Can&#39;t open gazebo11 when run tb3_simulation_launch.py · Issue #2757 · ros-planning/navigation2 · GitHub](https://github.com/ros-planning/navigation2/issues/2757)

Nav2 是 ROS 2 平面机器人导航包。提供感知、规划、控制、定位、可视化等功能。使用行为树来组织模块化的服务器来工作。

Nav2 的期望输入为符合 REP-105 的坐标变换、源地图（如使用静态代价地图）、行为树 XML 文件、以及其他必需的传感器信息，输出速度指令。

包含下列工具：

* 加载、管理、保存地图——`Map Server`

* 定位——`AMCL`

* 规划、壁障——`Nav2 Planner`

* 控制——`Nav2 Controller`

* 轨迹光滑——`Nav2 Smoother`

* 由传感器生成代价地图——`Nav2 Costmap 2D`

* 由行为树构建复杂行为——`Nav2 Behavior Trees and BT Navigator`

* 错误是计算恢复行为——`Nav2 Recoveries`

* 跟随路径点——`Nav2 Waypoint Follower`

* 生命周期和看门狗管理——`Nav2 Lifecycle Manager`

* 插件基类——`Nav2 Core`

* 碰撞监测——`Collision Monitor`

* Python3 API——`Simple Commander`

* 速度平滑——`Velocity Smoother`

安装：

```shell

```

# 0 例程

官方例程基于 Gazebo classical，与 Gazebo Garden 存在版本冲突问题，需要重装 Gazebo。尝试在 ROS Foxy + Gazebo Classicl 中运行，之后在 Humble 中使用。

运行`tb3_simulation_launch.py`，gazebo 无法启动，需要在运行 launch 前运行 gazebo 的 setup 脚本。

```shell
./usr/share/gazebo/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models
```

之后再启动示例，模型加载需要很长时间，且在 rviz 中显示不正确。将就用吧。
