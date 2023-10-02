# Nav2

> ROS2 下平面机器人导航包
> 
> 官方例程基于 Gazebo classical，与 Gazebo Garden 存在版本冲突问题
> 
> Dknt 2023.10

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
