# EGO-Planner 学习笔记

> 浙江大学 FAST 实验室无人机导航包
> 
> Dknt 2023.10.19
> 
> 环境：
> 
> Ubuntu 20.04 + ROS Noetic
> 
> 参考：
> 
> [GitHub - ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)
> 
> [GitHub - ZJU-FAST-Lab/ego-planner-swarm: An efficient single/multi-agent trajectory planner for multicopters.](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)

# 0 概述

EGO-Planner 是一种不需要计算 ESDF 的局部规划器。

依赖于这些库/项目：

* [Armadillo](https://arma.sourceforge.net/speed.html) 线性代数库

* [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) 无人机局部规划

* [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) 无约束优化问题求解器，仅包含头文件

* [mockamap](https://github.com/HKUST-Aerial-Robotics/mockamap) 地图生成

* [Teach-Repeat-Replan](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan) 好像是个飞控（好家伙...）

> Armadillo 类似与 Eigen
> 
> LBFGS-Lite 可以用 Ceres

从 Github 上克隆 swarm 项目，直接编译会失败，是由于消息库没有生成导致头文件缺失，解决方法参考：

https://github.com/ZJU-FAST-Lab/ego-planner-swarm/issues/9

运行示例代码

```shell
roslaunch ego_planner simple_run.launch
```

# 1 运行过程分析

> 其实看 single_run_in_sim.launch 结构更清楚，这个只启动一个无人机。

下载 Github 项目，工作空间 src 下包含两个元包，`planner`和`uav_simulator`，前者是 ego-planner 本身，后者是一个仿真器，仿真画面在 Rviz 中显示。

我们从示例代码开始，分析项目启动流程，示例代码启动文件如下。

```xml
<launch>
    <!-- 打开 rviz，加载配置 -->
    <include file="$(find ego_planner)/launch/rviz.launch"/>
    <!-- 生成地图，运行规划 -->
    <include file="$(find ego_planner)/launch/swarm.launch"/>
</launch> 
```

**地图显示**——`rviz.launch`启动了 rviz， 加载预设置号的参数，用于显示地图、轨迹、无人机等。

`swarm.launch`用于运行仿真，其中包含了如下节点：

**随机地图生成**——`map_generator random_forest`节点，这个节点不属于 ego 规划本身，作为仿真的一部分使用，基本用法如下。单独运行这个节点，可以在 Rviz 中生成随机地图。

```xml
<node pkg ="map_generator" name ="random_forest" type ="random_forest" output = "screen">
    <!-- 地图大小 -->
    <param name="map/x_size" value="36" />
    <param name="map/y_size" value="20" />
    <param name="map/z_size" value="3" />
    <!-- 栅格地图分辨率 -->
    <param name="map/resolution" value="0.1"/>  
    <!-- 地图种子 -->
    <param name="ObstacleShape/seed" value="1"/>
    <!-- 柱状障碍物数量、大小 -->
    <param name="map/obs_num" value="200"/>
    <param name="ObstacleShape/lower_rad" value="0.5"/>
    <param name="ObstacleShape/upper_rad" value="0.7"/>
    <param name="ObstacleShape/lower_hei" value="0.0"/>
    <param name="ObstacleShape/upper_hei" value="3.0"/>
    <!-- 圆圈障碍物数量、大小 -->
    <param name="map/circle_num" value="200"/>        
    <param name="ObstacleShape/radius_l" value="0.7"/>        
    <param name="ObstacleShape/radius_h" value="0.5"/>        
    <param name="ObstacleShape/z_l" value="0.7"/>        
    <param name="ObstacleShape/z_h" value="0.8"/>
    <param name="ObstacleShape/theta" value="0.5"/>
    <!-- 传感器设置 -->
    <param name="sensing/radius" value="5.0"/>        
    <param name="sensing/rate" value="1.0"/>  
    <param name="min_distance" value="1.2"/>         
</node>
```

之后在 `swarm.launch`中调用了 10 个 `run_in_sim.launch`，创建 10 个无人机，为每一架无人机设置起点和目标点，分别运行规划程序。这个启动文件中调用了如下启动文件 \ 节点：

**启动ego**——`ego_planner advanced_param.xml`文件，启动了`ego_planner_node`节点，即规划的主节点，有非常多的参数。这个节点的源文件是`ego_planner_node.cpp`，其中示例化了`EGOReplanFSM`类，即规划的状态机，这个是最核心的部分。

**轨迹服务器**——`ego_planner traj_server`节点，不知道有什么用处

**启动仿真器**——`ego_planner simulator.xml`文件，设置地图大小，初始位置，无人机 id。其中，地图生成部分被注释了，因为`swarm.launch`已经生成了随机地图，之后启动了如下节点：`poscmd_2_odom poscmd_2_odom`里程计，`odom_visualization`里程计可视化，`pcl_render_node`生成点云等传感器信息。

`swarm.launch`文件最后被 CDATA 注释掉的是动态障碍物生成。

> 先在 Ubuntu 20 下实现一个导航吧
> 
> 比起 ORB-SLAM 3 的三万行，代码量不是很多...

1

# 2 规划算法分析

## 2.1 轨迹规划状态机

* ROS 相关

发布如下话题

`replan_pub_`

`new_pub_`

`bspline_pub_` 类型 traj_utils::Bspline

`data_disp_pub_` 类型 traj_utils::DataDisp

`swarm_trajs_pub_` 类型 traj_utils::MultiBsplines 

`broadcast_bspline_pub_` 类型 traj_utils::Bspline

收听如下话题

`waypoint_sub_` 类型 geometry_msgs::PoseStamped  2d nav goal 收听

`odom_sub_` 类型 nav_msgs::Odometry  里程计话题

`swarm_trajs_sub_` 类型 traj_utils::MultiBsplines

`broadcast_bspline_sub_` 类型 traj_utils::Bsplines

`trigger_sub_` 类型 geometry_msgs::PoseStamped

创建了如下定时器

`exec_timer_`

`safety_timer_`

1

* 运行过程

`init`函数从 ROS 参数服务器读取了 fsm 相关的参数、路径点，创建了显示器、规划管理器。初始化了一系列发布者、收听者。

之后，按照模式执行。如果是手动目标模式，创建 goal 收听者，结束，剩余过程在回调函数中执行。如果是全局路径点模式，则等待里程计和触发器准备就绪。

ego 运行分为仿真和实物两种，实物中需要使用遥控上的触发器控制规划的运行，仿真中忽略这一点。

`waypointCallback`目标点回调函数，将规划初始位置设置为里程计当前位置，将消息中的目标点传入`planNextWaypoint`，进行规划







* 状态机逻辑

状态机的执行依靠定时器，在回调函数中按照当前状态执行对应功能。

编写实现了状态输出到终端、切换状态的函数。

状态切换会在其他函数中实现。切换了状态的函数有

planNextWaypoint



* 与其他模块的关系

PlanningVisualization

EGOPlannerManager

初始化函数作为整个状态机器的入口

## 2.2 规划管理器

planner_manager_

实现全局路径规划、局部路径规划

1

与其他模块的关系

PlanningVisualization
