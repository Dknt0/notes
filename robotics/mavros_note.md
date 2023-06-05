# MAVROS 笔记

> 参考：
> 
> 1. [ROS 1 with MAVROS | PX4 User Guide](https://docs.px4.io/main/en/ros/ros1.html)
> 
> 2. [XTDrone 源码](https://github.com/robin-shaun/XTDrone)
> 
> Dknt 2023.5.13

MAVROS 用于连接 ROS 与 PX4 固件。使用过程中会加载不同的插件，实现不同功能。

MAVROS 的端口配置全部在 launch 文件中完成，MAVROS 编程实际上只是在操作 ROS 话题和服务，用户不直接使用 MAVLink 。

# 1 节点

主要包含三个节点。

## 1.1 mavros_node

主通信节点，按照配置文件加载各种插件。可以只运行这个节点。

## 1.2 gcs_bridge

用于和地面站间的通信。额外的地面站？

## 1.3 event_launcher

按事件加载？

# 2 插件

插件话题命名：

`/mavros/plubin_name/topic_name`

XTD 中的话题：

`/type_id/mavros/plubin_name/topic_name`

默认插件列表：

`/opt/ros/noetic/share/mavros/launch/px4_pluginlists.yaml`

默认加载配置：

`/opt/ros/noetic/share/mavros/launch/px4_config.yaml`

按照默认配置会调用大部分插件，有许多冗余话题和服务。不精准配置插件，可能限制仿真性能。

> 插件列表中黑白名单有什么区别？

`px4.launch`会按照中的默认配置加载相应插件。

## 2.1 local_position

在 TF 和以下话题中发布位置消息，全局定位

**Published Topics**

* `~local_position/pose` (geometry_msgs/PoseStamped)

* `~local_position/velocity` (geometry_msgs/TwistStamped)

## 2._ setpoint_position

位置控制目标点设置

**Published Topics**

* `~mavros/setpoint_position/local` (geometry_msgs::PoseStamped)

## 2._ command

解锁、起飞等常用命令

**Services**

* `~cmd/arming` (mavros_msgs/CommandBool)

* `~cmd/takeoff` (mavros_msgs/CommandTOL)

* `~cmd/land` (mavros_msgs/CommandTOL)

## 2._ sys_status

发布系统状态，提供更改模式等服务

**Published Topics**

- `mavros/state` (mavros_msgs::State)

**Services**

* `~set_mode` (mavros_msgs/SetMode)

# 3 外部控制

PX4 固件全局坐标系使用 NED 北东下坐标系，但 MAVROS 使用 ENU 东北上坐标系。局部坐标系同理，PX4 使用 FRD 前右下，MAVROS 使用 FLU 前左上。

> 问题：进入外部控制模式后无法切换到其他模式！
