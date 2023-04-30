# PX4 SITL 仿真

> 环境：
> 
> Ubuntu 20.04
> 
> PX4-Autopilot v1.13
> 
> 参考：
> 
> [PX4 User Guide](https://docs.px4.io/main/en/)
> 
> [XTDrone使用文档 · Yuque](https://www.yuque.com/xtdrone/manual_cn)
> 
> Dknt 2022.4.22

# 0 环境搭建

## 0.1 PX4 SITL 的失而复得

PX4提供了比APM更完善的仿真功能，同时由于更自由的证书，更广泛地应用于学界和业界。

由于在 Ubuntu 20.04 上安装 XTDrone 的偶然成功，我果断地放弃了APM，学习 PX4 SITL。

PX4 提供了在 Ubuntu 22.04 上对高版本 ROS 和 Gazebo 的支持，等把 SITL、MAVLink、SLAM 学明白了，就转战 Ubuntu 22.04。

安装过程见 XTDrone 笔记。

# 2 MAVLink 协议

## 2.1 MAVLink

## 2.2 MAVSDK

MAVSDK 是 MAVLink 的 C++ API，好像只支持PX4。

使用 MAVSDK 可以达到我预期的控制效果。

## 2.3 MAVROS

不太懂这个包是干什么的，但XTDrone源码用的是这个。

# 3 ORB SLAM
