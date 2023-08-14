# PX4 SITL 仿真

> 软件版本：
> 
> Ubuntu 22.04
> 
> ROS2 Humble
> 
> Gazebo Garden
> 
> 参考：
> 
> [Gazebo Simulation | PX4 User Guide](https://docs.px4.io/main/en/sim_gazebo_gz/)
> 
> [Gazebo](https://gazebosim.org/docs/garden/ros2_integration)
> 
> Dknt 2023.7

# 0 环境配置

略，哈哈哈^_^

# 1 仿真启动

我没有找到启动仿真的 launch 文件，官网上是通过运行一个可执行文件实现的启动。位置如下:

```shell
PX4-Autopilot/build/px4_sitl_default/bin/px4
```

可以通过这个程序创建一个新仿真，或向已经运行的仿真中添加一架新无人机。

# 2 仿真场景搭建

之间将场景文件添加到场景文件夹下。路径：

```shell
PX4-Autopilot/Tools/simulation/gz/worlds
```

# 3 创建新无人机

参考：[Gazebo Simulation | PX4 User Guide](https://docs.px4.io/main/en/sim_gazebo_gz/#adding-new-worlds-and-models)

# _ MAVLINK UDP Broadcast

手机、平板上 QGC 的虚拟遥感可以模拟真实遥控器控制仿真中的无人机，为此需要在 SITL 运行时打开 MAVLINK 的网络广播。方法如下：

修改`PX4/build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink`文件中 mavlink 启动命令如下：

```shell
mavlink start -x -u $udp_gcs_port_local -r 4000000 -f -p
```

增加 -p 参数。

然后将运行仿真的电脑和手机连接到同一路由器下，就可以控制了。
