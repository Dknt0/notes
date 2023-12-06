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

# 1. PX4 & ROS 2

## 1.1 仿真启动

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

# 2 PX4 & ROS 1

### 2.1 环境配置

按照 PX4 用户手册，先运行 Tool 下的脚本安装 ubuntu 依赖，然后编译。

编译完成后，按用户手册中的操作配置环境变量。

为了使用 ROS，还需要下载 mavros 包，并下载一个数据集。

之后，可以通过 PX4-Autopilot/launch/mavros_posix_sitl.launch 启动 PX4，修改文件中的参数，选择自己的无人机和场景。

### 2.2 地图

PX4 为 Gazebo Classic 准备了许多地图，可以直接用于验证某些任务。描述如下

| 地图文件                    | 描述                        |
| ----------------------- | ------------------------- |
| baylands.world          | 这是一个大型的公园场景。有树林，可以进行避障实验。 |
| boat.world              | 一片空旷的海洋。用于测试无人船。          |
| empty.world             | 空地。默认场景。                  |
| iris_irlock.world       | 空地+二维码。精准降落测试。            |
| ksql_airport.world      | 一个大型机场。模型很粗糙，感觉用不到。       |
| mcmillan_airfield.world | 大型荒漠。用于航拍测试。              |
| ocean.world             | 类似 boat                   |
| ramped_up_wind.world    | 经典赛车道                     |
| safe_landing.world      | 安全降落场景                    |
| warehouse.world         | 一堆货架                      |
| yosemite.world          | 山地地形贴图地图。                 |

> 仿真消耗大量算力。运行仿真前，一定要成功安装显卡驱动。
> 
> 初次加载这些地图时需要下载模型，Gazebo 可能卡在启动界面，多等一会儿就好。

### 2.3 模型

PX4 提供了一些搭载不同传感器的模型，我们主要用四旋翼。

| 模型                | 描述   |
| ----------------- | ---- |
| iris_depth_camera | 深度相机 |

## 2.x Debug

### 2.x.1 深度相机没有话题

在 Gazebo 中加载 PX4 自带的深度相机无人机 iris_depth_camera，启动后相机模型无法加载，ROS 中也没有图像话题。原因未知。

解决方案：复制 iris 的 sdf 文件，并在文件中手动添加 depth_camera.sdf 中的模型，创建 joint 连接相机与 base_link。
