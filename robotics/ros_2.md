# ROS 2 Humble

> ROS 2 Humble 笔记
> 
> Dknt 2023.6

ROS 2 Humble 是 ROS 2 的第一个 LTS 发行版，运行在 Ubnutu 22.04 系统上。ROS 2 与 ROS 1 最大的区别在于底层通信协议的改变 —— DDS。ROS 2 可以运行在裸机上。

ROS 2 Humble 也可以通过源码编译运行在其他系统上。

> 先大概记录下内容，再分章节总结笔记

# 命令行

* 运行

```shell
ros2 run <pkg_name> <node_name> # 运行节点
```

重映射，用于改变节点、服务、话题名字。

```shell
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

启动同时设置参数

```shell
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

* 节点

```shell
ros2 node info <node_name>
ros2 node list
```

* 话题

```shell
ros2 topic list [-t]
ros2 topic echo <topic_name>
ros2 topic info <topic_name>
ros2 topic pub [--once] [--rate 1] <topic_name> <msg_type> <arguments>
ros2 topic hz /turtle1/pose # 统计话题频率
```

ROS 2 的话题好像没有自动补全功能。可以用 rqt 来发布话题，很方便。

* 服务

```shell
ros2 service list [-t]
ros2 service type <service_name>
ros2 service find <type_name> # 搜寻特定类型的服务
ros2 service call <service_name> <service_type> <arguments>
```

* 参数

ROS 2 中每一个节点有自己的参数。ROS 2 没有参数服务器，这一点与 ROS 1 不同。（可能是与通信结构有关，不存在中心节点）

参数类型可以是整数、浮点、布尔、字符串、列表。

```shell
ros2 param list
ros2 param get <node_name> <parameter_name>
ros2 param set <node_name> <parameter_name> <value>
ros2 param dump <node_name> # 显示节点全部参数
```

记录参数到文件，从文件中加载参数

```shell
ros2 param dump <node_name> > <parameter_file.yaml>
ros2 param load <node_name> <parameter_file.yaml>
```

注意，这种方式无法加载只读参数，命令行中会报错。

可以在启动节点时候加载参数，这样可以设置只读参数。

```shell
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

* 动作

```shell
ros2 action list [-t]
ros2 action info <action_name>
ros2 action send_goal [--feedback] <action_name> <action_type> <values>
```

* 查询

显示话题、服务详细信息。这里的`type_name`可以是话题、服务、动作。

```shell
ros2 interface show <type_name>
```

* 启动

```shell
ros2 launch <pkg_name> <launch_file.py>
```

* 录包与重放

录包与重放是针对话题的。

```shell
ros2 bag record <topic_name> # 录包
ros2 bag record [-o bag_file_name] <topic_name1> <topic_name2> # 指定包名
ros2 bag record -a # 录制全部话题
ros2 bag info <bag_file_name>
ros2 bag play <bag_file_name>
```

> 注意，原始视频、点云流话题录包相当占用磁盘，录制前需要保证有足够的存储空间。

* rqt 工具

很好用。

局限：不能操作参数。

* 日志

可以用 rqt 浏览、过滤、保存、加载日志。

日志分为五个等级：

```shell
Fatal # 灾难
Error # 错误
Warn # 警告
Info # 信息
Debug # debug 信息
```

通常情况默认等级为`Info`，默认等级以下的日志会被隐藏。

可以在启动节点时附加参数来改变默认日志等级：

```shell
ros2 run <pkg_name> <node_name> --ros-args --log-level WARN
```

# 启动

ROS 2 的启动文件有三种，XML、Python、YAML。Python 最为方便。

1

# _ 开发

## _. colcon 工具

`colcon`是 catkin 等 ROS 编译工具的迭代版本。

ROS 2 工作空间目录结构：

```bash
./ros2_ws
├── build
├── install
│   └── setup.bash # 初始化脚本
├── log
└── src
    ├── ros2_pkg
        └── 
```
