# ROS 2 Humble

> ROS 2 Humble 笔记
> 
> Dknt 2023.6

ROS 2 Humble 是 ROS 2 的第一个发行版，运行在 Ubnutu 22.04 系统上。ROS 2 与 ROS 1 最大的区别在于底层通信协议的改变 —— DDS。ROS 2 可以运行在裸机上。

ROS 2 Humble 也可以通过源码编译运行在其他系统上。

> 先大概记录下内容，再按章节总结笔记

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

显示话题、服务详细信息。这里的`type_name`可以是话题或服务。

```shell
ros2 interface show <type_name>
```

* 参数

ROS 2 中每一个节点有自己的参数，这一点与 ROS 1 不同。

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

1

```shell

```





```shell
ros2 service ...
ros2 action ...
ros2 pkg ...
```

* rqt 工具

很好用。

局限：不能操作参数。
