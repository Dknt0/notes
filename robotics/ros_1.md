# ROS Noetic 编程

> ROS Noetic 复习。ROS 是我在刚入门 Linux 时学习的，当时没有什么经验，错过了许多重点。这次再读文档，记录一些用得到的东西。
> 
> 参考：
> 
> [Introduction · Autolabor-ROS机器人入门课程《ROS理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/)
> 
> Dknt 2023.4

# 0 环境配置

略

# 1 工作空间

## 1.1 创建与初始化

1. 创建工作空间

```shell
mkdir -p catkin_ws/src
cd catkin_ws
# 编译基于cmake
catkin_make
```

> 编译后生成 devel/setup.bash 初始化文件，修改 ~/.bashrc 使其默认执行：
> `source {PATH_TO_WS}/devel/setup.bash`

2. 创建功能包 

```shell
cd catkin_ws/src
# 包名+依赖，通常需要这三个依赖
catkin_create_pkg my_pkg roscpp rospy std_msgs
```

## 1.2 项目结构

```shell
./catkin_ws
├── CMakeLists.txt # 好像没有这个
├── build
├── devel
│   └── setup.bash # 初始化脚本
└── src
    ├── CMakeLists.txt # 
    └── my_pkg
        ├── CMakeLists.txt # cmake
        ├── package.xml # 包信息
        ├── include # 头文件
        ├── src # C++ 源文件
        ├── scripts # py 等脚本
        ├── launch # 启动文件
        ├── msg # 消息定义
        ├── srv # 服务定义
        ├── world # sdf 世界
        ├── urdf # urdf & xacro 机器人
        ├── config # gz rviz 等配置文件
```

# 2 通信

## 2.1 话题 Topic

### 2.1.1 标准话题

图像？点云？

```cpp
#include <std_msgs/String.h>
std_msgs::string; // 字符串，底层是 std::string
...
```

### 2.1.2 自定义话题

自定义话题 msg/Info.msg

```shell
string name
uint16 age
```

修改 package.xml

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

修改 CML

```cmake
# 导包
find_package(catkin REQUIRED COMPONENTS
  ...
  message_generation
)
# 配置
add_message_files(
  FILES
  Info.msg
)
# 生成时依赖，可以使用其他功能包中定义的消息
generate_messages(
  DEPENDENCIES
  std_msgs
)
# 执行时依赖
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
```

编译后，消息头文件位于 catkin_ws/devel/include/my_package/Info.h

### 2.1.3 C++ 话题发布者

**Publisher C++**

```cpp
#include <ros/ros.h>
#include <my_pkg/Info.h>

// argv 不能是 const
int main(int argc, char** argv) {
    // 初始化
    ros::init(argc, argv, "node_publisher");
    // 句柄
    ros::NodeHandle handle;
    // 发布者 <消息类型>(话题名, 缓冲大小)
    ros::Publisher pub = handle.advertise<my_pkg::Info>("test_topic", 10);
    // 生成消息
    my_pkg::Info info_msg;
    info_msg.name = "Ivan";
    info_msg.age = 20;
    // 延时 10 ps
    ros::Rate rate(10);
    // 循环发送
    while (ros::ok()) {
        pub.publish(info_msg); // 发布
        ROS_INFO("Sent: %s %d", info_msg.name.c_str(), info_msg.age); // 显示
        rate.sleep(); // 延时
        ros::spinOnce(); // 回旋
    }
    return 0;
}
```

**CML**

额外添加依赖

```cmake
add_dependencies(node_name ${PROJECT_NAME}_generate_messages_cpp)
```

### 2.1.4 C++ 话题收听者

**Subscriber C++**

```cpp
#include <ros/ros.h>
#include <my_pkg/Info.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_subscriber"); // 初始化节点
    ros::NodeHandle handle; // 句柄
    // 注意回调函数参数类型，这里写为匿名函数，可以写函数指针、仿函数
    // <消息类型>(话题名，缓冲大小，回调函数)
    ros::Subscriber sub = handle.subscribe<my_pkg::Info>("test_topic", 10,
        [&](const my_pkg::Info::ConstPtr& msg_p) -> bool {
            ROS_INFO("Received: %s %d", msg_p->name.c_str(), msg_p->age);
            return true;
        });
    ros::spin(); // 循环
    return 0;
}
```

> 使用匿名函数作为回调函数，编译时会调用 boost 库中的工具，原理未知，报错一片。使用时注意参数列表中类型是否正确。后面服务回调函数同理

CML 中需要添加依赖项，见 2.1.3 节

## 2.2 服务 Service

### 2.2.1 自定义服务

自定义服务文件 Req.srv

```cpp
int32 num1
int32 num2
---
int32 sum
```

修改 package.xml

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

修改 CML

```cmake
# 导包
find_package(catkin REQUIRED COMPONENTS
  ...
  message_generation
)
# 配置
add_service_files(
  FILES
  Req.srv
)
# 生成时依赖
generate_messages(
  DEPENDENCIES
  std_msgs
)
# 执行时依赖
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)
```

编译后，消息头文件位于 catkin_ws/devel/include/my_package/Req.h

### 2.2.2 C++ 服务端节点

Server C++

```cpp
#include <ros/ros.h>
#include <my_pkg/Req.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "server"); // 初始化节点
    ros::NodeHandle handle; // 句柄
    // 实例化服务端，可以传入函数指针、仿函数、匿名函数
    // 使用匿名函数要显式给出模板参数，自动推导会调boost库，报错
    // 必须引用传入所有外部参数
    ros::ServiceServer server = handle.advertiseService<my_pkg::Req::Request, my_pkg::Req::Response>("test_service",
        [&](my_pkg::Req::Request &req, my_pkg::Req::Response &resp) -> bool {
            resp.sum = req.num1 + req.num2;
            ROS_INFO("I think %d + %d = %d", req.num1, req.num2, resp.sum);
            return true;
        });
    ros::spin(); // 循环
    return 0;
}
```

CML 添加额外依赖

```cmake
add_dependencies(node_name ${PROJECT_NAME}_gencpp)
```

### 2.2.3 C++ 客户端节点

Client C++

```cpp
#include <ros/ros.h>
#include <my_pkg/Req.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "client"); // 初始化节点
    ros::NodeHandle handle; // 句柄
    // 实例化客户端
    ros::ServiceClient client = handle.serviceClient<my_pkg::Req>("test_service");    
    client.waitForExistence(); // 等待服务出现
    my_pkg::Req req; // 生成请求
    req.request.num1 = 1;
    req.request.num2 = 2;
    bool flag = client.call(req); // 调用服务，返回值存在 req 中
    if (flag) { // 判断是否成功
        ROS_INFO("Server said 1 + 2 = %d", req.response.sum);
    }
    else {
        ROS_ERROR("Falied to call server.");
        return 1;
    }
    return 0;
}
```

CML 中需要添加依赖项，见 2.2.2 节

## 2.3 动作 Action

## 2.4 参数 Parameter

### 2.4.1 标准参数类型

- 32-bit integers

- booleans

- strings

- doubles

- iso8601 dates

- lists

- base64-encoded binary data

- 字典

### 2.4.2 C++ 参数

C++ 有两种 API 访问参数：通过`handle`或`ros::param`下的函数。两种方式等价。

```cpp
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "param_test"); // 初始化节点
    ros::NodeHandle handle; // 句柄

    /* 设置参数 */
    std::vector<std::string> vecString; // 用于演示向量
    vecString.push_back("Ivan");
    vecString.push_back("Anton");
    std::map<std::string, std::string> mapString; // 用于演示字典，即字符串到字符转的映射
    mapString["Ivan"] = "18 years old";
    mapString["Anton"] = "20 years old";
    // 设置参数，第一个为键（参数名），第二个为值
    handle.setParam("param_int", 10); // 整形
    handle.setParam("param_double", 3.141); // 浮点型
    handle.setParam("param_bool", true); // 布尔型
    handle.setParam("param_string", "hello world!"); // 字符串
    handle.setParam("param_vector", vecString); // 向量
    handle.setParam("param_dict", mapString); // 字典
    // 修改参数
    handle.setParam("param_string", "hello world again!"); // 字符串

    /* 获取参数 */
    int param_get = handle.param("param_int", 10); // 若参数存在，返回参数，否则返回默认值
    bool get_result = handle.getParam("param_int", param_get); // 获取参数
    std::vector<std::string> param_list;
    handle.getParamNames(param_list); // 获取全部参数名
    bool has_result = handle.hasParam("param_int");

    /* 删除参数 */
    bool delet_result = handle.deleteParam("param_int");

    return 0;
}
```

## 2.5 消息类型

### 2.5.1 自定义类型字段

srv, msg 中可使用的基本字段类型。和`std_msgs`中类型一致。

- int8, int16, int32, int64 (或者无符号类型: uint*)

- float32, float64

- string

- time, duration

- other msg files

- variable-length array[] and fixed-length array[C]

> 很像 protobuf

在自定义消息时，我们可以导入现有库中已经定义的消息，在编译时需要在 CML 中添加消息构建依赖：

```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  ...
)
```

### 2.5.2 标准消息库 std_msgs

在 package.xml 中添加：

```xml
<build_depend>std_msgs</build_depend>
<build_export_depend>std_msgs</build_export_depend>
<exec_depend>std_msgs</exec_depend>
```

在 CML 中添加：

```cmake
find_package(catkin REQUIRED COMPONENTS # 导包
  ...
  std_msgs
)
```

类型

```shell
std_msgs/Bool
std_msgs/Byte
std_msgs/Char
std_msgs/ColorRGBA
std_msgs/Duration
std_msgs/Empty
std_msgs/Float32 # 以及其他整数、浮点类型
std_msgs/Float32MultiArray
std_msgs/Header
std_msgs/MultiArrayDimension
std_msgs/MultiArrayLayout
std_msgs/String
std_msgs/Time
```

C++ 用法举例

```cpp
/* 标准字符串消息 */
#include <std_msgs/String>
std_msgs::String str_msg;
str_msg.data = "hello world!";
str_msg::Ptr; // 这个是 boost 中的 shared_ptr
```

### 2.5.3 几何消息库 geometry_msgs

`geometry_msgs`

在 package.xml 中添加：

```xml
<build_depend>geometry_msgs</build_depend>
<build_export_depend>geometry_msgs</build_export_depend>
<exec_depend>geometry_msgs</exec_depend>
```

在 CML 中添加：

```cmake
find_package(catkin REQUIRED COMPONENTS # 导包
    ...
    geometry_msgs
)
```

常用类型

```shell
geometry_msgs/Accel
geometry_msgs/Inertia
geometry_msgs/Point
geometry_msgs/Pose
geometry_msgs/Quaternion # 四元数
geometry_msgs/Transform # 变换  quat + v3d
geometry_msgs/TransformStamped # 带头信息的变换
geometry_msgs/Twist # 速度(m/s) + 角速度(rad/s)
geometry_msgs/Vector3 # 三维向量d
geometry_msgs/Wrench
```

C++ 用法举例

```cpp
#include <geometry_msgs/Transform.h>
geometry_msgs::Transform T; // 变换
T.rotation.w; // 四元数d
T.translation.x; // 3d向量
```

> 注意，tf2 中的变换都可以使用 geometry_msgs 中的消息类型实现。

### 2.5.4 传感器消息 sensor_msgs

包含了图像、点云、IMU、激光雷达等多种传感器类型。

如何将 sensor_msgs 消息转换到 OpenCV、PCL 等库中？

# 3 启动

## 3.1 rosrun 命令

设置参数

```shell
rosrun pkg node _param:=100
```

设置命名空间

```shell
resrun pkg node __ns:=/namespace
```

节点名重映射

```shell
resrun pkg node __name:=t2
```

话题重映射

```shell
resrun pkg node /cmd_vel:=/turtle1/cmd_vel
```

## 3.2 launch 文件

```xml
<launch>
    <!-- 启动节点 pkg 包名, type 节点类型, name 节点命名, output 日志 -->
    <node pkg="my_pkg" type="hello" name="hello1" output="screen" />

    <!-- 启动节点同时设置参数 -->
    <node pkg="" type="" name="" output="">
        <!-- 方式1 逐个设置，位于节点命名空间下 -->
        <param name="param_name" value="0" type="int" />
        <!-- 方式2 加载参数文件 -->
        <rosparam command="load" file="$(find package_name)/cfg/param.yaml" />
        <!-- 话题重命名 -->
        <remap from="old" to="new" />
    </node>

    <!-- 导入 launch 文件 -->
    <include file="$(find pkg)/xxx/xxx.launch" ns="ns" />

    <!-- 导入 launch 文件可以传递参数，例如 -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find pkg_name)/worlds/world_name.world" />
    </include>

    <!-- 参数服务器 -->
    <param name="" value="..." type="str|int|double|bool|yaml" />

    <!-- launch 动态传参，命令行调用 roslaunch this.launch xxx:=value -->
    <arg name="xxx" />
    <param name="param" value="$(arg xxx)" />
</launch>
```

# 4 命令行

## 4.1 启动命令

### 4.1.1 roscore

启动 master，朴实无华

### 4.1.2 rosrun

启动节点，见 3.1 节

### 4.1.3 roslaunch

从 launch 文件启动

## 4.2 查询命令

### 4.1.1 rosnode

节点管理

```shell
rosnode ping # 测试到节点的连接状态
rosnode list # 列出活动节点
rosnode info # 打印节点信息
rosnode machine # 列出指定设备上节点
rosnode kill # 杀死某个节点
rosnode cleanup # 清除不可连接的节点
```

### 4.1.2 rostopic

话题管理

```shell
rostopic bw # 显示主题使用的带宽
rostopic delay # 显示带有 header 的主题延迟
rostopic echo # 打印消息到屏幕
rostopic find # 根据类型查找主题
rostopic hz # 显示主题的发布频率
rostopic info # 显示主题相关信息
rostopic list # 显示所有活动状态下的主题
rostopic pub # 将数据发布到主题
rostopic type # 打印主题类型
```

常用：

```shell
# 按 10Hz 发布话题   没有 -r 只发送一次
rostopic pub -r 10 /turtle1/cmd_vel ...
# 获取话题详情
rostopic list -v 
```

### 4.1.3 rosmsg

查询消息类型

```shell
rosmsg show # 显示消息描述
rosmsg info # 显示消息信息
rosmsg list # 列出所有消息
rosmsg md5 # 显示 md5 加密后的消息
rosmsg package # 显示某个功能包下的所有消息
rosmsg packages # 列出包含消息的功能包
```

### 4.1.4 rosservice

服务管理

```shell
rosservice args # 打印服务参数
rosservice call # 使用提供的参数调用服务
rosservice find # 按照服务类型查找服务
rosservice info # 打印有关服务的信息
rosservice list # 列出所有活动的服务
rosservice type # 打印服务类型
rosservice uri # 打印服务的 ROSRPC uri
```

### 4.1.5 rossrv

查询服务类型

```shell
rossrv show # 显示服务消息详情
rossrv info # 显示服务消息相关信息
rossrv list # 列出所有服务信息
rossrv md5 # 显示 md5 加密后的服务消息
rossrv package # 显示某个包下所有服务消息
rossrv packages # 显示包含服务消息的所有包
```

### 4.1.6 rosparam

参数管理

```shell
rosparam set # 设置参数
rosparam get # 获取参数
rosparam load # 从外部文件加载参数
rosparam dump # 将参数写出到外部文件
rosparam delete # 删除参数回头是岸
rosparam list # 列出所有参数
```

常用：

```shell
# 从文件加载参数
rosparam load xxx.yaml
# 导出参数到文件
rosparam dump yyy.yaml
```

# 5 ROS 常用组件

## 5.1 时间相关 API

包含：时刻、持续时间的设置、执行频率、休眠、定时器。

```cpp
#include <ros/ros.h>
```

**时刻**

ROS 中的时刻（时间戳）是距离 1970 年 01 月 01 日 00:00:00 的秒数

```cpp
ros::Time right_now = ros::Time::now(); // 获取当前时刻，
ros::Time someTime(100,100000000); // 100.1s 时刻
```

**时间间隔**

```cpp
ros::Duration du(10); // 持续10秒钟,参数是double类型的，以秒为单位
du.sleep(); // 按照指定的持续时间休眠
ros::Time after_now = now + du1; // 时间与间隔、间隔与间隔可以相加
```

**频率**

```cpp
ros::Rate rate(10); // 指定频率
rate.sleep(); // 休眠 0.1s
```

**定时器**

```cpp
// 回调函数
void doSomeThing(const ros::TimerEvent &event){
    ...
}
// 创建定时器，时间到后执行回调函数
ros::Timer timer = nh.createTimer(ros::Duration(0.5), doSomeThing, false, false); //需要手动启动
timer.start(); // 定时开始
```

## 5.2 坐标变换 TF2

tf2 是很常用的坐标变换包，rviz 中依赖 tf2 显示位置。

依赖项 `tf2 tf2_ros tf2_geometry_msgs roscpp rospy std_msgs geometry_msgs`

tf2 常用功能包：

* `tf2_geometry_msgs`将 ROS geometry_msgs 转为 tf2 消息

* `tf2`封装了坐标变换常用消息

* `tf2_ros`封装了坐标变换常用 C++ 和 Python API

### 5.2.1 静态坐标变换广播器

用于两个坐标系之间位置相对固定的情况。

静态 TF 信息<mark>发布</mark>的示例如下：

```cpp
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(...) {
    ... // 初始化节点
    /* 创建 tf 静态广播器 */
    tf2_ros::StaticTransformBroadcaster broadcaster;
    /* 生成坐标信息 */
    geometry_msgs::TransformStamped ts;
    ts.header.frame_id = "base_link";
    ts.child_frame_id = "laser";
    // 通过 tf2::Quaternion 实现不同姿态表示间的转换
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, 0);
    ts.transform.rotation.w = qtn.getW();
    ... // 设置剩余姿态、位置、时间等信息
    // 发布信息
    broadcaster.sendTransform(ts);

    ros::spin();
    return 0;
}
```

> 注意，以上例程中使用 `geometry_msgs::TransformStamped` 生成消息，但实际上发布的是 `tf2_msgs/TFMessage`。`broadcaster`会帮助我们完成转换，为此必须包含头文件`tf2_geometry_msgs.h`。

### 5.2.2 动态坐标变换广播器

动态坐标变换用于描述机器人关节、移动机器人相对于世界等动态关系。其使用与静态坐标变换相似。

例子如下。这个例程中接受小海龟的位置，转换为 TF 坐标信息并发送。可以在 rviz 中实时观察小海龟的位置信息。

```cpp
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv) {
    ... // 初始化 ROS 节点
    // 创建 TF 动态广播器
    tf2_ros::TransformBroadcaster broadcaster;
    /* 接受海龟位姿消息并转发 使用匿名函数 */
    auto subscriber = handle.subscribe<turtlesim::Pose>("/turtle1/pose", 10,
        [&](const turtlesim::Pose::ConstPtr &pose) -> bool {
            /* 生成消息 */
            geometry_msgs::TransformStamped tfs;
            tfs.header.frame_id = "world"; // 坐标系
            tfs.header.stamp = ros::Time::now();
            tfs.child_frame_id = "turtle1"; // 子坐标系
            tfs.transform.translation.x = pose->x;
            ... // 其他位置坐标
            tf2::Quaternion qtn;
            qtn.setRPY(0, 0, pose->theta);
            tfs.transform.rotation.w = qtn.getW();
            ... // 其他姿态坐标
            broadcaster.sendTransform(tfs);
            return true;
        });
    ros::spin();
    return 0;
}
```

> 坐标之间是否存在过约束？

### 5.2.3 坐标变换监听器

TF 信息接收、点坐标变换的例子如下：

```cpp
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(...) {
    ... // 初始化节点
    /* 创建坐标变换监听节点 */
    // 接受到的坐标消息会存到 buffer 中，包含所有的坐标信息
    // 注意，至少执行一次 ros::spinOnce() 后 buffer 才不为空
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    /* 生成一个坐标点，相对于子坐标系 */
    geometry_msgs::PointStamped point_laser;
    point_laser.header.frame_id = "laser"; // buffer 中必须包含 laser，否则报错
    try { // 提高鲁棒性
        geometry_msgs::PointStamped point_base;
        // 求相对于 base_link 的坐标
        point_base = buffer.transform(point_laser, "base_link");
    }
    catch (const std::exception& e) {
        ROS_ERROR("Error: %s", e.what());
    }
    return 0;
}
```

> TF 坐标变换监听器不区分动态静态。
> 
> `buffer.transform`是一个模板函数，可以进行坐标、位姿等变换。目标坐标系可以为已知的任意坐标系。

命令行发布静态坐标信息

```shell
rosrun tf2_ros static_transform_publisher x y z y p r parent child
```

## 5.3 录制与回放 rosbags

rosbag 用于录制指定话题，记录传感器节点发布的数据，方便在实验后重放。

> 注意，录制原始图像、点云等话题非常占用磁盘空间！要保证预留足够的磁盘，并且有足够大的读写速度。

### 5.3.1 命令行录制与回放

录制

```shell
rosbag record -a -O target.bag
```

> 录制制定话题？

回放

```shell
rosbag play target.bag
```

### 5.3.2 C++ 录制与回放

CML 添加

```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  rosbag
)
```

录制示例。实际编程中，录制应位于话题收听者回调函数中。

```cpp
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) {
    ... // 初始化节点
    rosbag::Bag bag; // 创建 Bag 对象
    bag.open("./test.bag", rosbag::BagMode::Write); // 读取方式打开包文件
    // 生成消息
    std_msgs::String msg;
    msg.data = "hello world";
    // 录制一条消息
    bag.write("/chatter", ros::Time::now(), msg);
    // 关闭
    bag.close();
    return 0;
}
```

读取示例

```cpp
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

int main (int argc, char** argv) {
    ... // 初始化节点
    rosbag::Bag bag; // 创建 Bag 对象
    bag.open("./test.bag", rosbag::BagMode::Read); // 读取方式打开包文件
    // 读数据
    for (rosbag::MessageInstance const m : rosbag::View(bag)) {
        m.getTime(); // 时间戳
        std_msgs::String::ConstPtr p = m.instantiate<std_msgs::String>();
        if (p != nullptr) {
            ROS_INFO("Data: %d %s", p->data.c_str());
        }
    }
    // 关闭
    bag.close();
    return 0;
}
```

# 6 Rviz 可视化

launch 文件启动 Rviz。其中`config.rviz`为 Rviz 配置文件。

```xml
<node pkg="rviz" type="rviz" name="rviz" args="-d $(find package_name)/config/config.rviz" />
```

组件常用

# 7 Gazebo Classical 仿真

ROS Noetic 默认安装 Gazebo Classical 11

依赖 `urdf xacro gazebo_ros gazebo_ros_control gazebo_plugins`

## 7.0 仿真运行

launch 文件

```xml
<launch>
    <!-- 将 urdf 文件的内容加载到参数服务器 -->
    <param name="robot_description" textfile="$(find pkg_name)/urdf/robot.urdf" />

    <!-- 启动 gazebo，空世界 -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch" />

    <!-- 导入自定义世界 -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find pkg_name)/world/world_name.sdf" />
    </include>

    <!-- 在 gazebo 中生成机器人模型 -->
    <node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf -model robot -param robot_description -x 0.0 -y 0.0 -z 0.0" />
</launch>
```

> 注意，自己搭建场景时，要 sudo 运行 gazebo，否则场景无法保存。

## 7.1 URDF 机器人模型

URDF (Unified Robot Description Format) 为机器人描述格式，用作描述机器人。URDF 中的机器人由连杆(link)和关节(joint)组成，需要额外添加各种插件(plugin)作为驱动器或传感器。

> URDF 文件中用英文做注释，不要写其他语言的字符，会报错

URDF 文件结构如下

```xml
<robot name="mycar">
    <!-- 连杆 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
        </visual>
        <collision>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
        </collision>
        <inertial>
            <origin xyz="0 0 0" />
            <mass value="6" />
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1" />
        </inertial>
    </link>
    <gazebo reference="base_link">
        <material>Gazebo/Black</material>
    </gazebo>
    ...
    <!-- 关节 -->
    <joint name="camera2baselink" type="continuous">
        <parent link="base_link"/>
        <child link="camera" />
        <origin xyz="0.2 0 0.075" rpy="0 0 0" />
        <axis xyz="0 0 1" />
    </joint>
    ...

</robot>
```

### 7.1.1 连杆 link

连杆标签内可以设计形状、尺寸、位姿、颜色、惯性矩阵、碰撞参数等一系列属性。例子如下

```xml
<link name="base_link">
    <!-- 可视化 -->
    <visual>
        <!-- 形状 -->
        <geometry>
            <box size="0.5 0.3 0.1" />
        </geometry>
        <!-- 位姿 -->
        <origin xyz="0 0 0" rpy="0 0 0" />
        <!-- 颜色 -->
        <material name="black">
            <color rgba="0.7 0.5 0 0.5" />
        </material>
    </visual>
    <!-- 碰撞 -->
    <collision>
        <geometry>
            <box size="0.5 0.2 0.1" />
        </geometry>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
    </collision>
    <!-- 惯性 -->
    <inertial>
        <origin xyz="0 0 0" />
        <mass value="6" />
        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1" />
    </inertial>
</link>
<!-- gz 颜色设置 -->
<gazebo reference="base_link">
     <material>Gazebo/Blue</material>
</gazebo>
```

其中 `geometry` 可以设置为标准形状或 dae 网格。

```xml
<!-- 长方体 长 宽 高 -->
<box size="0.5 0.3 0.1" />
<!-- 圆柱 半径 长 -->
<cylinder radius="0.5" length="0.1" />
<!-- 球体 半径-->
<sphere radius="0.3" />
<!-- 网格 文件路径 -->
<mesh />
```

### 7.1.2 关节 joint

例子如下

```xml
<joint name="camera2baselink" type="continuous">
    <!-- 父连杆 -->
    <parent link="base_link"/>
    <!-- 子连杆 -->
    <child link="camera" />
    <!-- 相对位姿 -->
    <origin xyz="0.2 0 0.075" rpy="0 0 0" />
    <!-- 运动轴 -->
    <axis xyz="0 0 1" />
    <!-- 位置极限 -->
    <limit effort="3" lower="0" upper="0.040" velocity="0.05"/>
    <!-- 动力学参数 -->
    <dynamics damping="0.99"/>
</joint>
```

其中 `type` 可选为如下几种

* continuous: 旋转关节，可以绕单轴无限旋转

* revolute: 旋转关节，类似于 continues，但是有旋转角度限制

* prismatic: 滑动关节，沿某一轴线移动的关节，有位置极限

* planer: 平面关节，允许在平面正交方向上平移或旋转

* floating: 浮动关节，允许进行平移、旋转运动

* fixed: 固定关节，不允许运动的特殊关节

### 7.1.3 插件 plugin

### 7.1.4 URDF 显示与模型生成

依赖包：`urdf` 与 `xacro`

**显示机器人**。使用 `robot_description` 将 urdf 文件上传到参数服务器，之后可在 Rviz 中显示模型、在 gz 中生成机器人。

**发布关节状态**。使用`joint_state_publisher`或`joint_state_publisher_gui`发布关节状态，便于在Rviz中显示，只需要两个节点中的一个。发布的信息是 TF。

修改 launch 文件

```xml
<!-- 上传机器人到参数服务器 -->
<param name="robot_description" textfile="$(find package_name)/urdf/robot.urdf" />
<!-- 关节状态发布 -->
<node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
<!-- GUI 控制关节状态发布 -->
<node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />
```

### 7.1.5 URDF 工具

下载

```shell
sudo apt install liburdfdom-tools
```

URDF 语法检查

```shell
check_urdf robot.urdf
```

按机器人结构生成 PDF 文件

```shell
urdf_to_graphiz robot.urdf
```

## 7.2 xacro 机器人模型

依赖包：`urdf` 与 `xacro`

### 7.2.1 xacro 语法

xacro 文件的一般格式：

```xml
<robot name="mycar" xmlns:xacro="http://wiki.ros.org/xacro">
    <!-- 属性，类似于全局变量 -->
    <xacro:property name="wheel_radius" value="0.0325" />
    <!-- 宏，类似于函数 -->
    <xacro:macro name="wheel_func" params="wheel_name flag" >
        <link name="${wheel_name}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_radius + 10}" />
                </geometry>
            </visual>
        </link>
    </xacro:macro>
    <!-- 文件包含 -->
    <xacro:include filename="my_base.xacro" />
</robot>
```

使用`${property_name}`调用属性

使用`${expression}`计算数学表达式

### 7.2.2 xacro 使用

xacro 转 urdf 命令

```shell
rosrun xacro xacro robot.xacro > robot.urdf
```

launch 文件

```xml
<param name="robot_description" command="$(find xacro)/xacro $(find pkg_name)/urdf/xacro/robot.xacro" />
```

## 7.3 传感器插件 sensor-plugin

### 7.3.1 单线激光雷达 laser

```xml
<gazebo reference="laser">
    <sensor type="ray" name="rplidar">
        <pose>0 0 0 0 0 0</pose>
        <visualize>true</visualize>
        <update_rate>5.5</update_rate>
        <ray>
            <scan>
                <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3</min_angle>
                <max_angle>3</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.10</min>
                <max>30.0</max>
                <resolution>0.01</resolution>
            </range>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.01</stddev>
            </noise>
        </ray>
        <plugin name="gazebo_rplidar" filename="libgazebo_ros_laser.so">
            <topicName>/scan</topicName>
            <frameName>laser</frameName>
        </plugin>
    </sensor>
</gazebo>
```

### 7.3.2 单目相机 camera

```xml
<!-- 被引用的link -->
<gazebo reference="camera">
    <!-- 类型设置为 camara -->
    <sensor type="camera" name="camera_node">
        <update_rate>30.0</update_rate> <!-- 更新频率 -->
        <!-- 摄像头基本信息设置 -->
        <camera name="head">
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
                <width>1280</width>
                <height>720</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.02</near>
                <far>300</far>
            </clip>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.007</stddev>
            </noise>
        </camera>
        <!-- 核心插件 -->
        <plugin name="gazebo_camera" filename="libgazebo_ros_camera.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>0.0</updateRate>
            <cameraName>/camera</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>camera</frameName>
            <hackBaseline>0.07</hackBaseline>
            <distortionK1>0.0</distortionK1>
            <distortionK2>0.0</distortionK2>
            <distortionK3>0.0</distortionK3>
            <distortionT1>0.0</distortionT1>
            <distortionT2>0.0</distortionT2>
        </plugin>
    </sensor>
</gazebo>
```

### 7.3.3 深度相机 kinect

```xml
<gazebo reference="kinect_link">  
    <sensor type="depth" name="camera">
        <always_on>true</always_on>
        <update_rate>20.0</update_rate>
        <camera>
            <horizontal_fov>${60.0*3.14159/180.0}</horizontal_fov>
            <image>
            <format>R8G8B8</format>
            <width>640</width>
            <height>480</height>
            </image>
            <clip>
            <near>0.05</near>
            <far>8.0</far>
            </clip>
        </camera>
        <plugin name="kinect_camera_controller" filename="libgazebo_ros_openni_kinect.so">
            <cameraName>camera</cameraName>
            <alwaysOn>true</alwaysOn>
            <updateRate>10</updateRate>
            <imageTopicName>rgb/image_raw</imageTopicName>
            <depthImageTopicName>depth/image_raw</depthImageTopicName>
            <pointCloudTopicName>depth/points</pointCloudTopicName>
            <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
            <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
            <frameName>support_depth</frameName> <!-- 点云坐标系-->
            <baseline>0.1</baseline>
            <distortion_k1>0.0</distortion_k1>
            <distortion_k2>0.0</distortion_k2>
            <distortion_k3>0.0</distortion_k3>
            <distortion_t1>0.0</distortion_t1>
            <distortion_t2>0.0</distortion_t2>
            <pointCloudCutoff>0.4</pointCloudCutoff>
        </plugin>
    </sensor>
</gazebo>
```

注意，kinect 图像数据与点云数据使用两套不同的坐标系，需要进行一次变换，才能让点云正常显示。

launch 文件中添加如下内容：

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="static_transform_publisher" args="0 0 0 -1.57 0 -1.57 /support /support_depth" />
```

> 注意，颜色通道顺序好像是 BGR

### 7.3.4 双目相机 stereo

1

## 7.4 驱动器插件 driver-plugin

ROS 设计了控制器接口 <mark>ros_control</mark>，是一套中间件，gazebo 实现了 ros_control 的接口。

### 7.4.1 减速器 transmission

```xml
<transmission name="joint_name_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_name">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="joint_name_motor">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction> <!-- 减速比 -->
    </actuator>
</transmission>
```

### 7.4.2 差分控制器

```xml
<gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
        <rosDebugLevel>Debug</rosDebugLevel>
        <publishWheelTF>true</publishWheelTF>
        <robotNamespace>/</robotNamespace>
        <publishTf>1</publishTf>
        <publishWheelJointState>true</publishWheelJointState>
        <alwaysOn>true</alwaysOn>
        <updateRate>100.0</updateRate>
        <legacyMode>true</legacyMode>
        <leftJoint>left_wheel2base_link</leftJoint> <!-- 左轮 -->
        <rightJoint>right_wheel2base_link</rightJoint> <!-- 右轮 -->
        <wheelSeparation>${base_link_radius * 2}</wheelSeparation> <!-- 车轮间距 -->
        <wheelDiameter>${wheel_radius * 2}</wheelDiameter> <!-- 车轮直径 -->
        <broadcastTF>1</broadcastTF>
        <wheelTorque>30</wheelTorque>
        <wheelAcceleration>1.8</wheelAcceleration>
        <commandTopic>cmd_vel</commandTopic> <!-- 运动控制话题 -->
        <odometryFrame>odom</odometryFrame> 
        <odometryTopic>odom</odometryTopic> <!-- 里程计话题 -->
        <robotBaseFrame>base_footprint</robotBaseFrame> <!-- 根坐标系 -->
    </plugin>
</gazebo>
```

## 7.5 SDF 世界与机器人模型

见 gazeboGarden 笔记

# 8 平面机器人导航

平面差速驱动小车+单线激光雷达导航标准方法。

导航五要素：

1. 全局地图 -> 先验地图、SLAM
2. 定位 -> **amcl** 包（Adaptive MonteCarlo Localization，自适应蒙特卡罗定位）、SLAM
3. 路径规划 -> **move_base** 包
4. 运动控制 -> cmd_vel 话题
5. 环境感知 -> 传感器插件

在构建导航功能时，需要提前配置好**运动控制**和**环境感知**模块。

> 注意，导航包默认机器人可以订阅`/cmd_vel`话题改变 x, y, yaw 速度分量。即 x, y 方向线速度，偏航角速度。

坐标系关系： `map` -> `doom` -> `base_link` 

需要为机器人安装一个单线激光雷达。

> 这个可以用点云转激光雷达实现。

`gmapping` —— SLAM 包
`map-server` —— 地图服务
`navigation` —— 导航包，包括 amcl, move_base 等功能包

```shell
sudo apt install ros-noetic-gmapping
sudo apt install ros-noetic-map-server
sudo apt install ros-noetic-navigation
```

功能包依赖：`gmapping map_server amcl move_base`

## 8.1 2D LSLAM gmapping

> [gmapping ROS WIKI](http://wiki.ros.org/gmapping)

根据里程计信息和激光雷达数据绘制二维栅格地图。

```shell
sudo apt install ros-noetic-gmapping
```

<mark>输入</mark>

1. **tf** (tf/tfMessage) 雷达、底盘、里程计之间的坐标变换消息

2. **scan** (sensor_msgs/LaserScan) 单线激光雷达消息

<mark>输出</mark>

1. **map** (nav_msgs/OccupancyGrid) 地图栅格数据

2. **tf** 地图坐标系与里程坐标系之间的坐标变换

3. **map_metadata** (nav_msgs/MapMetaData) 地图参数，宽度、高度、分辨率等

4. **entropy** (std_msgs/Float64) 机器人姿态分布熵估计

使用示例，修改 launch 文件

```xml
<launch>
    <param name="use_sim_time" value="true"/>
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
      <remap from="scan" to="scan"/>
      <param name="base_frame" value="base_footprint"/> <!--底盘坐标系-->
      <param name="odom_frame" value="odom"/> <!--里程计坐标系-->
      <param name="map_update_interval" value="5.0"/>
      <param name="maxUrange" value="16.0"/>
      <param name="sigma" value="0.05"/>
      <param name="kernelSize" value="1"/>
      <param name="lstep" value="0.05"/>
      <param name="astep" value="0.05"/>
      <param name="iterations" value="5"/>
      <param name="lsigma" value="0.075"/>
      <param name="ogain" value="3.0"/>
      <param name="lskip" value="0"/>
      <param name="srr" value="0.1"/>
      <param name="srt" value="0.2"/>
      <param name="str" value="0.1"/>
      <param name="stt" value="0.2"/>
      <param name="linearUpdate" value="1.0"/>
      <param name="angularUpdate" value="0.5"/>
      <param name="temporalUpdate" value="3.0"/>
      <param name="resampleThreshold" value="0.5"/>
      <param name="particles" value="30"/>
      <param name="xmin" value="-50.0"/>
      <param name="ymin" value="-50.0"/>
      <param name="xmax" value="50.0"/>
      <param name="ymax" value="50.0"/>
      <param name="delta" value="0.05"/>
      <param name="llsamplerange" value="0.01"/>
      <param name="llsamplestep" value="0.01"/>
      <param name="lasamplerange" value="0.005"/>
      <param name="lasamplestep" value="0.005"/>
    </node>
</launch>
```

## 8.2 地图服务 map_server

用于栅格地图的序列化与反序列化。`map_saver`将地图保存到磁盘，`map_server`从磁盘读取地图。

```shell
sudo apt install ros-noetic-map-server
```

**保存地图**

<mark>输入</mark>： map (nav_msgs/OccupancyGrid) 地图栅格数据

launch 文件

```xml
<launch>
    <arg name="filename" value="$(find mycar_nav)/map/map_name" />
    <node name="map_saver" pkg="map_server" type="map_saver" args="-f $(arg filename)" />
</launch>
```

或命令行

```shell
rosrun map_server map_saver -f map_name
```

> 生成 map_name.yaml 中 resolution 单位为 米/像素。
> 两个阈值为认为像素空或被占有概率的阈值。

**加载地图**

<mark>输出</mark>： map (nav_msgs/OccupancyGrid) 地图栅格数据

> 注意：加载的地图和 SLAM 的地图、里程计话题名可能会冲突，可以通过重映射解决

launch 文件

```xml
<launch>
    <arg name="map" default="map_name.yaml" />
    <node name="map_server" pkg="map_server" type="map_server" args="$(find pkg_name)/map/$(arg map)"/>
</launch>
```

或命令行

```shell
rosrun map_server map_server map_name.yaml
```

## 8.3 2D定位 amcl

> [ROS AMCL WIKI](http://wiki.ros.org/amcl)

AMCL(adaptive Monte Carlo Localization) 是用于2D移动机器人的概率定位系统，它实现了自适应（或KLD采样）蒙特卡洛定位方法，可以根据已有地图使用粒子滤波器推算机器人位置。

```shell
sudo apt install ros-noetic-navigation
```

<mark>输入</mark>

1. **scan** (sensor_msgs/LaserScan) 激光雷达数据

2. **tf** (tf/tfMessage) 坐标变换

3. **initialpose** (geometry_msgs/PoseWithCovarianceStamped) 初始化粒子滤波器的均值和协方差

4. **map** (nav_msgs/OccupancyGrid) 地图

<mark>输出</mark>

1. **amcl_pose** (geometry_msgs/PoseWithCovarianceStamped) 机器人位姿估计

2. **particlecloud** (geometry_msgs/PoseArray) 位姿估计集合

3. **tf** 发布从 odom 到 map 的转换

> 注意：导航 launch 文件和启动仿真 launch 文件写成两个。先运行仿真，再运行导航。否则会报错。

使用示例

```xml
<launch>
    <node pkg="amcl" type="amcl" name="amcl" output="screen">
        <!-- Publish scans from best pose at a max of 10 Hz -->
        <param name="odom_model_type" value="diff"/> <!-- 里程计模式为差分，也可以是全向 -->
        <param name="odom_alpha5" value="0.1"/>
        <param name="transform_tolerance" value="0.2" />
        <param name="gui_publish_rate" value="10.0"/>
        <param name="laser_max_beams" value="30"/>
        <param name="min_particles" value="500"/>
        <param name="max_particles" value="5000"/>
        <param name="kld_err" value="0.05"/>
        <param name="kld_z" value="0.99"/>
        <param name="odom_alpha1" value="0.2"/>
        <param name="odom_alpha2" value="0.2"/>
        <!-- translation std dev, m -->
        <param name="odom_alpha3" value="0.8"/>
        <param name="odom_alpha4" value="0.2"/>
        <param name="laser_z_hit" value="0.5"/>
        <param name="laser_z_short" value="0.05"/>
        <param name="laser_z_max" value="0.05"/>
        <param name="laser_z_rand" value="0.5"/>
        <param name="laser_sigma_hit" value="0.2"/>
        <param name="laser_lambda_short" value="0.1"/>
        <param name="laser_lambda_short" value="0.1"/>
        <param name="laser_model_type" value="likelihood_field"/>
        <!-- <param name="laser_model_type" value="beam"/> -->
        <param name="laser_likelihood_max_dist" value="2.0"/>
        <param name="update_min_d" value="0.2"/>
        <param name="update_min_a" value="0.5"/>

        <param name="odom_frame_id" value="odom"/> <!-- 里程计坐标系 -->
        <param name="base_frame_id" value="base_footprint"/> <!-- 机器人基坐标系 -->
        <param name="global_frame_id" value="map"/> <!-- 地图坐标系 -->

        <param name="resample_interval" value="1"/>
        <param name="transform_tolerance" value="0.1"/>
        <param name="recovery_alpha_slow" value="0.0"/>
        <param name="recovery_alpha_fast" value="0.0"/>
    </node>
</launch>
```

## 8.4 导航 move_base

move_base 提供基于 Action 的路径规划实现，可以根据给定的目标点，控制机器人移动到目标位置。路径规划由两部分组成：全局路径规划、局部路径规划。

```shell
sudo apt install ros-noetic-navigation
```

<mark>输入</mark>

1. 动作 **move_base/goal** (move_base_msgs/MoveBaseActionGoal) 运动规划目标点

2. **move_base_simple/goal** (geometry_msgs/PoseStamped) 运动规划目标点，话题实现

<mark>输出</mark>

1. **cmd_vel** (geometry_msgs/Twist) 底盘运动控制

move_base 使用时需要编辑 `launch`, `costmap_common_params.yaml`, `local_costmap_params.yaml`, `global_costmap_params.yaml`, `base_local_planner_params.yaml` 文件。

### 8.4.1 launch 文件

```xml
<launch>
    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen" clear_params="true">
        <rosparam file="$(find pkg_name)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find pkg_name)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find pkg_name)/param/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find pkg_name)/param/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find pkg_name)/param/base_local_planner_params.yaml" command="load" />
    </node>
</launch>
```

### 8.4.2 costmap_common_params.yaml 配置文件

在全局路径规划与本地路径规划时调用的通用参数，包括:机器人的尺寸、距离障碍物的安全距离、传感器信息等

```yaml
#机器人几何参，如果机器人是圆形，设置 robot_radius,如果是其他形状设置 footprint
robot_radius: 0.12 # 圆形
# footprint: [[-0.12, -0.12], [-0.12, 0.12], [0.12, 0.12], [0.12, -0.12]] #其他形状

obstacle_range: 3.0 # 用于障碍物探测，比如: 值为 3.0，意味着检测到距离小于 3 米的障碍物时，就会引入代价地图
raytrace_range: 3.5 # 用于清除障碍物，比如：值为 3.5，意味着清除代价地图中 3.5 米以外的障碍物

#膨胀半径，扩展在碰撞区域以外的代价区域，使得机器人规划路径避开障碍物
inflation_radius: 0.2
#代价比例系数，越大则代价值越小
cost_scaling_factor: 3.0

#地图类型
map_type: costmap
#导航包所需要的传感器
observation_sources: scan
#对传感器的坐标系和数据进行配置。这个也会用于代价地图添加和清除障碍物。例如，可以用激光雷达传感器用于在代价地图添加障碍物，再添加kinect用于导航和清除障碍物。
scan: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true}
```

### 8.4.3 global_costmap_params.yaml 配置文件

用于全局代价地图参数设置

```yaml
global_costmap:
  global_frame: map #地图坐标系
  robot_base_frame: base_footprint #机器人坐标系
  # 以此实现坐标变换

  update_frequency: 1.0 #代价地图更新频率
  publish_frequency: 1.0 #代价地图的发布频率
  transform_tolerance: 0.5 #等待坐标变换发布信息的超时时间

  static_map: true # 是否使用一个地图或者地图服务器来初始化全局代价地图，如果不使用静态地图，这个参数为false.
```

### 8.4.4 local_costmap_params.yaml 配置文件

用于局部代价地图参数设置

```yaml
local_costmap:
  global_frame: odom #里程计坐标系
  robot_base_frame: base_footprint #机器人坐标系

  update_frequency: 10.0 #代价地图更新频率
  publish_frequency: 10.0 #代价地图的发布频率
  transform_tolerance: 0.5 #等待坐标变换发布信息的超时时间

  static_map: false  #不需要静态地图，可以提升导航效果
  rolling_window: true #是否使用动态窗口，默认为false，在静态的全局地图中，地图不会变化
  width: 3 # 局部地图宽度 单位是 m
  height: 3 # 局部地图高度 单位是 m
  resolution: 0.05 # 局部地图分辨率 单位是 m，一般与静态地图分辨率保持一致
```

### 8.4.5 base_local_planner_params.yaml 配置文件

局部规划器参数配置，这个配置文件设定了机器人的最大和最小速度限制值，也设定了加速度的阈值

```yaml
TrajectoryPlannerROS:
  max_vel_x: 0.5 # X 方向最大速度
  min_vel_x: 0.1 # X 方向最小速速

  max_vel_theta:  1.0 # 
  min_vel_theta: -1.0
  min_in_place_vel_theta: 1.0

  acc_lim_x: 1.0 # X 加速限制
  acc_lim_y: 0.0 # Y 加速限制
  acc_lim_theta: 0.6 # 角速度加速限制

  # Goal Tolerance Parameters，目标公差
  xy_goal_tolerance: 0.10
  yaw_goal_tolerance: 0.05

  # 是否是全向移动机器人
  holonomic_robot: false

  # Forward Simulation Parameters，前进模拟参数
  sim_time: 0.8
  vx_samples: 18
  vtheta_samples: 20
  sim_granularity: 0.05
```

# 9 机械臂抓取
