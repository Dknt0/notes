# ROS Noetic 编程

> 好的 cpp 绘图方法？
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
├── CMakeLists.txt # 
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
        ├── config # gz rviz 等配置
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

编译后，消息头文件位于 catkin_ws/devel/include/my_package/Info.h

### 2.1.3 C++ 话题

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
        [](const my_pkg::Info::ConstPtr& msg_p) -> bool {
            ROS_INFO("Received: %s %d", msg_p->name.c_str(), msg_p->age);
            return true;
        });
    ros::spin(); // 循环
    return 0;
}
```

**CML**

额外添加依赖

```cmake
add_dependencies(node_name ${PROJECT_NAME}_generate_messages_cpp)
```

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

### 2.2.2 C++ 服务

Server C++

```cpp
#include <ros/ros.h>
#include <my_pkg/Req.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "server"); // 初始化节点
    ros::NodeHandle handle; // 句柄
    // 实例化服务端，可以传入函数指针、仿函数、匿名函数
    // 使用匿名函数要显式给出模板参数，自动推导会调boost库，报错
    ros::ServiceServer server = handle.advertiseService<my_pkg::Req::Request, my_pkg::Req::Response>("test_service",
        [](my_pkg::Req::Request &req, my_pkg::Req::Response &resp) -> bool {
            resp.sum = req.num1 + req.num2;
            ROS_INFO("I think %d + %d = %d", req.num1, req.num2, resp.sum);
            return true;
        });
    ros::spin(); // 循环
    return 0;
}
```

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

CML 添加额外依赖

```cmake
add_dependencies(node_name ${PROJECT_NAME}_gencpp)
```

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

srv, msg 中可使用的字段类型。和`std_msgs`中类型一致。

- int8, int16, int32, int64 (或者无符号类型: uint*)

- float32, float64

- string

- time, duration

- other msg files

- variable-length array[] and fixed-length array[C]

> 很像 protobuf

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

`tf`

图像

点云

激光雷达

IMU等

# 3 启动

```xml
<launch>
    <!-- 启动节点 pkg 包名, type 节点类型, name 节点命名, output 日志 -->
    <node pkg="my_pkg" type="hello" name="hello1" output="screen" />


</launch>
```

# 4 命令行

## 4.1 启动命令

### 4.1.1 roscore

启动 master ， 朴实无华

### 4.1.2 rosrun

启动节点

话题重定向，重命名

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

# 仿真

# 功能包
