# ROS 2 Humble

> ROS 2 Humble 笔记
> 
> 参考：
> 
> https://docs.ros.org/en/humble/
> 
> Dknt 2023.6

ROS 2 Humble 是 ROS 2 的第一个 LTS 发行版，运行在 Ubnutu 22.04 系统上。ROS 2 与 ROS 1 最大的区别在于底层通信协议的改变 —— DDS。ROS 2 是实时系统。可以运行在裸机上。跨平台。

ROS 2 Humble 也可以通过源码编译运行在其他系统上。

> 先大概记录下内容，再分章节总结笔记

# 1 命令行

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

* 包管理

创建新功能包

```shell
ros2 pkg create <package_name> --build-type ament_cmake [--dependencies rclcpp ...]
```

# 启动

ROS 2 的启动文件有三种，XML、Python、YAML。Python 最为方便。

Python 启动文件位于 package/launch 目录下

1

示例：

```python

```

修改 CMakeLists.txt 文件

```cmake

```

启动命令：

```shell

```

# 依赖管理

rosdep

1

# _1 开发

ROS 2 编程与 ROS 1 有很大区别（C++）。用到了一些高级语法特性。

ROS 2 的一个进程中可以运行多个节点。在 C++ 中，节点被抽象成一个类，我们可以继承`rclcpp::Node`类，来建立自己的节点，实现自己的功能。

在节点基类中实现了创建发布者、服务器等功能，通过`this->create_*()`来创建，不需要“句柄”。这些函数返回一个共享指针，因此在我们的节点类需要创建一个对应话题/服务类型的共享指针。

回调函数可以写为节点类的成员函数，注册时需要使用`std::bind`进行适配，详见例程。

ROS 2 C++ 编程中大量使用了共享指针`std::shared_ptr<T>`。

在 ROS 2 源码中提供了多种通信的编程实现方式，可以根据实际情况选用最合适的。

## _1.1 colcon 工具

`colcon`是 `catkin`, `ament` 等 ROS 编译工具的集成。

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

编译需要在工作空间**根目录**下进行：

```shell
colcon build --symlink-install
colcon build --packages-select package_name # 编译某个包
```

`--symlink-install`将 python 文件移动到 install 目录下。

`setup.bash`位于`install`目录下，执行这个脚本，会把包中所有可执行文件和库文件添加到目录中。执行`setup.bash`会将当前包以及他的以来包添加到环境，`local_setup.bash`只将当前包添加到环境。

`package.xml`文件用于定义一个包。

使用如下命令按照模板创建一个包。

```shell
ros2 pkg create <package_name>
```

`colcon_cd`命令用于跳转到指定包目录，需要预先将这个命令加入到环境变量中。

```shell
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

## _1.2 工作空间与 ROS 包

如果在包目录下有一个叫`COLCON_IGNORE`的空文件，那么这个包不会被 colcon 编译。

在工作空间根目录下输入如下指令，检查依赖

```shell
rosdep install -i --from-path src --rosdistro humble -y
```

ROS 2 包分为两类：CMake 和 Python。ROS 2 包使用 ament 作为编译系统，使用 colcon 作为编译工具。

### _1.2.1 CMake 包

用于管理 C++ 项目。

CMake 包最小要求：

```shell
CMakeLists.txt # CMake 编译文件
include/<package_name>/ # 头文件
package.xml # 包元信息
src/ # 源文件
```

创建 CMake 包的指令：

```shell
ros2 pkg create --build-type ament_cmake <package_name>
```

创建包同时创建节点源文件：

```shell
ros2 pkg create --build-type ament_cmake --node-name my_node <package_name>
```

创建包同时添加依赖：

```shell
ros2 pkg create --build-type ament_cmake <package_name> --dependencies rclcpp std_msgs
```

基于 ament 的 CMake 总结：

1

### _1.2.2 Python 包

用于管理 Python 项目

```shell
ros2 pkg create --build-type ament-python <package_name>
```

## _1.3 话题 Topic

ROS 2 C++ 中的节点为`rclcpp::Node`的派生类，这种方式可以让一个进程中运行多个节点（由此，进程与节点不再等价）。ROS 2 也支持与 ROS 1 类似的编程方式，这时一个进程之能运行一个节点。推荐使用新方法。

### _1.3.1 自定义话题

话题定义文件存放在 ROS 包 msg 目录下。话题可以导入其他 underlay 中已经定义的话题。

msg/Sphere.msg 文件

```shell
geometry_msgs/Point center
float64 radius
```

修改 package.xml

```xml
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

修改 CMakeLists.txt

```cmake
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# 生成消息库
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sphere.msg"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```

按官方文档所述，话题/服务定义必须使用 cmake 编译，且最好放在单独的包中。但也可以在同一个包中定义并使用，这时需要在 CMakeLists.txt 中添加如下指令：

```cmake
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(target_name "${cpp_typesupport_target}")
```

自定义话题可以使用 underlay 中定义过的话题，也可以使用这些话题的数组。为此，必须在 package.xml 中添加依赖，在 CMakeLists.txt 生成话题库时添加依赖，编写代码时加入 underlay 的头文件。详见 https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html

### _1.3.2 C++ Publisher

源码 publisher_member_function.cpp

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// chrono 神奇的运算符重载，可以直接写 500ms 这样的字面值
using namespace std::chrono_literals;
// ROS 2 中的节点需要继承自节点基类
class MinimalPublisher : public rclcpp::Node {
public:
    // 构造函数  列表初始化 : Node(节点名)
    MinimalPublisher() : Node("minimal_publisher"), conut_(0) {
        // 话题发布者，第一个参数为话题名，第二个参数是 QoS，这里代表 buffer 大小
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        // 初始化定时器回调函数 这里用了 std::bind
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
private:
    // 回调函数
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(conut_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    // 定时器 共享指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 发布者 共享指针
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // 计数器
    size_t conut_;
};

int main(int argc, char **argv) {
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 开始处理节点数据 循环处理
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    // 终止进程
    rclcpp::shutdown();
    return 0;
}
```

修改 package.xml

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

修改 CMakeLists.txt

```cmake
# 导包
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# 添加可执行文件
add_executable(talker src/publisher_member_function.cpp)
# ament 链接配置
ament_target_dependencies(talker rclcpp std_msgs)
# 下载位置
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME}
)
```

> 注意：存在其他编程方式用 C++ 实现话题通信，详见源码

### _1.3.3 C++ Subscriber

源码 subscriber_member_function.cpp

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// std::bind 占位符
using std::placeholders::_1;
// 节点派生类
class MinimalSubscriber : public rclcpp::Node {
public:
    // 构造函数 Node(节点名)
    MinimalSubscriber() : Node("minimal_subscriber") {
        // "话题名"  QoS 为 buffer 长度
        subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10,
            std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }
private:
    // 回调函数，注意参数类型
    void topic_callback(const std_msgs::msg::String &msg) const {
        RCLCPP_INFO(this->get_logger(), "I heared: '%s'", msg.data.c_str());
    }
    // 收听者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    // 处理节点信息
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    // 终止进程
    rclcpp::shutdown();
    return 0;
}
```

修改 package.xml

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

修改 CMakeLists.txt

```cmake
# 导包
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# 添加可执行文件
add_executable(listener src/subscriber_member_function.cpp)
# ament 链接配置
ament_target_dependencies(listener rclcpp std_msgs)
# 下载位置
install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME}
)
```

### _1.3.4 Python P & S

Python 中模块导入的问题。

Python 默认会从当前执行文件目录下搜索导入的包，但 ROS 2 在安转时，会将节点代码拷贝到 install 目录下，造成包导入报错。可以通过添加路径的方式解决，在代码前添加如下两行：

```python
import sys
sys.path.append("/home/dknt/Projects/uav_sim/src/yolo_detector/yolo_detector")
import detect
```

## _1.4 服务 Service

1

### _1.4.1 自定义服务

服务定义文件存放在 ROS 包 srv 目录下

srv/AddThreeInts.srv 文件

```shell
int64 a
int64 b
int64 c
---
int64 sum
```

修改 package.xml

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

修改 CMakeLists.txt

```cmake
find_package(rosidl_default_generators REQUIRED)
# 生成服务库
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddThreeInts.srv"
  DEPENDENCIES # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```

类似于话题，在同一个包中定义和使用服务，需要在 CMakeLists.txt 中添加如下命令：

```cmake
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(target_name "${cpp_typesupport_target}")
```

### _1.4.2 C++ Server

源码 add_two_ints_server.cpp

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

// 回调函数 注意参数格式
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld", request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv) {
    // 初始化 ROS
    rclcpp::init(argc, argv);
    // 实例化节点，参数为节点名  共享指针
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");
    // 创建服务器  共享指针
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
        node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
    // 处理数据
    rclcpp::spin(node);
    // 终止进程
    rclcpp::shutdown();
    return 0;
}
```

通过`node`对象创建的服务器会与他绑定，当我们将`node`加入`spin`时，会同时启动服务。

修改 package.xml

```xml
<depend>rclcpp</depend>
<depend>example_interfaces</depend>
```

修改 CMakeLists.txt

```cmake
# 导包
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
# 添加可执行文件
add_executable(server src/add_two_ints_server.cpp)
# ament 链接配置
ament_target_dependencies(server rclcpp example_interfaces)
# 下载位置
install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME}
)
```

### _1.4.3 C++ Client

源码 add_two_ints_server.cpp

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

// 神奇的 chrono 字面值
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    // 初始化 ROS
    rclcpp::init(argc, argv);
    // 参数数量错误
    if (argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
        return 1;
    }
    // 节点  共享指针
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
    // 客户端  共享指针
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
        node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    // 创建请求对象，设置请求值
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);
    // 等待服务
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not avaible, waiting again...");
    }
    // 申请服务，异步方式获取结果
    auto result = client->async_send_request(request);
    // 等待服务器返回结果  异步并发
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }
    // 终止进程
    rclcpp::shutdown();
    return 0;
}
```

> 客户端调用服务时使用异步方式，结果从`future`中获取。这里没有定义一个新的节点类。

修改 package.xml

```xml
<depend>rclcpp</depend>
<depend>example_interfaces</depend>
```

修改 CMakeLists.txt

```cmake
# 导包
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
# 添加可执行文件
add_executable(client src/add_two_ints_client.cpp)
# ament 链接配置
ament_target_dependencies(client rclcpp example_interfaces)
# 下载位置
install(TARGETS
  client
  DESTINATION lib/${PROJECT_NAME}
)
```

### _1.4.4 Python S & C

1

## _1.5 参数 Parameter

ROS 2 中参数属于节点，不存在参数服务器。

节点可以在程序中设置，也可以在启动文件中设置。

### _1.5.1 C++ 参数配置

ROS 2 首先要在节点代码中声明参数，之后才可以设置它。

源码 cpp_parameters_node.cpp

```cpp
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamNode : public rclcpp::Node {
public:
    ParamNode() : Node("param_test"), count_(0) {
        // 定义参数
        this->declare_parameter("my_param", "Hello world");
        timer_ = this->create_wall_timer(1000ms, std::bind(&ParamNode::timer_callback, this));
    }
private:
    void timer_callback() {
        std::string my_param = this->get_parameter("my_param").as_string();
        RCLCPP_INFO(this->get_logger(), "Parameter: %s", my_param.c_str());
        std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_param", (std::string("Hello world") + " " + std::to_string(count_)))};
        // 设置参数
        this->set_parameters(all_new_parameters);
        count_++;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamNode>());
    rclcpp::shutdown();

    return 0;
}
```

可以为参数添加描述，声明过程如下：

```cpp
ParamNode() : Node("param_test"), count_(0) {
    // 定义参数描述
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This is a parameter";

    // 定义参数
    this->declare_parameter("my_param", "Hello world", param_desc);
    timer_ = this->create_wall_timer(1000ms, std::bind(&ParamNode::timer_callback, this));
}
```

### _1.5.2 Python 参数配置

1

### _1.5.2 启动文件参数配置

1

## _1.6 动作 Action

1

## _1.7 插件 Plugin

插件是一种 C++ 动态链接库，使用插件时，用户不需要预先知道库中的类和库的头文件。插件有助于应用程序的扩充、模块化，同时不需要提供源码。

插件编写依赖于`pluginlib`。

编写一个基类的头文件，基类是虚类，包含一个无参数的构造函数，一个虚析构函数，其余成员函数全部为纯虚函数。

> 详见 https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html
> 
> 这个是高级主题，暂时不会用到。

# _2 常用功能包

## _2.1 std_msgs

1

## _2.2 geometry_msgs

1

## _2.3 senser_msgs

1

## _2.4 tf2

1

# _3 常用工具

## _3.1 cv_bridge

图像话题转 OpenCV 矩阵。

```cpp

```

# note

应该整理一套可以直接拷贝的 ros 模板

C++ 和 Python 的都需要
