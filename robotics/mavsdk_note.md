# MAVSDK 笔记

> 参考：
> 
> [Introduction · MAVSDK Guide](https://mavsdk.mavlink.io/main/en/index.html)
> 
> Dknt 2023.4.22

MAVSDK 是一个为PX4开发的，高性能的、可靠的 MAVLink C++ 库，已应用于业界。可用于视觉、避障、路径规划等任务。

类似的 MAVLink 通信方法有 dronesdk (Python), MAVROS。 C++ 最靠谱。

> 事实证明，APM 飞控也可以用 MAVSDK。可能有未知的 bug。

# 1 MAVSDK 基础编程

## 1.1 概述

### 1.1.1 主要类

MAVSDK 由几个类组成。其中最基础的类是`Mavsdk`，通过这个类可以连接到端口，监听端口上出现的新系统，可以认为是地面站类、上位机类。从`Mavsdk`中实例化`System`对象，对应一个具体的载具。对载具信息的获取和控制通过**插件**实现，插件类对象需要用对应的`System`对象初始化。

MAVSDK 中主要的类如下：

`Mavsdk` - 最基本的类，用于连接到载具

`System` - 系统。代表连接到的载具，通过后面的这些类提供对载具访问与控制

`Info` - 信息插件。最基本的硬件软件信息

`Telemetry` - 数传插件。获取数传数据和状态信息，设置频率

`Action` - 动作插件。如解锁、起飞、降落

`Mission` - 任务插件。任务上传/下载，任务由`MissionItem`对象创建

`Offboard` - 外部控制插件。外部控制，实现速度控制，是无人机智能化的基础。

`Geofence` - 地理栅栏插件

`Gimbal` - 云台控制插件

`Camera` - 相机插件

`FollowMe` - 跟随插件

`Calibration` - 校准插件

`LogFiles` - 日志插件。下载日志文件

### 1.1.2 编程技巧

<mark>错误句柄</mark>

MAVSDK中参数获取、控制等函数通常会有一个错误返回值，用于判断错误类型。这个返回值仅代表载具是否会执行指令，并不代表指令执行完毕。

<mark>回调函数</mark>

在如`Telemetry::subscribe_position(PositionCallback callback)`或`Action::takeoff_async(ResultCallback callback)`中会使用回调函数，可以传入一个函数指针、仿函数、匿名函数。

所有回调函数都在当前线程中执行，因此不要处理太多消息。

<mark>同步与异步函数</mark>

MAVSDK 中许多函数（命令）提供了**同步**和**异步**两种方式。

同步方式是阻塞式的，向固件发送民令，等待成功或失败的结果。

异步方式是非阻塞式的，传入一个回调函数作为参数，可以是函数指针、仿函数、匿名函数。常用 promise future （C++ 多线程）的方式实现异步并发，在回调函数中为 promise 赋值，在外部调用 future 的 get 阻塞。如果停止这个异步函数，则要向其传入空指针。MAVSDK 中大量使用了 promise future。

在后面的例子中会有体现。异步方式更灵活。

最好不要`using namespace std;`！

## 1.2 Cmake 编译

根据使用的插件，链接到对应的库文件。

```cmake
find_package(MAVSDK REQUIRED)

include_directories(
    "/usr/include/mavsdk"
)

target_link_libraries(target
    MAVSDK::mavsdk
    MAVSDK::mavsdk_action
    ...
)
```

## 1.3 建立连接 Mavsdk & System

首先建立`Mavsdk`对象，连接到端口，再实例化`System`对象。

可以使用TCP、UDP或串口通信。形式如下：

| Connection | URL Format                          |
| ---------- | ----------------------------------- |
| UDP        | `udp://[Bind_host][:Bind_port]`     |
| TCP        | `tcp://[Server_host][:Server_port]` |
| Serial     | `serial://[Dev_Node][:Baudrate]`    |

当三秒内没有受到心跳信息时，认为连接中断。

> udp://:14540 是 PX4 默认的端口，虽然在我编译的固件(XTDrone)里是另一个。

地面站可以使用 server 模式或 client 模式。

使用`add_any_connection`函数<mark>建立连接</mark>，也可以使用每种连接方式专用的函数。建立连接后，`Mavsdk`会自动监听端口，并创建`System`对象。

UDP 连接。有服务器和客户端两种模式，默认为服务器模式。通常地面站的 mavsdk 使用服务器模式，机载电脑、固件使用客户端模式。

服务器模式下会监听端口，直到心跳到来。客户端模式会连接到端口。初始化时，两者区别如下：

```cpp
// 服务器 mavsdk
ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540");
// 客户端 mavsdk
ConnectionResult connection_result = mavsdk.add_any_connection("udp://192.168.1.12:14550");
```

> 看一下 TCP / UDP 原理。

TCP 连接只支持客户端模式。

```cpp
ConnectionResult connection_result = mavsdk.add_any_connection("tcp://192.168.1.12:14550");
```

用如下函数发现新系统，并在回调函数中处理这个系统。

```cpp
mavsdk.subscribe_on_new_system(callback)
```

可以在两个连接之间转发 MAVLink 信息，略。

一个最简单的建立连接，并实例化系统对象的例子如下：

```cpp
#include <mavsdk/mavsdk.h>
Mavsdk mavsdk;
auto conn_result = mavsdk.add_any_connection(port);
while (mavsdk.systems().size() == 0) {
    std::this_thread::sleep_for(chrono::seconds(1));
}
// system 是一个 shared_ptr<System>
auto system = mavsdk.systems()[0];
```

## 1.4 信息类 Info

`Info`类用于获取载具信息，包括 UUID, PX4 固件版本，供应商固件版本，宿主系统版本（Nuttx, Linux等）。

```cpp
#include <mavsdk/plugins/info/info.h>
auto info = Info{system};
```

好像没啥好说的，用的话看看官网吧。

## 1.5 数传类 Telemetry

数传类是最重要的插件之一。用于获取数传信息，包括载具状态和飞行模式信息。

数传类所有的方法都有同步与异步两种，用户可以设置更新频率。

```cpp
#include <mavsdk/plugins/telemetry/telemetry.h>
auto telemetry = Telemetry{system};
```

### 1.5.1 设置更新频率

分为同步和异步两种方法。

同步方法：

```cpp
const Telemetry::Result set_rate_result = telemetry.set_rate_position(1.0); // 阻塞
if (set_rate_result != Telemetry::Result::Success) {
    std::cout << "Setting rate failed:" << set_rate_result << '\n';
}
```

异步方法：

```cpp
{
    auto prom = std::make_shared<std::promise<Telemetry::Result>>();
    auto future_result = prom->get_future();
    telemetry.set_rate_position_async(1.0, [prom](Telemetry::Result result) {
        prom->set_value(result); //fulfill promise
    });
    const Telemetry::Result result = future_result.get(); // 阻塞
    if (result != Telemetry::Result::Success) {
        std::cout << "Setting telemetry rate failed (" << result << ")." << '\n';
    }
}
```

### 1.5.2 接受数传更新

异步方法

```cpp
telemetry.subscribe_position([](Telemetry::Position position) {
    std::cout << "Altitude: " << position.relative_altitude_m << " m" << std::endl
              << "Latitude: " << position.latitude_deg << std::endl
              << "Longitude: " << position.longitude_deg << '\n';
});
```

### 1.5.3 自检

通过数传类可以获取无人机的健康信息，以及传感器是否需要标定。

获取健康信息，可以查看传感器是否需要标定

```cpp
Telemetry::Health check_health = telemetry.health();
```

检查是否可以起飞。

```cpp
while (telemetry.health_all_ok() != true) {
    ...
}
```

## 1.6 动作类 Action

动作类是最重要的插件之一。所有命令分为同步和异步两种。

```cpp
#include <mavsdk/plugins/action/action.h>
auto action = Action{system};
```

<mark>动作执行前提</mark>。无人机起飞需要满足一定条件，不是说飞就能飞的（当然也有强制指令）。通常，起飞流程如下：自检（Health check），启动(arm)，起飞(takeoff)，其他任务(other missions)。

常用命令。以下展示同步

解锁 Arm

```cpp
const Action::Result arm_result = action.arm();
```

起飞 Takeoff

```cpp
const Action::Result takeoff_result = action.takeoff();
```

降落 Landing

```cpp
const Action::Result land_result = action.land();
```

返航 RTL

> 注意 RTL 的切换是动作类实现的，官方文档的示例代码有误。

```cpp
const Action::Result rtl_result = action.return_to_launch();
```

上锁 Disarm

```cpp
const Action::Result disarm_result = action.disarm();
```

关机 Kill

```cpp
const Action::Result kill_result = action.kill();
```

> 上所和关机的区别在于，仅当无人机着陆后在可以 Disarm，但 Kill 可以在无人机飞行过程中强制关机。

设置巡航速度

飞向固定点 Fly to

## 1.7 任务类 Missions

自动化任务中不包括解锁、起飞、降落、RTL，如果需要，在执行任务前用动作类完成上述命令。

**暂略**

## 1.8 外部控制 Offboard

外部控制提供了对速度和偏航角度的控制，机载电脑控制载具时使用的就是这种方法。外部控制是自主无人机实现的前提，机载电脑上的程序是自主飞行的灵魂。

```cpp
#include <mavsdk/plugins/offboard/offboard.h>
auto offboard = Offboard{system};
```

> PX4 Offboard 必须有<mark>稳定的 RC 输入</mark>作为前提，缺少 RC 会导致故障保护生效，无法进入外部模式。在仿真时，我们可以先运行 QGC，打开虚拟摇杆，再运行 SITL 仿真。（仿真运行后加入摇杆无效）
> 
> APM Offboard 好像不需要 RC。

## 1.8.1 开始和结束

使用外部控制模式，首先要设定值（setpoint），可以是速度或位置。外部控制开始后，会执行这个设定值。

**开始和结束**

```cpp
// 开始前需要设定值
offboard.set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
// 开始外部控制
Offboard::Result offboard_start_result = offboard.start();
// 结束外部控制
Offboard::Result offboard_stop_result = offboard.stop();
```

### 1.8.2 速度设定值

API 提供两种方法设置速度和偏航角，基于绝对坐标系（NED frame, North, East, Down）和相对坐标系(body frame)。

NED frame 例子如下。函数中列表初始化`VelocityNEDYaw`对象，前三个数为北、东、下方向的速度（相对于 NED，m/s），第四个为**偏航角**（**deg**）。

```cpp
offboard.set_velocity_ned({5.0f, -5.0f, 0.0f, 0.0f});
```

body frame 例子如下。前三个数为前、右、下方向的速度（m/s），第四个为**角速度**（**deg/s**）。

```cpp
offboard.set_velocity_body({0.0f, 0.0f, 0.0f, 60.0f});
```

> 还有位置控制、加速度控制模式，略
> 
> 键盘好像用加速度控制更灵活

## 1.9 日志类 Logging

MAVSDK 默认在 std::cout 中输出日志信息，很乱。我们可以重载日志类，将日志信息输出到文件，或隐藏。

```cpp
// 仅在终端中输出错误信息
mavsdk::log::subscribe([](mavsdk::log::Level level,
                            const std::string &message,
                            const std::string &file,
                            int line){
    return (level != mavsdk::log::Level::Err);
});
```

> MAVSDK 设计中没有考虑多线程的安全性，需要我们自己考虑。
> 
> 回调函数应尽可能简短。
