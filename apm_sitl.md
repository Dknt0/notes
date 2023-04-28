# 无人机仿真

目标：在Ubuntu下使用APM SITL、ROS、Gazebo Garden联合仿真实现基于VSLAM的无人机动态避障。

> 好家伙，好难，好会给自己找事 \\(— _ —)/

# 0 环境搭建

> 做本科毕设时使用过仿真平台XTDrone，当时由于PX4的问题，只能运行在Ubuntu18、Gazebo9下。我不想重装系统，并且Gazebo Classical在我的电脑上运行很不顺畅。最近看AMP官网，提供对新版本Ubuntu和Gazebo的支持，所以用Gazebo Garden试试，参考XTDrone的代码，实现仿真。
> 
> 2023.4.6  dknt
> 
> 软件版本：
> 
> Ubuntu 20.04
> 
> ROS Noetic
> 
> Gazebo Garden
> 
> Ardupilot (APM)
> 
> 参考：
> 
> [无人机 - 个人心得分享](http://www.lxshaw.com/tech/drone) （难得的中文资料！）
> 
> [Getting Started &mdash; MAVProxy documentation](https://ardupilot.org/mavproxy/docs/getting_started/index.html)
> 
> [SITL Simulator (Software in the Loop) &mdash; Dev documentation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

## 0.0 对PX4不切实际的幻想

PX4好像更新了对高版本Ubuntu、ROS、Gazebo的支持。

使用PX4飞控。源码下载，运行配置脚本

```shell
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

安装Gazebo Garden，参考Gazebo官网。

之后cd到PX4固件的目录下，执行命令：

```shell
make px4_sitl gz_x500
```

问题：

Protobuf版本

Makefile中没有gz_x500 target

放弃。

## 0.1 APM仿真环境搭建

PX4编译不成功，使用APM。

按照官网配置APM，SITL。

运行如下命令，进行仿真：**注意在这会在当前目录下创建一些文件！，最好新建一个文件夹**

```shell
gz sim -v4 -r iris_runway.sdf
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

在`simvehicle.py`终端中输入如下命令，无人机起飞：

```shell
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
```

# 1 仿真入门

## 1.1 APM SITL

SITL(Software in a loop)仿真，是用x86 C++编译器编译得到的APM固件，允许我们在PC上进行无人机仿真。对于MAVLINK来说，SITL和通过Telemetry连接的飞控是没有区别的，我们基于SITL仿真得到的代码，可以直接部署在真机上。

> 最好在一个新的文件夹下运行SITL，因为会创建一些奇奇怪怪的文件！主要是MAVProxy的日志、EEPROM等。

运行如下脚本进行SITL仿真，这个脚本同时启动了**SITL**和**MAVProxy**。在终端中输入命令可以控制无人机完成指定任务。SITL仿真需要学习MAVProxy。

```shell
sim_vehicle.py --console --map
```

> SITL位于ardupilot/build/sitl/bin目录下，我们可以通过如下命令运行一个纯SITL`ardupilot/build/sitl/bin -S -I0 --home -35.363261,149.165230,584,353 --model "+" --speedup 1 --defaults ardupilot/Tools/autotest/default_params/copter.parm`

可以对SITL仿真进行可视化，使用可视化软件FlightGear，很大，2G。

**设置模型和机型**（参数），使用参数`-v`和`-f`。

```shell
sim_vehicle.py -v ArduCopter -f quad --console --map
```

**设置初始位置**，使用参数`-L`。 可用位置见ardupilot/Tools/autotest/locations.txt

```shell
sim_vehicle.py -L Ballarat --console --map
```

**添加参数设置**。可以通过如下方法在启动SITL时设置飞控参数。当然，也可以在SITL启动后设置。当然，啥也不会的我选择默认。

```shell
sim_vehicle.py -v ArduPlane --console --map --add-param-file=<path to file>
```

在运行sim_vehicle.py时添加`-w`参数，可以擦除当前文件夹下的EEPROM设置。对，没错，模拟的单片机ROM。

SITL可以运行在实物飞控上，所以存在使用遥控器控制仿真环境中无人机的可能性。

> 这会很好玩！

## 1.2 MAVProxy

MAVProxy是一个Python包。用于实现基于MAVLINK的通信。注意，MAVLINK并不依赖于SITL。

### 1.2.1 基础部分

一个MAVProxy只能连接到一个载具。

> 多个MAVProxy好像能连到同一个载具
> 
> 同理最好在一个新的文件夹下运行。
> 
> 不支持C++

MAVProxy通过命令行启动。如果只有一架无人机，则不需要指定USB口或端口。同理，通常也不需要我们设置波特率。

```shell
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600
mavproxy.py --master=tcp:127.0.0.1:14550 # 可以使用udp
mavproxy.py --master=tcpout:10.10.1.1:14550 # 远程端口
```

数传转发。MAVProxy可以通过网络将天线接收的无人机信息发送给其他设备上的GCS，实现远程控制。使用参数`--out`

```shell
mavproxy.py --out 127.0.0.1:14550
```

有很多参数可以选择，其他的好像用不到。见[Startup Options — MAVProxy documentation](https://ardupilot.org/mavproxy/docs/getting_started/starting.html)

**启动脚本**。暂时不知道有啥用，应该很有用。

每一次启动MAVProxy，都会将接收到的数传数据记录到日志中。主要是flight.tlog和flight.tlog.raw。可以用MAVExplorer分析日志文件、绘图。

MAVProxy运行中，可以通过`set`<mark>查看和修改参数</mark>。见[Settings &mdash; MAVProxy documentation](https://ardupilot.org/mavproxy/docs/getting_started/settings.html)

```shell
param set <settingname> <value>
```

如果不给出参数，则打印当前参数值

通过`param`查看和修改参数

```shell
param show rtl_alt # 查看参数
param set rtl_alt 2100 # 修改参数
```

> 这两种参数有什么区别？

<mark>常用命令</mark>：[MAVProxy Cheatsheet &mdash; MAVProxy documentation](https://ardupilot.org/mavproxy/docs/getting_started/cheatsheet.html)

### 1.2.2 无人机操控与配置

以下是MAVProxy中一些常用的无人机操控命令。运行MAVProxy，成功连接到飞控后，可以调用这些命令，对无人机进行操控和配置。

<mark>启动与制动</mark>

```shell
arm throttle # 启动油门
arm throttle force # 强制启动油门
disarm # 制动
disarm force # 强制制动，我称之为坠机
```

<mark>启动配置</mark>

```shell
arm safetyon # 打开安全开
arm safetyoff # 关闭安全开关
arm check X # 启动时检查X
arm uncheck X # 启动时不检查X
```

X可以是：all, baro, compass, gps, ins, params, rc, voltage, battery, airspeed, logging, switch, gps_config

<mark>路径点设置</mark>

```shell
wp list # 显示现在导入飞控的路径点
wp load filename.txt # 从文件导入路径点
wp save filename.txt # 将现有路径点保存到文件
wp clear # 清空现有路径点
wp update filename.txt 6 # 导入文件中指定路径点
wp move 6 # 将某路径点移动到新地址
wp loop # 闭合路径
wp remove 6 # 删除路径点
wp set 6 # 将某路径点设置为起点
wp undo # 回到最近路径点
```

<mark>地理栅栏</mark>

console中有fence的GUI

```shell
fence list # 显示当前地理栅栏列表
fence load filename.txt # 加载地理栅栏
fence save filename.txt # 保存当前地理栅栏到文件
fence draw # 在map上绘制地理栅栏
fence enable # 使能地理栅栏
fence disable # 禁用地理栅栏
fence clear # 清除所有地理栅栏
fence move 6 # 重新设置某个栅栏点位置
fence remove 6 # 删除某个栅栏点
```

<mark>集结点</mark>

当检测到故障时，无人机会选择最近的集结点RTL。一个飞控最多储存5个集结点。

console中有rally的GUI

```shell
rally list # 显示当前集结点列表
rally load filename.txt # 从文件加载集结点
rally save filename.txt # 保存集结点到文件
rally clear # 清除当前集结点
rally move 6 # 移动某个集结点
rally add # 添加一个集结点
rally remove 6  # 删除一个集结点
rally land # 降落在一个集结点
```

<mark>飞行模式</mark>

最常用的：guided, auto, rtl, land

```shell
mode auto # 切换到自动模式，飞向第一个路径点
mode loiter # 切换到闲逛模式（？？）
mode rtl # 返航模式
mode manual # 手动模式，对四旋翼，自稳模式
mode fbwa # 固定翼的一个模式
mode land # 固定翼，降落模式
mode guided # 向导模式。飞向指定点，或给出经纬度、高度
```

<mark>传感器校准</mark>

```shell
# 好像不需要校准
```

<mark>系统命令</mark>

```shell
# 暂时用不到
```

## 1.3 QGroundControl

Ubnutu下使用QGC比较方便，但是在运行时候出现了Qt版本的问题。

命令行输入以下命令，问题解决。（每个新终端中都要重新输入一次）

这个也可以解决rqt的问题，<mark>遇到Qt相关错误，执行下面这行命令试试</mark>！

```shell
export LD_LIBRARY_PATH=/opt/qt514/lib/
```

## 1.4 Gazebo Garden

打开Gazebo Garden，运行QGC，运行SITI。可以用QGC控制无人机飞行。

终端输入以下指令，运行Gazebo Garden（需要在world目录下）：

```shell
gz sim -v4 -r iris_runway.sdf
```

运行SITL，参数设置为gazebo

```shell
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

> 终于动了，仿佛看到了希望，呜呜呜

<mark>重装显卡驱动后，Gazebo不能用了</mark>，OpenGL报错，真好。解决方案，运行Gazebo时添加如下参数：

```shell
--render-engine ogre
```

怎么加摄像头，并通过ROS通信？

## 1.5 MAVLink

> 参考：[无人机编程入门（六）：MAVLink和DroneKit简介和安装 - 个人心得分享](http://www.lxshaw.com/tech/drone/2021/04/11/%e6%97%a0%e4%ba%ba%e6%9c%ba%e7%bc%96%e7%a8%8b%e5%85%a5%e9%97%a8%ef%bc%88%e5%85%ad%ef%bc%89%ef%bc%9amavlink%e5%92%8cdronekit%e7%ae%80%e4%bb%8b%e5%92%8c%e5%ae%89%e8%a3%85/)

MAVLink是无人机和地面站直接通信的标准信息协议。

一条MAVLink的大小在8字节到263字节之间，总是包含以下内容并且按照顺序排布：

1. 开始位，总是以0xFE开始，不以这个开始的字节包都不是MAVLink且不会被无人机解析
2. Payload length，表达了包的长度
3. Packet sequence，可以用于探测包丢失
4. System ID，一般来说1指无人机，255指地面站，取值范围可以是1-255
5. Component ID，用来区分传输来的信息所属的传感器
6. Message ID，代表信息的类型，这样可以用对应的解析方式解析
7. Data，信息的内容
8. 校验位

对我们来说，最重要的是6和7。

另一种重要的MAVLink是Command Long，这种命令可以把其他命令包含在数据包里，并额外发送最多七个参数。

> 能不能直接使用MAVLink与APM SITL通信
> 
> 学一下，学一下

## 1.6 DroneKit

> 参考：[无人机编程入门（七）：DroneKit使用（一） - 个人心得分享](http://www.lxshaw.com/tech/drone/2021/04/11/%e6%97%a0%e4%ba%ba%e6%9c%ba%e7%bc%96%e7%a8%8b%e5%85%a5%e9%97%a8%ef%bc%88%e5%85%ad%ef%bc%89%ef%bc%9adronekit%e4%bd%bf%e7%94%a8%ef%bc%88%e4%b8%80%ef%bc%89/)
> 
> 注意DroneKit版本。

DroneKit是一个Python包，可以用DroneKit简单地生成MAVLink。可以用DroneKit将MAVLink和其他Python任务结合起来，包括ROS。

> 虽然我更倾向于C++，但还是学一下吧，了解一下。
> 
> 试着写一个键盘控制无人机的程序

详见例程。

需要学习MAVLink命令。

操作包括模式切换、启动、飞向指定点、设置速度等。编程时候一定注意，<mark>只有当上一步操作完成时，才能执行下一步操作</mark>。因此每一条指令后都需要加入循环等待，直到达到预期的状态。

## 1.7 MAVROS

> 参考：[ROS with SITL &mdash; Dev documentation](https://ardupilot.org/dev/docs/ros-sitl.html)

APM 有Python DroneKit包，用于MAVLink通信，但并没有提供像PX4 MAVSDK这样的C++ SDK。用C++和APM SITL通信几乎只能通过MAVROS（没错，你还可以自己基于MAVLink写一个库），但我本来就要用ROS去做，所以刚好。

# 2 综合仿真

1

ORB-SLAM

Path Planning Alghorithm
