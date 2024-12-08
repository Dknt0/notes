# Gazebo Garden

> Dknt 2023.4

Gazebo Garden 是 2023.4.11 为止最新版的 Gazebo。与 Gazebo Classical (ogre)不同, 新版 Gazebo 默认使用 OpenGL 作为 3D 显示框架。

> Gazebo Garden 可以安装在 Ubuntu 20.04 以上的系统中, 但只能同 ROS 2 Hamble 以上的版本同时使用, 在我的机器（20.04）上无法使用 ROS 与 Gz 交互, 因此需要调用 Gazebo API 实现驱动器控制、传感器信息获取。如果需要, 我可以将获取的传感器信息通过 ROS1 Topic 发送出去。

## 0. 准备工作

## 0.1 下载与测试

通过命令行下载：

```shell
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```

测试场景：

```shell
gz sim shapes.sdf -v 4
```

`-v 4`输出生成Debug信息。

卸载：

```shell
sudo apt remove gz-garden && sudo apt autoremove
```

## 0.2 GPU OpenGL

Ubuntu 下 OpenGL 默认使用 Mesa, 不是显卡驱动中的 OpenGL, 导致画面显示不正常, 速度慢。这是一个普遍的问题, 任何基于 OpenGL 的软件都会遇到。

要解决这个问题, 首先要正确安装显卡驱动。之后, 在命令行中切换到 nvidia。

```shell
sudo prime-select nvidia
```

在`~/.bashrc`中输入如下内容：

```shell
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

之后运行 gz, 在 nvidia-smi 输出中可以看到 gz sim。

## 1. 机器人与世界

机器人模型通常为 URDF, 世界模型为 SDF。区别在与 URDF 只能包含一个机器人, SDF 可以包含多个机器人, 以及其他场景信息。

### 1.1 加载机器人模型

运行gz。终端输入命令, <mark>查询gz服务</mark>。这个和ROS中的服务很像。

```shell
gz service -l # 列出所有服务
gz service -is /world/empty/create # 查看服务详细信息
```

查看详细信息后, 返回结果：

```shell
gz.msgs.EntityFactory, gz.msgs.Boolean
```

第一个是调用服务需要的参数类型——完整目录；第二个是返回值类型——Bool。

以下命令在gz中生成一个名为`urdf_model`的机器人。

```shell
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/model.urdf", name: "urdf_model"'
```

不太好用, 不是所有模型都能顺利生成。

gz的模型库Fuel里有相当多好用的模型（通常是带插件的机器人）和地图, 可以常去看看。

对于水果、瓶瓶罐罐这些模型, 可以去找旧版Gazebo的模型库。

链接：[Fuel 模型库](https://app.gazebosim.org/fuel/models)

## 2. GUI

gz Garden 的界面十分简洁, 只有左上角和右上角两个按钮是一直存在的。左上角的按钮用于保存项目、保存或加载插件设置。一些常见的工具, 如生成模型、移动模型、模型树等作为插件使用。这些插件可以关闭, 也可以通过右上角的插件按钮找到并打开。

在gz中, 通常红色为x轴, 绿色代表y轴, 蓝色代表z轴。

常用插件：

`Entity tree` —— <mark>实体树</mark>, 显示所有实体 

`Compoment inspector` —— <mark>部件检查</mark>（设置模型属性等）

`Transform control` —— <mark>移动模型</mark>

`World control` —— 仿真控制

`Grid Config` —— 网格设置, 设置场景中的网格

`World start` —— 显示时间信息, 默认存在于右下角的插件

`View Angle` —— 改变视角, 左视图等

`Align Tool` —— <mark>对齐工具</mark>, 帮助我们对齐实体坐标, 有很多种对齐方式

`Resource Spawner` —— 浏览Fuel模型库

## 3. 快捷键

> gz Garden快捷键和教程里的不完全一样。

| 快捷键        | 功能                                       |
|:----------:|:----------------------------------------:|
| `Ctrl`     | 1. 按住Ctrl可以同时选择多个实体 2. 按住Ctrl按间隔(Snap)移动 |
| `Shift`    | 按住Shift将坐标系对其到世界                         |
| `r`        | 按住r选择实体, 进入旋转模式                           |
| `t`        | 按住t选择实体, 进入平移模式                           |
| `esc`      | 回到选择模式                                   |
| `Space`    | 开始/暂停仿真                                  |
| `/`        | 搜索插件                                     |
| `Ctrl + q` | 退出gz                                     |
| `Ctrl + o` | 加载设置                                     |

## 4. SDF模型

SDF是gz中世界的格式, 也可以是机器人的格式。

> [SDFormat的官方文档](http://sdformat.org/)

### 4.1 机器人模型

机器人由`link`和`joint`组成。他们位于`</model>` tag下。

<mark>定义模型</mark>：

```xml
<model name='vehicle_blue' canonical_link='chassis'>
    <pose relative_to='world'>0 0 0 0 0 0</pose>
```

* 其中name为模型的名字, 在场景中必须是唯一的。

* 模型的坐标系会固连于`canonical_link`, 如果没有给出, 则使用第一个`link`

* `</pose>`用于定义模型相对于`relative_to`参考系的位置, 默认为世界坐标系。坐标格式为`<pose>X Y Z R P Y</pose>`

<mark>添加连杆</mark>：

```xml
    <link name='chassis'>
        <pose relative_to='__model__'>0.5 0 0.4 0 0 0</pose>
```

* 添加连杆, 设置其与model的相对位置

连杆惯性参数设置

```xml
    <inertial> <!--inertial properties of the link mass, inertia matrix-->
        <mass>1.14395</mass>
        <inertia>
            <ixx>0.095329</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.381317</iyy>
            <iyz>0</iyz>
            <izz>0.476646</izz>
        </inertia>
    </inertial>
```

* `<mass>`为link质量

* `<inertia>`为惯性矩阵  [惯性矩阵计算网站](https://amesweb.info/inertia/mass-moment-of-inertia-calculator.aspx)

<mark>连杆可视化设置</mark>

```xml
    <visual name='visual'>
        <geometry>
            <box>
                <size>2.0 1.0 0.5</size>
            </box>
        </geometry>
        <!--let's add color to our link-->
        <material>
            <ambient>0.0 0.0 1.0 1</ambient>
            <diffuse>0.0 0.0 1.0 1</diffuse>
            <specular>0.0 0.0 1.0 1</specular>
        </material>
    </visual>
```

* `<geometry>`为外形

* 可以使用的外形有：box, cylinder, sphere.... 查文档

* 材料为颜色信息。环境光、漫反射、镜面反射。rgba范围为0~1。

<mark>连杆碰撞体设置</mark>

```xml
        <collision name='collision'>
            <geometry>
                <box>
                    <size>2.0 1.0 0.5</size>
                </box>
            </geometry>
        </collision>
    </link>
</model>
```

<mark>定义额外坐标系</mark>

```xml
<frame name="caster_frame" attached_to='chassis'>
    <pose>0.8 0 -0.2 0 0 0</pose>
</frame>
```

定义一个相对于机器人某部件的坐标系, 可以在这个坐标系的基础上增加连杆。

<mark>添加关节</mark>：

```xml
<joint name='left_wheel_joint' type='revolute'>
    <pose relative_to='left_wheel'/>
    <parent>chassis</parent>
    <child>left_wheel</child>
    <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz> <!--can be defined as any frame or even arbitrary frames-->
        <limit>
            <lower>-1.79769e+308</lower>    <!--negative infinity-->
            <upper>1.79769e+308</upper>     <!--positive infinity-->
        </limit>
    </axis>
</joint>
```

* 角度是弧度制

### 4.2 驱动器插件

插件是被编译为动态链接库的代码, 可以将其插入到仿真中, 用于控制仿真。

```xml
<plugin
    filename="gz-sim-diff-drive-system"
    name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
```

* `filename`为库文件名字

* `name`为插件名字

### 4.3 世界模型

可以在SDF世界模型中加入各种各样的插件。

```xml
<physics name="1ms" type="ignored">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
</physics>
```

上述代码确定了仿真时间1ms, 仿真器为忽略。也可以设置仿真器为Ode, Bullet, Simbody and Dart.

## 5. Gazebo 命令行

通过`gz gui`命令可以打开图形化管理界面, 方便显示话题、图像等。

发布话题

```shell
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
```

显示话题

```shell
gz topic -e -t /keyboard/keypress
```

# _ Gazebo 与 ROS 2 通信

Gazebo 与 ROS 2 之间通过`ros_gz_bridge`包进行通信。需要给出 ros 和 gz 中的话题名, 以及这个话题对应在 ros 和 gz 中的类型。

注意需要下载与 ROS 和 gz 版本对应的包, gz Garden 对应的包为 gz 而不是 ign。

```shell
sudo apt-get install ros-humble-ros-gz-bridge
```

支持的话题类型见[这个文档](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)

可以将话题信息保存在`yaml`文件中。

可以写一个 C++ 节点用于转发话题。
