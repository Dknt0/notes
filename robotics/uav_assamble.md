# F450四旋翼

计划：

1. 纯Linux开发

2. 模糊滑模控制

3. 完备的视觉系统

<mark>电池电压</mark>：最低 10.8V，充满 12.6V，长期保存 11.55V。

安装 PX4 固件，飞控串口有连接时（QGC、充电器等任意电源）可以用遥控器解锁，当连接断开时解锁报错。解决方案：将参数`CBRK_SUPPLY_CHK`调整为`894281`。

# 无人机组装

## 参考链接

## 零件

*动力系统*

电调：好盈乐天 20A

电机：郎宇 A2312

电池：3S 5200mAH（XT60电源线；带报警器）

螺旋桨：1045碳纤尼龙桨

*机架*

F450机架

脚架

减震板

*控制系统*

飞控：pixhawk2.4.8（飞控1个，蜂鸣器1个，安全开关1个，电流计1个，内存卡1张）

> [区分PIXHAWK、PX4与APM(ardupilot) | 所念皆星河](https://immortalqx.github.io/2021/07/27/uav-notes-1/)
> 
> Pixhawk是一款由PX4开源项目设计并由3DR公司制造生产的高级自动驾驶仪系统，其前身是APM。
> 
> Pixhawk是一个双处理器的飞行控制器，一个擅长于强大运算的 32 bit STM32F427 Cortex M4 核心 168 MHz/256 KB RAM/2 MB Flash 处理器，还有一个主要定位于工业用途的协处理器 32 bit STM32F103，它的特点就是安全稳定。所以就算主处理器死机了，还有一个协处理器来保障安全。
> 
> PX4固件专为PIXHAWK开发的固件。相对封闭，代码体系相对简单清晰，社区相对小，迭代慢一些，但因为相对清晰，适合学习研究。
> 
> PX4地面控制站被称为QGroundControl，（简称QGC）是PX4自驾系统不可分割的一部分，可以将PX4固件烧写到硬件，设置机器，改变不同的参数，获得实时航班信息，创建和执行完全自主的任务。

<img title="" src="file:///home/dknt/Projects/uavF450/image/656.jpeg" alt="" data-align="center" width="798">

GPS：M8N GPS+折叠支架

回传

电流记

遥控器：乐迪AT9S

地面站：QGroundControl

固件：PX4

*计划中*

机载电脑：Respberry Pi 4B

摄像头：D435i or 单目

## 安装步骤

安装步骤参考苍穹四轴公众号。

> 组装教程上 [(click here)](https://mp.weixin.qq.com/s?__biz=MzkyNzI1MDUyNw==&mid=2247484992&idx=1&sn=6a9c937df1ee3b226d24d1aa73793e9f&source=41#wechat_redirect)

### 1 焊接

#### 1.1 底板焊接

将4个电调，XT60电源线，和2根JST公头线焊接在底板上。2根JST公头预留给云台和图传供电。

<img title="" src="file:///home/dknt/Projects/uavF450/image/640.jpeg" alt="" width="502" data-align="center">

#### 1.2 电机接口焊接

香蕉头焊接和热缩管安装，郎宇 A2312不需要。

### 2 机架组装

<mark>教程里没有说明脚架的安装方式</mark>

#### 2.1 电机安装

将电机固定在机臂上，用机架里面配套的银色大螺丝安装电机。

<img title="" src="file:///home/dknt/Projects/uavF450/image/motor.jpeg" alt="" width="895" data-align="center">

#### 2.2 机臂安装

用机架里面的银色小螺丝将机臂固定在底板上，螺丝先不用拧得太紧，方便后面调节。

注意两种电机的安装位置！

打点螺杆是反丝，顺时针旋转；平头螺杆帽是正丝，逆时针旋转，这样安装以后才不会射桨。

<img title="" src="file:///home/dknt/Projects/uavF450/image/641.jpeg" alt="" width="424" data-align="center">

#### 2.3 机架上版安装

安装机架上板，拧紧固定螺丝。

<img title="" src="file:///home/dknt/Projects/uavF450/image/642.jpeg" alt="" width="404" data-align="center">

#### 2.4 紧固螺丝

上板安装好以后，拧紧底板螺丝。紧固所有螺丝。

#### 2.5 连接电机与电调

将电机焊接好的香蕉头分别插到电调的3个香蕉头里面。无刷电机三根线等价，他们的通电顺序决定转向，可以通过软件调整，这里任意连接。

<img title="" src="file:///home/dknt/Projects/uavF450/image/643.jpeg" alt="" width="410" data-align="center">

### 3 飞控安装

#### 3.1 安装减震板

先将减震球安装于小板上，然后再连接大板。注意：减震球不可用尖锐的工具安装，减震球破损后就没有减震的功能了。

<img title="" src="file:///home/dknt/Projects/uavF450/image/644.jpeg" alt="" width="353" data-align="center">

取一块3M胶，将减震板粘在机架上板中心。可在四个角用尼龙扎带绑扎一下，进一步加固减震板。

<img title="" src="file:///home/dknt/Projects/uavF450/image/645.jpeg" alt="" width="400" data-align="center">

<img title="" src="file:///home/dknt/Projects/uavF450/image/646.jpeg" alt="" width="386" data-align="center">

#### 3.2 飞控安装

用减震板里面的3M胶将飞控粘贴在减震板上。安装安全开关，蜂鸣器。

<mark>蜂鸣器与安全开关的安装步骤在后面</mark>

<img title="" src="file:///home/dknt/Projects/uavF450/image/647.jpeg" alt="" width="467" data-align="center">

<img title="" src="file:///home/dknt/Projects/uavF450/image/648.jpeg" alt="" width="473" data-align="center">

注意：飞控的箭头前向就是无人机的机头前向。<mark>明确机头前向很重要！</mark>涉及到后面电机的旋转方向，GPS安装方向和飞行时的前向的确定。

飞控要安装内存卡（PIX2.4.8发货前已经安装好内存卡）、蜂鸣器、安全开关（内存卡用于记录飞行时的数据。蜂鸣器会根据飞控输出不同提示音。安全开关是一个硬件保护措施，防止意外解锁电机转动造成伤害）

#### 3.3 电流计安装

电流计的作用是给飞控提供稳定的5V电源，并且测量电池电压和电流

<img title="" src="file:///home/dknt/Projects/uavF450/image/649.jpeg" alt="" width="382" data-align="center">

#### 3.4 GPS模块安装

GPS的箭头前向一定要和机头前向一致，同时连接到飞控的GPS接口和I2C总线接口。

GPS包含卫星定位与一个外部罗盘。

<img title="" src="file:///home/dknt/Projects/uavF450/image/650.jpeg" alt="" width="341" data-align="center">

<img title="" src="file:///home/dknt/Projects/uavF450/image/651.jpeg" alt="" width="287" data-align="center">

#### 3.5 电调接线

将电机对应的电调连接到飞控<mark>主要输出口main out1~4通道</mark>。电调线<mark>从机臂之间穿过</mark>到飞控后方。

注意<mark>黑色负极在上，白色信号在下</mark>。

<img title="" src="file:///home/dknt/Projects/uavF450/image/652.jpeg" alt="" data-align="center" width="663">

#### 3.6 接收机接线

<img title="" src="file:///home/dknt/Projects/uavF450/image/653.jpeg" alt="" data-align="center" width="497">

#### 3.7 回传模块接线

将回传模块连接到接收机、飞控的telem2接口。注意接线方向。

这样飞机起飞后遥控器就能接收到飞控实时回传的数据。连续按2下遥控器左下角的END，就能看到相关参数。

整理接线，将接收机和回传模块置于飞机左侧，先不用绑扎，后面还需要调试。

<img title="" src="file:///home/dknt/Projects/uavF450/image/654.jpeg" alt="" data-align="center" width="487">

#### 3.8 电池安装

剪一块魔术贴的针面，粘在飞机底板的前端位置。电池上粘魔术贴毛面。电池从机身后端往前塞进即可。

最后将BB响报警器插到电池平衡头上，注意插线方向不要弄错。否则不会显示。

<img title="" src="file:///home/dknt/Projects/uavF450/image/655.jpeg" alt="" data-align="center" width="458">

BB响低电压报警器：主要作用是提醒操作者电池电量较低，需要充电。在飞机飞行的时候需要一直插在电池上。BB响报警器连接后，会显示循环显示：ALL表示电池总电压，NO1, NO2, NO3分别表示3片电芯的分电压。比如我们用的是3S电池，就是由3片电芯串联的，充满电的总电压是<mark>12.6V</mark>左右，也就是单片电芯满电压是4.2V；总电压放电到<mark>10.8V</mark>左右，也就是单片电芯电压放电到3.6V就需要充电。锂电池不能完全放完电才充电，这样锂电池就损坏了无法修复。

BB响两个喇叭之间有一个小按钮，是设置报警电压的。预设为3.6V报警，不需要调整。

#### 3.9 整理接线

由于电机还没有调试方向、接收机没有调整模式，所以电调、接收机、回传模块暂时不用绑扎。

> 组装教程中 [(click here)](https://mp.weixin.qq.com/s?__biz=MzkyNzI1MDUyNw==&mid=2247484992&idx=1&sn=6a9c937df1ee3b226d24d1aa73793e9f&source=41#wechat_redirect)

### 4 飞控调试

> 我使用的地面站：**QGroundControl**；飞控固件：**PX4**。
> 
> 教程中的地面站：**Misson Planner**；飞控固件：**ardupilot**。
> 
> QGC支持linux，MP不支持。PX4相比于ardupilot更适合于研发。

调试需要一根<mark>Micro-B USB线</mark>。

调试过程中非明确说明<mark>不要连接电池</mark>。

#### 4.1 烧写固件

打开QGC，选择Vehicle Setup/Firmware，将飞控连接至电脑，左侧选择PX4，安装。

#### 4.2 选择机架类型

选择Quadrotor X。

#### 4.3 加速度计校准

按照提示，将飞控摆放于六种位置。

#### 4.4 指南针校准

飞控里面和GPS模块里分别各有一个指南针。

GPS里集成的指南针一般叫做外置罗盘是1#，飞控里面的指南针叫内置罗盘是2#。

**注意：校准罗盘请远离金属构件、喇叭等强磁性东西。**

校准时要求将飞机每个面都朝上旋转1~2圈（正反均可）。

<mark>MP参考图</mark>

<img title="" src="file:///home/dknt/Projects/uavF450/image/657.jpeg" alt="" data-align="center" width="797">

重新连接USB后，再连接地面站，指南针页面就能看到指南针的数据。数值小于 400 为绿色代表数值正常可用，当大于 400 黄色代表警告，当超过 600为红色完全不可用。大于400数值需要重新校准。

#### 4.5 遥控器校准

遥控器供电可以使用8节5号（？）电池，套件中包含一块遥控器专用的锂电池，可以直接使用，注意电池保养。

<img title="" src="file:///home/dknt/Projects/uavF450/image/658.jpeg" alt="" data-align="center" width="536">

##### 4.5.1 系统设置

长按Mode键，进入设置菜单。选择PARAMETER，按右边圆盘确认进入。

<img title="" src="file:///home/dknt/Projects/uavF450/image/659.jpeg" alt="" data-align="center" width="508">

继续旋转右边圆盘，将**发射报警**设置成10.5V，这是遥控器电池低压报警值。将**动力报警**设置成 10.6V（对于3S电池而言，如果是4S电池，则设置成14.4V），这是飞行器电压低压报警值。以后当飞行器电量低于设定值，遥控器会震动报警。

摇杆模式和发射模式不能改。用户名字可以自己改。

<img title="" src="file:///home/dknt/Projects/uavF450/image/660.jpeg" alt="" data-align="center" width="535">

##### 4.5.2 机型设置

系统设置完成后，按左下角**END**键，回到菜单。旋转圆盘，选择**机型选择**。

进入后，将直升机模型设置成**多旋翼模型**，长按圆盘确认。

##### 4.5.3 相位设置

进入**舵机相位**菜单，将油门相位设置成**反相**。

##### 4.5.4 辅助通道设置

进入**辅助通道**菜单，然后选择**五通**进入。

将5通道，三段开关分配给SWE，二段开关分配给SWD，其他设置依照下图分别设置。注意：模式名称与百分比参数不能设置错误，否则和后面设置飞行模式对应不上。

<img title="" src="file:///home/dknt/Projects/uavF450/image/661.jpeg" alt="" data-align="center" width="563">

当开关D在上档，开关E 3个档位分别对应：**定高、悬停、返航**

当开关D在下档，开关E 3个档位分别对应：**降落、绕圈、自稳**

<mark>开关组合要牢记，飞行时很重要</mark>

通道设置好以后，再根据下图分别设置好6~10通道。

<img title="" src="file:///home/dknt/Projects/uavF450/image/662.jpeg" alt="" data-align="center" width="582">

遥控器背后两个拨杆分别对应VRC和VRD两个开关。

<img title="" src="file:///home/dknt/Projects/uavF450/image/663.jpeg" alt="" data-align="center" width="510">

##### 4.5.5 接收机模式设置

AT9S接收机出厂默认是PWM模式输出，需要将它设置成SBUS模式输出，飞控才能识别到遥控器信号。给飞机接上电池，这时候会看到接收机亮红灯，表示是PWM信号输出。

找一个小螺丝刀，**连续按2下**接收机侧面的小按钮，LED变成蓝色。表示接收机是SBUS信号输出。

<img title="" src="file:///home/dknt/Projects/uavF450/image/664.jpeg" alt="" data-align="center" width="429">

设置好以后，可以将接收机，回传模块绑扎好。

<img title="" src="file:///home/dknt/Projects/uavF450/image/665.jpeg" alt="" data-align="center" width="419">

##### 4.5.6 地面站设置

选择好波特率115200与端口后点击connect连接PIX，接着点击校准遥控按钮。

点击OK开始。将遥控器左右两边摇杆上下左右均推到最大，使每个通道的红色提示条移动到上下限的位置。并且四个通道的最大值和最小值数据都应该差不多。

<img title="" src="file:///home/dknt/Projects/uavF450/image/666.jpeg" alt="" data-align="center" width="657">

现在使用最多的是左手油门（也叫美国手：油门摇杆在左边，不能自动回中），摇杆对应通道如下：

通道 1：低 = roll 向左，高 = roll 向右。（横滚：控制飞机左右运动）

通道 2：低 = pitch 向前，高 = pitch 向后。（俯仰：控制飞机前后运动）

通道 3：低 = throttl减（关），高 = throttl 加。（油门：控制飞机动力及高度）

通道 4：低 = yaw 向左，高 = yaw 向右    （航向：控制飞机机头方向）

不管是哪个品牌的遥控器，1~4通道都是分配的摇杆的，是固定不变的。

拨动E和D开关，查看对应5通道是否动作；

拨动VRC开关，查看对应的6通道是否动作；

拨动A开关，查看对应7通道是否动作；

拨动B开关，查看对应8通道是否动作;

拨动H开关，查看对应9通道是否动作;

#### 4.6 飞行模式设置

飞机的飞行姿态，一方面是遥控器摇杆人为控制，比如前后，左右运动。一方面，就是靠选定的飞行模式控制，保持空中定高，或则定点。切换不同的飞行模式，飞行器也就执行不同的预设动作。

<mark>要保证熟练了一种飞行模式再进行下一个练习</mark>

点击**初始设置**->**必要硬件**->**飞行模式**选择，按下图设置飞行模式

<img src="file:///home/dknt/Projects/uavF450/image/667.jpeg" title="" alt="" data-align="left">

ALTHOLD - 定高

LOITER - 定点

RTL - 返航

LAND - 降落

CIRCLE - 绕圈

STABILITE - 自稳

设置SWA切换简单模式。地面站软件点击调试配置页面->扩展调参，将CH7设置为SIMPLE MODE简单模式。简单模式就是无头模式。

#### 4.7 飞控解锁与上锁

<mark>调试阶段不能安装桨叶！</mark>

做完上面的六步，长按安全开关，安全开关LED灯长亮，飞控LED变成蓝色闪烁状态，表示飞控就可以解锁了。

**<mark>解锁方法</mark>：**

1. 将飞行模式开关切换到自稳模式，开关组合：E=下，D=下

2. 首先长按安全开关，直到灯停止闪烁变为常亮

3. 遥控器解锁，PIX 的解锁动作是以检测到第三通道最低值+第四通道最高值为标准的，即油门最低，方向最右。请保持油门最低，方向最大的动作3秒左右。

<img title="" src="file:///home/dknt/Projects/uavF450/image/668.jpeg" alt="" data-align="center" width="623">

当 PIX 解锁成功，LED 灯长亮，蜂鸣器会长响一声

观察地面站， **故障保护**页面，3通道值最小（在1000左右），4通道值最大（在2000左右），DISARMED 会变成 ARMED，表示解锁成功。

解锁后推油门，电机就会旋转。并且右边电机输出会随着油门的增加而增大。

**<mark>上锁方法</mark>：**

1.油门摇杆处于最低位15秒没有任何动作，飞控自动上锁；

2.油门摇杆往左下角一直按住3秒左右上锁。

#### 4.8 电机旋转方向调整

解锁后电机可以转动，我们就要进行电机旋转方向的调整（前面遗留未做）。

推动油门，电机转动，用手轻触电机外壳，能感觉到电机转向。

如果电机旋转方向与示意图不同，就交换电机与电调的任意两根接线。这样电机的旋转方向就改变了。

电机旋转方向调整好，可以用尼龙扎带将电调绑扎在机臂上。做到整洁美观。减少震动。

> 组装教程下 [(click here)](https://mp.weixin.qq.com/s?__biz=MzkyNzI1MDUyNw==&mid=2247484992&idx=1&sn=6a9c937df1ee3b226d24d1aa73793e9f&source=41#wechat_redirect)

#### 4.9 失控保护设置

<mark>这里的失控返航是APM固件的</mark>

PIX 的失控保护是通过故障保护菜单配置的，点击初始设置->必要硬件->故障保护。

使用最多的失控保护有：油门失控保护和低电压失控保护（需电流计）。

失控保护启动后有三种选项： RTL（返航），继续任务，LAND（着陆）等可选，我们一般是选RTL（返航）。

最重要的是<mark>油门失控保护</mark>，它的正确理解是：当飞行器飞出遥控器的控制范围，接收机无法接收到遥控器的控制信号，飞机就会自动返回到起飞的位置，并且自动降落上锁。整个过程不需要人工干预。油门失控保护在回航过程中遥控器可以控制飞机前后左右。

但是失控返航是有**前提条件**的：起飞前GPS一定要定位成功，返航时GPS信号必须稳定。

<mark>低电压失控保护</mark>是指：当电池电压低于设定值，飞机就自动返回起飞点并降落，要求也是GPS信号必须稳定得到保证。

开源飞控如果触发了低电压返航，整个回航过程，**遥控器是无法干预**的，无人机没有自主避障功能，容易炸机。

解决低电压回航的方法有：

1. 在电池上安装BB响报警器，低电压报警提醒操作者返航；（适合视线范围内飞行，均能听到）

2. 使用遥控器配套的电压回传模块将电压回传到遥控器提示操作者返航；（适合遥控器带电压回传功能）

3. 带航拍设备的无人机可以加OSD模块，将电压信息回传到画面上，提醒操作者返航。

##### 4.9.1 遥控器设置

1. 将油门摇杆拉到最低位，往下调油门摇杆微调到-88，如下所示：

<img title="" src="file:///home/dknt/Projects/uavF450/image/669.jpeg" alt="" data-align="center" width="518">

2. 将油门摇杆拉到最低位，往下调油门摇杆微调到-88，如下所示：

3. 长按Mode键进入菜单，旋转右边圆盘选择 失控保护，按圆盘确认进入。

4. 继续转动圆盘，选择3：油门，按一下圆盘确认，选择 F/S，然后再长按圆盘，数值会自动变到7%（或则6%）。

5. 同样的方法，将 5：姿态 设置成-14%。

##### 4.9.2 地面站设置

初始设置--必选硬件--故障保护

电台->Enabled always RTL 油门失控返航

故障保护PWM=975  油门值低于975触发失控返航

设置完成后，<mark>务必记得将油门微调恢复到0</mark>，否则飞控一直处于失控保护状态无法解锁！

检测是否设置好，可将遥控器关闭，观察第3通道油门值是否在975以下，飞行模式是否自动切换到RTL。

#### 4.10 电流记设置

不建议开启飞控的低电压返航功能，但是我们可以开启电流计的检测功能，这样就能在地面站实时看到电池电压。

点击 **初始设置**->**可选硬件**->**电池监测器**页面，按照如下所示设置

<img title="" src="file:///home/dknt/Projects/uavF450/image/670.jpeg" alt="" data-align="center" width="622">

设置好以后，将飞控USB断开，再重新连接电脑，并且需要将电池连接飞行器，按照下图填写电池电压。

<img src="file:///home/dknt/Projects/uavF450/image/671.jpeg" title="" alt="" data-align="center">

电压设置完成，即可在地面站上看到电压。

设置好电流计，遥控器上也能看到飞行器电池电压。连续按2下遥控器的END键，就能看到回传参数菜单。

#### 4.11 整理绑扎，安装桨座桨叶

上面飞控的调试全部完成，将多余的线绑扎好，固定牢固。<mark>避免桨叶打到</mark>，减少震动。

**桨叶的识别：**

根据迎风面的不同，桨叶有正反之分。一般桨叶上带R字样的，如1045R，1147R是正桨，安装在顺时针旋转电机上。不带R字样的，如1045,1147就是反桨，安装在逆时针旋转的电机上。
