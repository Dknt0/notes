# DDS 数据分发服务

DDS (Data Distribution Service) 是 2004 年由对象管理组织 OMG 发布的，专门为实时系统设计的数据分发/订阅标准，广泛应用于国防、民航、工控等领域。

> 其的分布式通信协议：
> 
> MQTT - 消息队列遥测传输，基于发布/订阅模型，存在单点故障可能性。
> 
> SOME/IP - 车载以太网协议。

在 IOS 七层模型中的位置如下图。

<img title="" src="file:///home/dknt/Documents/notes/robotics/image/Screenshot%20from%202023-06-26%2020-47-08.png" alt="" data-align="center" width="389">

实时 Real-time 的定义：在单位时间内的跳动很小。很稳定，但未必要快。

DDS 核心标准：DDS Specification —— API 标准，保证不同 DDS 实现的应用程序的可移植性。DDSI-RTPS Specification —— 协议标注，保证不同 DDS 实现的互操作性。

QoS

现在 ROS 2 支持的 DDS 供应商有 eProsima’s Fast DDS（开源）, RTI’s Connext DDS, Eclipse Cyclone DDS（开源）, and GurumNetworks GurumDDS。默认使用 eProsima’s Fast DDS。

# 1 基本概念

**GDS** (Global Data Space)，全局数据空间。全分布式结构，无注册机，不存在单点故障。GDS 动态发现 Publisher, Subscriber, 数据及其类型。

<img title="" src="file:///home/dknt/.config/marktext/images/2023-06-26-21-45-51-Screenshot%20from%202023-06-26%2021-44-55.png" alt="" data-align="center" width="419">

DDS 应用中，GDS 称为 **Domain**。Domain 是对一族 DDS 应用程序的逻辑分组，不同 Domain 的实体相互独立，不能互相访问。每一个 Domain 有唯一的 **Domain ID**，应用程序通过 Domain ID 创建 **Domain Participant** (DP) 来获取相应 Domain 的访问权限。

DDS 数据发布最小单元为 **Topic**。Topic 三要素：数据类型，Topic 名称，QoS 策略。数据类型支持 IDL 定义的类型，如 short, long, float, string 以及 array, sequence, union, enumeration，和结构体嵌套。只有在三要素都相同的情况下，才能建立通信。

# 2 通信形式

DDS 的技术核心是以数据为核心的发布/订阅模型（Data-Centric Publish-Subscribe, DCPS）。应用程序能够通过 Topic 判断其所包含的数据类型，不必依赖其他上下文信息。DDS 能按照用户定义的方式自动进行存储、发布、订阅。

示例图如下：

<img title="" src="file:///home/dknt/.config/marktext/images/2023-06-26-21-49-41-Screenshot%20from%202023-06-26%2021-49-07.png" alt="" width="428" data-align="center">

另一张示例图如下。注意图中的从属关系。

<img title="" src="file:///home/dknt/.config/marktext/images/2023-06-26-21-52-08-Screenshot%20from%202023-06-26%2021-51-54.png" alt="" data-align="center" width="438">

数据读取有以下方式：轮询（非阻塞），阻塞，创建回调函数。

# 3 QoS Policy

QoS —— 服务质量，通过设置 QoS 策略控制数据在应用程序间的共享方式，满足不同场景下对功能和性能的需求。

提供数据可用性控制、数据交付方式控制、数据时效性控制、必须的网络和计算资源控制、定义和分发用户信息、

> DDS 测试。通信协议一致性测试。如何测试？
