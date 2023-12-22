# ORB-SLAM2 笔记

> ORB-SLAM3 已经问世，功能强大，加入了地图集、IMU 融合等，可以在 Ubuntu 22 下稳定运行。但相对于视觉 SLAM 任务本身，加入了许多额外的东西。所以决定先阅读 ORB-SLAM2 的源码入门。
> 
> 计划：通读源码，理解项目结构，学习其中用到的编程技巧，学习怎样在此基础上扩充自己的东西。实现稠密建图、Octree 建图，并与 ROS2 结合。
> 
> Dknt 2023.9.11
> 
> 参考：
> 
> [无处不在的小土-index](https://gaoyichao.com/Xiaotu/?book=ORB_SLAM%E6%BA%90%E7%A0%81%E8%A7%A3%E8%AF%BB&title=index)
> 
> [一步步复现ORBSLAM2 - 随笔分类 - 小C酱油兵 - 博客园](https://www.cnblogs.com/yepeichu/category/1356379.html)

# 1 源码分析

ORB 2 中大部分矩阵用的是 cv::Mat，提供对 Eigen 接口。ORB 3 中改为 Eigen。

**注意**：追踪的结果是 T_cw，世界坐标系相对于相机坐标系的位姿，默认 z 轴向前，x 轴向右，y 轴向下。注意变换顺序。

> 以下分析全部基于适配到 OpenCV 4 的 ORB-SLAM 2

自下而上，先了解底层数据结构，再学习功能类，最后看线程。

为使表述准确、避免歧义，名字使用英文类名。

## 1.1 数据结构

1

### 1.1.1 MapPoint 地图点

地图点

MapPoint 是空间中的 Feature，可能同时被多个 Frame 观测到。类内存储了 MapPoint 的最佳描述子、指向观测 KeyFrame 的指针、以及在 KeyFrame 内对应 KeyPoint 的索引。

ORB-SLAM 地图管理策略是先大量添加 KeyFrame 与 MapPoint，后严格剔除。如何做到剔除？

### 1.1.2 Frame 帧

帧

Frame 中包含关键点

特征提取过程中的网格划分，对于单目、双目、深度图像的不同处理。双目特征点匹配，特征匹配、块匹配、亚像素精度拟合。

### 1.1.3 KeyFrame 关键帧

关键帧

关键帧由帧生成，其实可以继承自帧

### 1.1.4 Map 地图

地图

## 1.2 功能模块

### 1.2.1 ORBextractor 特征提取器

ORB 特征提取器。ORB 特征是加入了旋转不变性的 FAST 特征，通过计算灰度质心得到特征主方向，保证旋转不变性。使用 BRIEF 描述子，描述子计算时抵消了特征主方向。为了保证尺度不变，考虑了从金字塔不同层中提取的特征。

这个提取器是作者从 OpenCV 源码修改得到的，添加了图像金字塔、图像网格划分、四叉树特征筛选等。与 OpenCV 的 ORB 提取相比，特征分布更均匀，能提取到不同尺度的特征，运行速度也更快。

> 源码中特征筛选方法为“八叉树”，但每个节点实际只划分四个子节点，所以我称之为四叉树。

ORBextractor 不依赖于 ORB-SLAM 中的其他模块，我们可以把 ORBextractor 的头文件和源文件单独拿出来编译测试。

ORBextractor 特征提取主要通过 `ComputeKeyPointsOctTree` 实现，但也有另一版本的提取函数 `ComputeKeyPointsOld`，后者划分图像网格，在每个网格中提取特征点，在网格中的按特征响应进行排序，筛选最优特征。如果特征总数不够，则从有多余特征的网格中再筛选一些出来。

ORBextractor 在 Tracking 中创建，Tracking 接收到图像后将 ORBextractor 地址传给 Frame 构造函数，在构造函数中完成特征提取。ORBextractor 提取特征后，会暂时保留着上一次帧的金字塔、尺度等信息，Frame 构造函数之后的操作中会从 ORBextractor 获取这些数据使用。

* 重要成员变量

```cpp
std::vector<cv::Point> pattern;  // 描述子模板
std::vector<int> umax;  // 预先计算的 1/4 圆弧点坐标
std::vector<float> mvScaleFactor;  // 每一层的绝对尺寸比例  大于 1
std::vector<float> mvInvScaleFactor;  // 每一层的逆绝对尺寸比例  小于 1
```

#### 1.2.1.1 构造函数

传入特征总数、尺度比例、层数、FAST初始阈值、FAST小阈值。

计算尺度比例信息，计算金字塔每一层需要提取的特征总数，将 BRIEF 数组点对模板加载为 vector，计算 1/4 半圆坐标。1/4 半圆坐标确定灰度质心的计算范围。

#### 1.2.1.2 特征提取与描述子计算

运算符重载 operator()

1 调用 `ComputePyramid` 构建金字塔，金字塔图像预留了 BRIEF 计算边界。

2 调用 `ComputeKeyPointsOctTree` 提取特征。

2.1 遍历金字塔每一层，逐层提取特征。

2.1.1 首先将图像划分大小为 30*30 的网格，遍历每个网格，在网格中提取特征点，提取使用 OpenCV `FAST`，开启 NMS。如果没有提取到，则使用小阈值提取。每个网格中提取到的特征，无论来自大、小阈值，都会先将坐标从网格变换为相对于特征提取边界，再无差别地放入 vToDistributeKeys 向量中。

2.1.2 调用 `DistributeOctTree` 分配特征点

2.1.2.1 划分四叉树，直到子节点数量大于等于目标特征数，或每个子节点都只包含一个特征

2.1.2.2 保留每个子节点中响应最强的特征点，存入结果向量，返回

2.1.3 将特征坐标从相对于特征提取边界变换为相对于金字塔图像，添加层数、尺度信息。

2.2 调用 `computeOrientation` 计算特征主方向。遍历每个特征点，计算灰度质心角

3 逐层计算描述子。

3.1 调用 `GaussianBlur` 进行高斯平滑

3.2 调用 `computeDescriptors` 计算描述子。对每个特征点调用 `computeOrbDescriptor`，取模板点旋转质心角后在图像中的坐标进行比较，得到旋转无关的描述子。

3.3 将特征坐标恢复到金字塔 0 层，放入结果向量

提取后的特征点包括了在金字塔第 0 层中的坐标、方向、尺度、出自金字塔层数、响应强度。

### 1.2.2 ORBmatcher 特征匹配器

1 代码量大

描述一下不同函数的功能

如何匹配不同尺度的特征点?

### 1.2.3 优化、初始化等

1 代码量大

1 数学成分高

### 1.2.4 显示

1 代码量中

## 1.3 线程

1

### 1.3.1 Tracking

1 代码量大

### 1.3.2 LocalMapping

1 代码量中

### 1.3.3 LoopClosing

1 代码量中

### 1.3.4 Viewer

1

# 99 技巧

* std::vector 内存管理

vector 是最常用的 STL 数据结构，可以随机访问。vector 中元素在内存中连续存储，可以使用中动态分配内存，如果我们总是让 vector 自适应的分配内存，会浪费很多时间。所以在使用 vector 时，最好使用 reserve 为其预分配足够的内存。例如，在特征四叉树划分时，会为子节点的特征 vector 预分配与父节点一样大小的内存。这是一种空间换时间的方式。

# 0 其他

## 0.1 数据集

* TUM Dataset 下载。

不知道为什么，从 TUM 视觉组官网进到下载页面没有下载的选项，可能是制裁俄罗斯吧。从这个网址还可以正常下载[TUM Dataset Download](https://cvg.cit.tum.de/rgbd/dataset/)。

> 只是那几天没有，可能在维护。

TUM 数据集分为三个部分 freiburg1, freiburg2, freiburg3，分别用三个不同的深度相机录制，内参和畸变不同，使用 ORB 时，对应参数文件为 TUM1, TUM2, TUM3。

## 三方库版本问题

原版的 ORB-SLAM2 发行已经很久了，依赖的软件都比较新，更新很快，依赖关系复杂。

原版 ORB-SLAM2 主要以来以下库：

* Eigen 3.2

* Pangolin 0.5

* g2o 位于 Thirdparty

* DBoW2 位于 Thirdparty

* OpenCV 3.4

高博的稠密建图依赖于点云库

* PCL 1.7

运行于 Ubuntu 18 系统。

我的系统是 Ubuntu 22.04。使用 OpenCV 3.4.19。Eigen 版本 3.4.0。

系统自带 ffmpeg 4.0，无法编译 Pangolin 0.5，于是使用 Pangolin 0.8。这时由于 Eigen 的版本问题，编译时 Pangolin 报错。

Google said，修改 Eigen 导包时的参数可以解决这个问题。

```cmake
find_package(Eigen3 REQUIRED NO_MODULE)
```

运行时报段错误，Google said again，将 CMakeLists.txt（包括三方库目录下 g2o 和 DBoW2的）作如下修改：

```cmake
# 将
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
# 修改为，去掉 -march=native 参数
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
```

编译后，可以成功运行。

编译高博的稠密建图版本，我的 PCL 版本为 1.13.1。代码不多，原理略懂，编译成功，运行报错，真好。错误是段错误：

```shell
Segmentation fault (core dumped)
```

经测试，报错的原因是由于导入了 PCL。即使不用高博的稠密版本，在原本能运行的 ORB-SLAM 的 CMakeLists.txt 中加入 find_package(PCL)，也会导致 Segmentation fault。

猜测原因如下，导入 Eigen 时加入 `NO_MODULE` 才可以正常运行，但 PCL 的 config/find 文件中，有 find_package(Eigen3 REQUIRED) 这么一句，所以就有问题了。

理论上，应该使用 Eigen 3.2 版本，Pangolin 0.5 版本，对应 PCL 使用 1.7 版本，但后两者在我的电脑上无法编译，由于更底层的库版本太新了。

于是找到了别人移植到 OpenCV 4，Eigen 3.4 的版本 [GitHub - gaoyichao/ORB_SLAM2: Real-Time SLAM for Monocular, Stereo and RGB-D Cameras, with Loop Detection and Relocalization Capabilities](https://github.com/gaoyichao/ORB_SLAM2)。导入 PCL 后，也可以正常运行。

1

> 没有用到 Sophus。李代数是怎么实现的？

1

## ORB-SLAM 稠密重建

基于 ORB-SLAM3 计算的轨迹进行三维稠密点云重建。

点云占用大量内存，需要进行合理内存管理。

实现实时重建时，PCL 会与某些库发生冲突，导致段错误。尝试过将 ORB 2 中所有 Viewer 和 Pangolin 相关的都去除掉，不使用 PCL，仅仅加一句链接到 PCL，运行时候也会报段错误。很奇怪，得看一下源码，看看哪些库发生冲突。

1

## UAV Stereo SLAM Dataset

无人机惯性视觉 SLAM 数据集，可以先在这个上面跑。

https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

1
