# ORB SLAM2 使用记录

> ORB-SLAM3 已经问世，功能强大，加入了地图集、IMU 融合等，可以在 Ubuntu 22 下稳定运行。但相对于视觉 SLAM 任务本身，加入了许多额外的东西。所以决定先阅读 ORB-SLAM2 的源码入门。
> 
> 计划：通读源码，理解项目结构，学习其中用到的编程技巧，学习怎样在此基础上扩充自己的东西。实现稠密建图、Octree 建图，并与 ROS2 结合。
> 
> Dknt 2023.9.11
> 
> 参考：
> 
> https://gaoyichao.com/Xiaotu/?book=ORB_SLAM%E6%BA%90%E7%A0%81%E8%A7%A3%E8%AF%BB

# 数据集

* TUM Dataset 下载。

不知道为什么，从 TUM 视觉组官网进到下载页面没有下载的选项，可能是制裁俄罗斯吧。从这个网址还可以正常下载[TUM Dataset Download](https://cvg.cit.tum.de/rgbd/dataset/)。

> 只是那几天没有，可能在维护。

TUM 数据集分为三个部分 freiburg1, freiburg2, freiburg3，分别用三个不同的深度相机录制，内参和畸变不同，使用 ORB 时，对应参数文件为 TUM1, TUM2, TUM3。

# 三方库版本问题

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

# ORB-SLAM 稠密重建

基于 ORB-SLAM3 计算的轨迹进行三维稠密点云重建。

点云占用大量内存，需要进行合理内存管理。

实现实时重建时，PCL 会与某些库发生冲突，导致段错误。尝试过将 ORB 2 中所有 Viewer 和 Pangolin 相关的都去除掉，不使用 PCL，仅仅加一句链接到 PCL，运行时候也会报段错误。很奇怪，得看一下源码，看看哪些库发生冲突。

1

# UAV Stereo SLAM Dataset

无人机惯性视觉 SLAM 数据集，可以先在这个上面跑。

https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

1

# 源码

很多矩阵用的是 cv::Mat 不是 Eigen
