# VINS-Fusion

> 香港科技大学航空机器人组开源 VISLAM 框架 VINS-Fusion 记录
> 
> 参考
> 
> [GitHub - HKUST-Aerial-Robotics/VINS-Fusion: An optimization-based multi-sensor state estimator](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
> 
> Dknt 2023.11

相机外参估计，IMU 零偏估计。

光流特征？

移动初始化，失效恢复。

IMU 预积分。

基于滑动窗口的紧耦合优化。

回环检测。

四自由度位姿图优化，认为 IMU 给出了准确的俯仰角和滚转角估计。

？关键帧选择？

1

VINS-Fusion 为三线程结构，三线程分别如下：

* `vins_estimator` 惯性视觉里程计

* `loop_fusion` 回环检测

* `global_fusion` 全局传感器（GNSS）融合

其中，vins_estimator 为最基本的视觉里程计线程，使用滑动窗口法求解当前帧位姿，也是代码量最大的部分。回环与全局部分可以选择性运行。

**VINS 可以在运行时校正相机内参。**

VINS-Fusion 完全依赖 ROS，脱离 ROS 无法运行，导致向 ROS 2 的移植会比较困难。

在 VINS 的基础上可以做视觉外力估计，这是个方向。

> 很有趣，在 KITTI 官网排行榜上，VINS-Fusion 的精度是超过 ORB-SLAM2 的（2023.11.27），这说明 ORB-SLAM2 中的中期数据融合似乎没有起到太大作用，里程计精度的关键在于滑动窗口，全局轨迹精度的关键在于回环检测。并且，全局 BA 相比位姿图优化对精度并没有太大的提升，却消耗了大量的算力。
> 
> 因此，我们的系统初步拟定以滑动窗口为主，回环检测也是必要的。为了降低跟踪失效的概率，需要融合 IMU 信息。优化器选择前途光明的 Ceres。系统不依赖于 ROS，但提供对 ROS 的接口。

# 0 代码框架与编程风格

VINS 源码中大量使用了全局变量，例如 parameters.cpp, visualization.cpp 中，包含大量全局变量，并且没有对变量的保护锁。这些全局变量最好按功能封装成类。

# 1 视觉里程计 vins_estimator

功能包 src 下有三个 cpp 文件，分别为一个 ROS 接口节点、两个数据集测试节点。

其余文件夹对应如下模块：

* `estimator` 观测器类 (estimator)、特征管理、参数加载 (全局变量)

* `factor` 实现各种因素对应的 Ceres 残差类、流形类

* `featureTracker` 特征提取（光流）

* `initial` 初始化工具

* `utility` ROS 发布者、数学工具。

1

## 1.1 里程计入口

见 `vins_estimator/src/rosNodeTest.cpp`。

入口中读取配置文件，初始化观测器、发布者。

系统为数据驱动，三类数据：图像、IMU、特征。分别调用 inputImage, inputIMU, inputFeature。特征不知道是哪里来的。

提供了重置系统、切换运行模式的接口。

## 1.2 Estimator 类

观测器类，类似于 ORB 中的 Tracking。

Estimator 可以将 processMeasurements 函数运行为一个单独的线程，用于处理数据，也可以设置为数据驱动模式，当数据到来时，在相应的处理函数中调用 processMeasurements。

> 为什么要搞这个多线程......

1

特征类型为`pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>`，

inputFeature 可能不会被调用。意义何在？imputImage 中会提取图像特征，并将特征放入缓存队列，为什么要额外接收特征？

1

注意各种 CostFunction 中信息矩阵的设置

1

> 边缘化对于关键帧剔除的意义？

1

## 1.3 FeatureTracker 类

VINS 使用 Shi-Tomasi 特征，是基于光流法的 VSLAM。据说上限低，不如 ORB，且回环检测可能存在问题。可能速度快吧。

检测的主要函数是 trackImage，输入图像，返回一组特征状态，光流相关的，这个不太懂。

返回的特征是这个类型的：`map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>`

1

# 2 回环检测 loop_fusion

1

# 3 全局传感器融合 global_fusion
