# PCL

> 我最爱的 ORB_SLAM2 稠密建图跑不起来，于是学一下点云库，自己搭吧。
> 
> 了解点云处理的基本方法、流程。对于具体的算法，了解其使用场合即可。
> 
> Dknt 2023.9.1
> 
> 参考：
> 
> [Introduction &mdash; Point Cloud Library 0.0 documentation](https://pcl.readthedocs.io/projects/tutorials/en/master/#)
> 
> [PCL(Point Cloud Library)学习记录· Yuque](https://www.yuque.com/huangzhongqing/pcl/)

PCL (Point Cloud Library) 是一个开源点云处理库，包含了滤波、特征检测、表面重建、点云配准、模型匹配、语义分割等算法。遵循 BSD 协议。PCL 又分为了一系列更小的模块，这些模块可以单独编译，以减小最终项目的大小。

PCL 是点云处理中的 OpenCV，但相比与 OpenCV，PCL 的规模小很多。

> 注意版本，注意版本，注意版本。
> 
> 2023年9月5日，我电脑系统路径下安装有两个 Eigen，两个 VTK，两个 PCL ...更可怕的是，他们竟然能正常运行...

# 0 模块概述

几乎所有 PCL 中的类都继承自`pcl::PCLBase`。

PCL 中的常用模块如下：

* `Filters`——点云滤波

由于测量误差，原始点云数据中往往存在大量外点，干扰局部点估计。

**外点去除滤波器**——可以使用统计滤波的方法去除稀疏外点，计算每个点与其最临近N点的距离和，得到点云中所有点距离和的分布，假设这个分布是高斯分布，可以根据均值和标注差去除距离过大的外点。

**体素滤波器**——每个体素内保留一个点，保证点云中点的数量不会过大。

* `Features`——特征

包含3D特征提取的算法和容器。

特征可用于点云配准。点云特征是基于点的局部信息计算的。常用特征包括**形状描述子**和**几何特征**，他们都是局部描述子。好的特征应该具有如下特点：刚体变换不变性、采样密度不变性、噪声不敏感。PCL 中的特征基于两种方式计算：K 临近搜索、半径搜索。

最常用的几何特征：表面曲率和表面法线方向。

* `Keypoints`——关键点

类似于图像处理，与点云总体相比，关键点数量少很多，但可以很好地表示原始点云。

包含两种关键点检测算法。NARF，和另一种。

* `Registration`——点云配准

将几组点云拼接在一起。PCL 提供了大量适用于不同场合的配准算法。

* `KdTree`——K维树

为了方便局部特征计算，点云常按照K维树的方式存储。K维树结构有利于范围搜索和邻近搜索。对于三维空间中的点云，k为3。

基于 FLANN 实现快速临近搜索。

* `Octree`——八叉数

描述空间位置占用情况，占用内存可以灵活调整。可用于空间划分、下采样、搜索。

点云所占空间被逐层分为不同大小的体素(voxel)块。

* `Segmentation`——分割

点云聚类，非深度学习方法。能不能把图像语义分割和点云处理结合？

* `Sample Consensus`——采样一致性

包含采样一致算法，如 RANSAC。用于检测点云中的形状，如平面、圆柱、线、球体。

* `Surface`——表面

表面模块用于三维重建，可解决三种问题：平滑和重采样、网格化、凸面凹面。

平滑和重采样适用于点云噪声较大，或多次扫描没有完美对齐获得的点云。

网格化是从点云构建表面的一般方法，有两种常用方法：从原始点云快速三角化、构建网格同时平滑并填补空洞。

凸面凹面可用于确定点云边界，简化物体表示。

* `Range Image`——深度（范围）图像

从 RGBD 相机或双目相机获得的深度图像，当相机模型和参数已知时，可以恢复一组点云。

* `I/O`——输入/输出

从 pcl 文件、相机硬件中读取点云。

* `Visualization`——显示

PCL 库的显示模块基于 VTK，VTK 底层是 OpenGL。（VTK 很强大，Paraview 底层也是 VTK）

* `Common`——常规

包含 PCL 中常用的数据结构和方法。包括点云类`PointCloud`，以及点、法线、RGB 颜色向量、特征描述子等数据结构。也包含了大量常用算法，用于计算距离、法线、均值、方差、角度变换、空间变换等。

* `Search`——搜索

提供了基于不同数据结构的最邻近搜索算法。包括：k维树、八叉树、暴力匹配、对于有序数据的特殊搜索算法。

* `Binaries`——二进制

一些常用工具的可执行文件。

`pcl_viewer` 用于显示 PCD, VTK 等点云文件。

`pcl_octree_viewer` 展示点云的八叉树。

还有一些其他用于格式转换的工具。

# 1 基础

## 1.1 点云类

PCL 基本数据结构是点云类`pcl::PointCloud<PointT>`，包含如下成员变量：

* `int width` 点云宽度

无组织点云中点的数量，有组织点云一行中点的数量。有组织点云指从深度相机中获得的点云，可以从图片里得到点的相邻关系，这时临近算法运行更快。

* `int hight` 点云高度

有组织点云中的行数。

* `std::vector<PointT> points` 点集

点云中全部点

* `bool is_dense`

`points`中所有数据有限时为 true，包含 Inf/NaN 时为 false

* `Eigen::Vector4f sensor_origin_` 传感器采集来源/位置

* `Eigen::Quaternionf sensor_orientation_` 传感器采集姿态

以及如下常用成员函数：

1

> 通常情况下，通过深度相机获得的是有组织点云，通过激光雷达获得的是无组织点云

## 1.2 点类

点云类为模板类，使用时需要用具体点类型填充模板参数。全部的点类型见头文件`point_types.hpp`。用户也可以定义自己的点类型，用于执行特定的任务。

> 点类的定义用到了联合`union`的技巧。在联合中定义不同的类型，就可以通过不同的方式访问同一片内存中的内容。使用联合也可以实现内存对齐，从而使用 SSE 指令集。

常用点类型如下：

`PointXYZ` 点    注意这个点占用4个 float 的内存，便于使用 SSE 指令集加速处理。

`PointXYZI` 灰度点    占用8位 float

`PointXYZRGBA` 彩色透明点

`PointXYZRGB` 彩色点

`InterestPoint` 兴趣点    包括位置和强度

`Normal` 法线    包括法线方向和曲率

`PointNormal` 法线点

`PointXYZRGBNormal` 彩色法线点

`PointXYZINormal` 灰度法线点

`PointWithRange` 带测量的点

`PointWithViewpoint` 带视角的点

...

还包括了一些直方图和描述子的类型，详见类型声明头文件。

## 1.3 Cmake 中使用 PCL

CMakeLists.txt 模板如下：

```cmake
cmake_minimum_required(VERSION 3.0.0)
project(MY_GRAND_PROJECT)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

add_executable(my_test src/my_test.cpp)
target_link_libraries(my_test ${PCL_LIBRARIES})
```

## 1.4 PCL 与 Eigen

PCL 依赖于 Eigen。Eigen 是个只有头文件的 C++ 矩阵模板库，很神奇，很常用。

注意，在 PCL 中使用 Eigen 时，<mark>必须使用 float 类型的矩阵</mark>，否则编译报错。

> 如果报了 Eigen 相关的错，或变换结果有问题，先检查一下数据的类型是不是 float

点云坐标变换可以通过 Eigen 中的几何模块实现，<mark>变换需要使用仿射变换矩阵</mark>，不能用欧式变换矩阵。

> 仿射变换矩阵一定要初始化。

1

1

> 我感兴趣的模块：显示，配准，I/O，滤波
> 
> 其实我也不熟悉点云处理，且学且看吧

1

# Common

包含类的定义，以及许多实用的几何学、统计学函数。

> 常用头文件、函数见如下链接
> 
> https://www.yuque.com/huangzhongqing/pcl/uv5cn9

# KD-tree

> 理论基础
> 
> https://zhuanlan.zhihu.com/p/23966698
> 
> 代码参考
> 
> https://www.yuque.com/huangzhongqing/pcl/xr8761

KD 树是一种二叉树，用于分割 k 维空间。对点按 k 维中的某一维度排序，以中位数作为根节点，将剩余点分为左右两颗子树，再对子树按另一纬度排序、分割，直到无法分割子树。

KD 树用于范围搜索和最近邻搜索，过程比较复杂，但相比于遍历全部点云时间复杂度小很多。

PCL 中 KD 树的数据结构是`pcl::KdTree<PointT>`，提供了基于 FLANN 进行搜索的子类与包装类。

`pcl::search::KdTree<PointT>`

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h> // 包含kdtree头文件
typedef pcl::PointXYZ PointT;
int main() {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("read.pcd", *cloud);
    // 定义KDTree对象
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    kdtree->setInputCloud(cloud); // 设置要搜索的点云，建立KDTree
    std::vector<int> indices; // 存储查询近邻点索引
    std::vector<float> distances; // 存储近邻点对应距离的平方
    PointT point = cloud->points[0]; // 初始化一个查询点

    // 查询距point最近的k个点
    int k = 10;
    int size = kdtree->nearestKSearch(point, k, indices, distances);
    std::cout << "search point : " << size << std::endl;
    // 查询point半径为radius邻域球内的点
    double radius = 2.0;
    size = kdtree->radiusSearch(point, radius, indices, distances);
    std::cout << "search point : " << size << std::endl;
    return 0;
}
```

> 搜索结果默认是按照距离point点的距离从近到远排序；如果InputCloud中含有point点，搜索结果的的第一个点是point本身。

`pcl::KdTreeFLANN<PointT>`

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/kdtree/kdtree_flann.h>
typedef pcl::PointXYZ PointT;
int main() {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("read.pcd", *cloud);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; //创建KDtree
    kdtree.setInputCloud(cloud); // 设置要搜索的点云，建立KDTree
    std::vector<int> indices; // 存储查询近邻点索引
    std::vector<float> distances; // 存储近邻点对应距离的平方
    PointT point = cloud->points[0]; // 初始化一个查询点

    // 查询距point最近的k个点
    int k = 10;
    int size = kdtree.nearestKSearch(point, k, indices, distances);
    std::cout << "search point : " << size << std::endl;
    // 查询point半径为radius邻域球内的点
    double radius = 2.0;
    size = kdtree.radiusSearch(point, radius, indices, distances);
    std::cout << "search point : " << size << std::endl;
    return 0;
}
```

1

# Octree

`pcl::octree::Octree2BufBase<LeafContainerT, BranchContainerT>`

1

# Features

> 具体代码见例程

PCL 中特征检测器是一个类，我们需要向其传入输入点云、搜索下标、搜索层（后两者为可选，不传入代表在输入点云中为每个点计算特征）。搜索下标指提取这些点的特征，搜索层指以这些点为基础提取特征。

```cpp
setSearchSurface(); // 设置搜索层，以这些点为基础计算特征
setInputCloud(); // 设置输入点云，计算这些点的特征
setIndices(); // 设置搜索下标，计算输入点云中这些标号对应的点的特征
```

常用的特征如下：

```cpp
#include <pcl/features/normal_3d.h> // 概率方法估计法线
#include <pcl/features/integral_image_normal.h> // 积分图求法线 有组织点云
// 曲率半径
// 点特征直方图
// 快速点特征直方图
// 估视点特征直方图
// NARF特征
// 惯性矩和偏心率的描述符
// RoPs
// GADS
```

## 法线估计

使用传感器对真实平面进行扫描时会得到在平面附近分布的点，有两种方法来获取点的法线方向。第一种，通过点云恢复网格，计算网格的法线方向；第二种，利用统计方法直接推断法线方向。

统计方法过程如下，以某个点为中心，计算这个点 k 个近邻或 半径 r 内点的坐标均值和协方差。计算协方差矩阵的特征值和特征向量，其中最小的特征值对应的特征向量代表了这一点的法线方向。如果这个向量指向视点，则正确，否则需要对这个向量取反。

曲率为最小特征值除以特征值之和。

> 原理是这样，实际会调库就行。

## PFH 点特征直方图描述子

PFH (Point Feature Histograms) 法线和曲率维数太小，不能很好地描述一个点，需要更复杂的特征。

PFH 计算基于法线，对点云对应曲面的6维姿态来说它具有不变性，并且在不同的采样密度或邻域的噪音等级下具有鲁棒性。

> PCLHistogramVisualization 可用于显示直方图

还有点特征直方图的加速版本 FPFH (Fast PFH)。

## VFH 视角特征直方图

VFH (Viewpoint feature h) 用于聚类识别和位姿估计。用从不同方向采集的点云来训练一个模型，给模型输入一组点云，模型输出位姿。

## NARF 法线对齐的径向特征

NARF (normal aligned radial feature) 一种3D特征点检测和描述算法，适用于深度图像。考虑表面稳定性，考虑物体边缘处。

特点：

* 提取出边缘点，这些点往往更加具有稳定性和可重复性

* 提取出Normal稳定的，也就是一些平面相对光滑平面和共面的点

* 所有操作都是基于2D的range image，计算量相比于直接操作点云要小

## 基于惯性矩和偏心矩的特征

1

## RoPs 旋转投影统计特征

RoPs (Rotational Projection Statistics)

1

## GASD 全局对齐空间描述子

GASD (Globally Aligned Spatial Distribution)

1

# Filtering

点云处理中需要运用滤波算法的情况有以下几种：

1. 点云数据密度不规则需要平滑 

2. 因为遮挡等问题造成离群点需要去除

3. 重复数据过多需要下采样

4. 噪声数据需要去除

提供的滤波方案有以下几种：

1. 按照给定的规则限制过滤去除点

2. 通过常用滤波算法修改点的部分属性

3. 对数据进行下采样

有序点云滤波方法有双边滤波、高斯滤波、中值滤波等

无序点云滤波方法有体素栅格、随机采样等

## 直通滤波器

PassThrough 直通滤波器原理很简单，当点位于某指定区间内时，保留/去除这个点。可以提取某空间范围内点云。

## 体素滤波器降采样

创建一个三维体素栅格，容纳后每个体素内用体素中所有点的重心来近似显示体素中其他点，这样该体素内所有点都用一个重心点最终表示，对于所有体素处理后得到的过滤后的点云，这种方法比用体素中心逼近的方法更慢，但是对于采样点对应曲面的表示更为准确。

## 统计外点去除滤波

统计近邻点分布，按照一定准则去除这些点。

## 将点投影到参数化模型上

将点投影到如平面、球体等参数化模型上。

1

# I/O

常用内容

```cpp

```
