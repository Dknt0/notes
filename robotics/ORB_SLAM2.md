# ORB-SLAM2 笔记

> 在我开始学习 ORB-SLAM2 时，ORB-SLAM3 已问世三年。相比 ORB2，ORB3 完善了代码结构，扩充了如地图集 (Atlas)、IMU-PreIntegration、Camera Model 等功能，但代码量是前作的三倍。在纯视觉 SLAM 方面，ORB2 与 ORB3 没有太大区别，所以我选择从 ORB2 学起。
> 
> ORB 是 SOTA 的开源框架，源码阅读，学习算法为次，主要是学习编程技巧、项目组织方式，编写高效实用的 C++ 代码，设计适当的类、协调各个模块之间的工作。
> 
> 这会耗费大量的时间、精力，希望能有所收获。
> 
> Dknt 2023.9.11
> 
> 参考：
> 
> [无处不在的小土-index](https://gaoyichao.com/Xiaotu/?book=ORB_SLAM%E6%BA%90%E7%A0%81%E8%A7%A3%E8%AF%BB&title=index)
> 
> [一步步复现ORBSLAM2 - 随笔分类 - 小C酱油兵 - 博客园](https://www.cnblogs.com/yepeichu/category/1356379.html)

# 1 源码分析

ORB 2 中大部分矩阵用的是 cv::Mat，提供对 Eigen 接口。ORB 3 中改为 Eigen，更规范化。

自下而上，先了解底层数据结构，再学习功能类，最后看线程。

找出类中关键的成员变量；找出与类关联的类，以及关联的方式与目的，给出理由；确定每种锁的使用情景。

为使表述准确、避免歧义，名词使用英文类名。

```
伪代码
L 循环
? 判断
C 调用
```

## 1.1 数据结构

ORB-SLAM 中核心数据库是 Map 与重定位数据库。底层由 KeyFrame, Frame, MapPoint 以及 DBow2 提供的类实现。

Map 由全部 KeyFrame, MapPoint 组成，并包含有抽象的数据结构——生成树 (Spanning Tree) 和共视图 (Covisibility Graph)。生成树与共视图没有专门的数据结构。生成树通过 KeyFrame 中父、子节点成员变量实现；共视图通过 KeyFrame 中共视帧向量实现。论文中提到的本质图 (Essential Graph)，实际上是共视图保留共视大于某阈值的边得到的。

重定位数据库由视觉词典 (Vocabulary) 和 KeyFrameDatabase 实现。其中，视觉辞典是由外部库 DBow2 实现的，KeyFrameDatabase 基于 KeyFrame。

### 1.1.1 MapPoint 地图点

MapPoint 是空间中的特征点，是 ORB-SLAM 地图基本元素之一。

MapPoint 自身的信息包括空间位置、描述子、观测方向、尺度无关距离。MapPoint 源自于                 二维特征点，带有描述子信息，而图像特征通常只在某个方向、一定距离范围内可以被观测到，因此需要上述信息进行特征匹配。

一个 MapPoint  可能被多个 KeyFrame 观测到，需要记录这种观测关系，并提供双向查询的方法。ORB 中 MapPoint 与 KeyFrame 的关联以 KeyFrame 中的 KeyPoint 为准，在 KeyFrame 中建立了指向 MapPoint 的指针向量，按 KeyPoint 顺序进行索引；MapPoint 也建立了 KeyFrame 映射关系，记录观测到当前点的 KeyFrame 以及当前点在 KeyFrame 中对应的 KeyPoint 序号。这种双向关系是通过普通指针实现的，普通指针可以直接作为映射中的 keyword。利用双向关系，有利于实现共视查询：给定一关键帧，遍历其观测地图点，再遍历这些地图点的观测关键帧，并记录共视数，就得到了共视关键帧。

由于一个 MapPoint 可能被多个关键帧观测到，需要选择最优的描述子与参考关键帧，这会在下文中提及。

访问成员变量时，会先上锁，将成员变量内容拷贝，解锁，再使用。

**重要成员变量**

```cpp
cv::Mat mWorldPos;  // 绝对坐标
cv::Mat mDescriptor;  // 最佳描述子  与其他描述子汉明距离中位数最小的描述子作为最佳描述子
std::map<KeyFrame*, size_t> mObservations;  // 观测关键帧集  映射<关键帧指针, 此地图点在此关键帧的特征点集合中的序号>
cv::Mat mNormalVector;  // 平均观测方向  相机指向地图点
int nObs;  // 被观测次数
KeyFrame* mpRefKF;  // 参考关键帧
float mfMinDistance;  // 最近尺度无关距离
float mfMaxDistance;  // 最远尺度无关距离
int mnVisible;  // 可视次数  可视代表地图点位于帧视野范围内，但未必能成功提取特征  可视通过 Frame::isInFrustum 判断
int mnFound;  // 检测次数  检测代表地图点在某个帧内能成功提取特征，成为关键点
// 存放了一些在三线程中使用的 public 变量
```

**互斥锁**

```cpp
std::mutex mMutexPos;  // 位姿锁
std::mutex mMutexFeatures;  // 特征锁
static std::mutex mGlobalMutex;  // 特征点全局锁
```

**和 MapPoint 关联的类**

1. Map 地图——地图包含所有地图点

2. KeyFrame 关键帧——关键帧和地图点之间存在观测关系

3. 三线程

#### 1.1.1.1 构造函数

MapPoint 有两种构造函数，从 KeyFrame 构造和从 Frame 构造。

构造 MapPoint 时需要提供点的空间位置，即点必须被三角化(Monocular)，或深度有效(Stereo, RGB-D)。

* 从 KeyFrame 构造 MapPoint

设置观测帧 id，设置空间位置、观测方向、序号

* 从 Frame 构造 MapPoint

设置观测帧 id，设置空间位置、观测方向、序号，计算最近和最远尺度无关距离

**最近和最远尺度无关距离**——这个距离对应的是 ORB 可以在图像中被成功检测、正确匹配的范围。当地图点位于最远观测距离时，它应该在金字塔 0 层被检测到；当位于最近观测距离时，它应该在金字塔最上层被检测到。由此，获取地图点在对应帧中的金字塔层数，用当前层和最底层、最顶层间的比例乘特征点深度，可以得到上述尺度无关距离。

#### 1.1.1.2 最佳描述子计算

`ComputeDistinctiveDescriptors()`

遍历所有观测关键帧中对应关键点的描述子，计算他们之间的汉明距离，选用与其他描述子汉明距离中位数最小的描述子作为最佳描述子

```
1 从观测关键帧中获取地图点对应的描述子
2 计算这些描述子的汉明距离矩阵
3 选用与其他描述子汉明距离中位数最小的描述子作为最佳描述子
```

#### 1.1.1.3 地图点替换

`Replace(MapPoint* pMP)`

其实是地图点继承，用于地图点融合，新地图点会继承当前点与关键帧之间的观测关系。

```
1 获取旧点观测信息，设置坏点标志位
2 遍历旧点观测集，新点继承旧点观测
  1 新点没有被关键帧观测到 ?
    T 新点添加关键帧，关键帧替换旧点为新点
    F 清除关键帧对旧点观测
3 新点继承检测次数和可视次数
4 计算最佳描述子 ComputeDistinctiveDescriptors C
5 从地图删除当前点
```

#### 1.1.1.4 更新法线和深度

`UpdateNormalAndDepth()`

当观测关系改变后会调用此函数，用于更新地图点平均观测方向和尺度无关距离

```
1 遍历所有观测，累加地图点相对于相机的平均方向，除以观测总数
2 计算相对于参考关键帧的距离，按照金字塔比例确定尺度无关距离
```

### 1.1.2 Frame 帧

Frame 是 SLAM 中的帧，输入图像的信息被提取后，创建 Frame 对象存储其信息。为节省内存 Frame 不会直接存放图像。Frame 通常是当前帧，如果满足关键帧条件，则会构造关键帧，存于 Map 中。

Frame 主要用于处理 Tracking 中的图，提取特征，去除畸变，将特征分配给网格。

Frame 中包含关键点。

* 畸变处理在哪一步?

* 计算中使用的是去畸变关键点？

**重要成员变量**

```cpp
/* 位姿 序号 时间 */
long unsigned int mnId;  // 帧 id
cv::Mat mTcw;  // 相机位姿
/* 特征相关 */
std::vector<cv::KeyPoint> mvKeysUn;  // 左图去畸变关键点
std::vector<MapPoint*> mvpMapPoints;  // 关联到特征点的地图点
cv::Mat mDescriptors, mDescriptorsRight;  // ORB 描述子  左, 右
std::vector<float> mvuRight;  // 特征点右图 u 坐标，双目点第三维坐标  单目取-1
std::vector<float> mvDepth;  // 深度  单目取-1
std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];  // 去畸变左图网格 (x)(y)(vector<关键点序号>)
std::vector<cv::KeyPoint> mvKeys;  // 左图畸变关键点
std::vector<cv::KeyPoint> mvKeysRight;  // 右图畸变关键点
/* 词袋 */
DBoW2::BowVector mBowVec;  // 视觉描述向量
DBoW2::FeatureVector mFeatVec;  // 特征向量
```

特征提取过程中的网格划分，对于单目、双目、深度图像的不同处理。

双目特征点匹配，特征匹配、块匹配、亚像素精度拟合。

#### 1.1.2.1 构造函数

Frame 构造函数分为四类：拷贝构造函数 、单目构造函数、双目构造函数、深度构造函数。

* 拷贝构造函数

复制信息，id 也是拷贝的

* 单目构造函数

1

* 双目构造函数

输入带畸变的双目图像，构造 Frame 对象。

首先对图像进行特征提取，计算描述子。提取特征是在畸变图像上进行的，之后会将提取结果去畸变，得到真实像素坐标。

双目视差不是直接计算得出的，构造函数中先提取了左、右图像特征点，进行特征匹配，再进行匹配点对之间的视差。视差的计算比较复杂，结果为亚像素精度，由`ComputeStereoMatches`函数实现。

最后，会将提取到的特征点分配到特征网格中。

```
1 从左提取器获取金字塔尺度信息
2 开双线程，在畸变图像上提取 ORB 特征，计算描述子  `ExtractORB` C
  1 调用左图或右图 ORB 提取器，计算畸变特征与其描述子，存于向量中
3 计算关键点的去畸变坐标  `UndistortKeyPoints` C
  1 根据畸变参数，计算左图关键点的去畸变坐标
4 匹配双目特征点，计算右点坐标  `ComputeStereoMatches` C
5 如果是初始化过程，计算去畸变图像边界  `ComputeImageBounds` C
  1 计算去畸变图像边界位置，根据畸变类型不同，边界可能位于图像外
6 分配特征到网格  `AssignFeaturesToGrid` C
  1 按照去畸变坐标计算特征点对应网格标号，记录特征点序号到网格向量中
```

* 深度构造函数

1

#### 1.1.2.2 双目匹配计算

`ComputeStereoMatches()`

匹配双目特征点，确定右点坐标与深度。

> **此函数中使用的左、右特征点都是带畸变的，这样匹配出的视差貌似有点问题**

ORB-SLAM 中的双目匹配不是逐像素的稠密匹配，而是基于特征点在左、右图特征点之间进行。双目匹配点对应满足以下条件：匹配点对位于同一行；左点坐标位于右点之右；视差应在合理范围之内；特征尺度应该相同。也许是考虑标定、畸变带来的误差，在此函数中，对于一个左图特征点，会在右图相邻几行中，遍历视差在合理范围内，且特征尺度（即金字塔层数）相差不超过 1 的特征点，寻找最佳的特征点匹配。上述“合理范围”中，视差范围仅与内参有关，相邻行范围与特征尺度有关，尺度相差范围为固定值 1.

如果特征点对描述子汉明距离小于阈值，则会通过 SAD (Sum of absolute differences) 横向滑动窗口块匹配的方法，寻找最佳块匹配结果。再依据最佳匹配块对以及相邻块对的 SAD 值，通过抛物线拟合 (Parabola fitting) 的方法，确定亚像素精度的匹配结果。

```
1 遍历右图特征点，按行号将右点放入对应的几个候选特征点向量中
2 遍历左图特征点  L
  1 对一个左图特征点，遍历对应行号右图候选特征点，对右点中视差、尺度在合理范围内的，计算描述子汉明距离，记录最佳匹配点对及距离
  2 如果最佳匹配距离小于阈值，进行块匹配
    1 将左、右点坐标都变换到左点金字塔对应层上
    2 从 ORBextractorLeft 获取左点在对应层上的像素块，对像素快进行归一化，即减去中心像素值
    3 在滑动窗口范围内，按照右点坐标，从 ORBextractorRight 获取像素块，归一化，计算与左像素块的 SAD，记录最佳匹配
    4 对最佳匹配块进行抛物线拟合，并将结果变换到金字塔 0 层上，记录块匹配误差、亚像素精度右点坐标
3 匹配点对筛选，依据块匹配误差对匹配结果排序，按照误差中值剔除误差过大的匹配对
```

### 1.1.3 KeyFrame 关键帧

关键帧  **存疑**

KeyFrame 由 Frame 生成，可以认为是继承自 KeyFrame

KeyFrame 内部通过指针实现了 SpanningTree 、CovisibilityGraph 数据结构，实现了和 MapPoint 的关联

这些数据结构是如何实现的，提供了哪些接口？应该是在 LocalMapping 中调用的，先看下线程函数吧

**重要成员变量**

```cpp
/* 继承 Frame 中大部分成员变量，包括关键点、右点坐标、深度、描述子等 */
...
/* 地图点相关 */
std::vector<MapPoint*> mvpMapPoints;  // 关联到关键点的地图点 按关键点索引，未关联到地图点的关键点赋 nullptr
/* 共视图相关 */
std::map<KeyFrame*,int> mConnectedKeyFrameWeights;  // 共视帧与共视数 map<pKeyFrame, weight>
std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;  // 有序共视帧
std::vector<int> mvOrderedWeights;  // 有序权重
/* 生成树与回环边相关 */
KeyFrame* mpParent;  // 父关键帧
std::set<KeyFrame*> mspChildrens;  // 子关键帧
std::set<KeyFrame*> mspLoopEdges;  // 回环边
/* 词袋相关 */
DBoW2::BowVector mBowVec;  // 视觉单词向量  map<单词序号, 单词值>
DBoW2::FeatureVector mFeatVec;  // 特征向量
```

**和 KeyFrame 关联的类**

1. MapPoint

2. Map

1

与地图点的关系

生成树的实现

共视图的实现

### 1.1.4 KeyFrameDatabase 关键帧数据库

关键帧数据库 **存疑**

**重要成员变量**

```cpp
std::vector<list<KeyFrame*> > mvInvertedFile;  // 包含单词的关键帧列表
```

#### 1.1.4.1 检测回环候选关键帧

1

#### 1.1.4.2 检测重定位候选关键帧

1

### 1.1.5 Map 地图

地图，是 MapPoint, KeyFrame 的集合 **存疑**

地图是关键帧和地图点的集合，本身比较简单，没有特别复杂的成员函数。

**重要成员变量**

```cpp
std::set<MapPoint*> mspMapPoints;  // 地图点集合
std::set<KeyFrame*> mspKeyFrames;  // 关键帧集合
std::vector<MapPoint*> mvpReferenceMapPoints;  // 参考地图点集
```

1

## 1.2 功能模块

### 1.2.1 ORBextractor 特征提取器

ORB 特征提取器。ORB 特征是加入了旋转不变性的 FAST 特征，通过计算灰度质心得到特征主方向，保证旋转不变性。使用 BRIEF 描述子，描述子计算时抵消了特征主方向。为了保证尺度不变，考虑了从金字塔不同层中提取的特征。

这个提取器是作者从 OpenCV 源码修改得到的，添加了图像金字塔、图像网格划分、四叉树特征筛选等。与 OpenCV 的 ORB 提取相比，特征分布更均匀，能提取到不同尺度的特征，运行速度也更快。

> 源码中特征筛选方法命名为“八叉树”，但每个节点实际只划分四个子节点，所以我称之为四叉树。

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

算法

```
1 调用 `ComputePyramid` 构建金字塔，金字塔图像预留了 BRIEF 计算边界。
2 调用 `ComputeKeyPointsOctTree` 提取特征  C
  1 遍历金字塔每一层，逐层提取特征  L
    1 首先将图像划分大小为 30*30 的网格，遍历每个网格，在网格中提取特征点，提取使用 OpenCV `FAST`，开启 NMS。如果没有提取到，则使用小阈值提取。每个网格中提取到的特征，无论来自大、小阈值，都会先将坐标从网格变换为相对于特征提取边界，再无差别地放入 vToDistributeKeys 向量中。
    2 调用 `DistributeOctTree` 分配特征点  C
      1 划分四叉树，直到子节点数量大于等于目标特征数，或每个子节点都只包含一个特征
      2 保留每个子节点中响应最强的特征点，存入结果向量，返回
    3 将特征坐标从相对于特征提取边界变换为相对于金字塔图像，添加层数、尺度信息
  2 调用 `computeOrientation` 计算特征主方向。遍历每个特征点，计算灰度质心角
3 遍历每一层逐层计算描述子  L
  1 调用 `GaussianBlur` 进行高斯平滑
  2 调用 `computeDescriptors` 计算描述子。对每个特征点调用 `computeOrbDescriptor`，取模板点旋转质心角后在图像中的坐标进行比较，得到旋转无关的描述子。
  3 将特征坐标恢复到金字塔 0 层，放入结果向量
```

提取后的特征点包括了在金字塔第 0 层中的坐标、方向、尺度、出自金字塔层数、响应强度。

### 1.2.2 ORBmatcher 特征匹配器

1 代码量大

包含不同模块中用到的不同匹配方法。描述一下不同函数的功能

如何匹配不同尺度的特征点?

### 1.2.3 Optimizer 优化器

ORB-SLAM 中优化器使用 g2o

> 有空可以试着改成 Ceres

1

地图点作为因子图顶点时开启了边缘化选项，这是为什么？可能是在其增量小于一定阈值后视作固定点使用。

关键帧对地图点的图优化构建过程中，每一个观测的信息矩阵与对应关键点所在的金字塔层数有关，这个层数并不能直接与地图点深度等价，相当于只考虑了特征提取的误差，而没有考虑深度相机本身的测量噪声。对于 Intel D435i 相机，这里可能会引入一个较大的误差。

多次优化方法。先优化，然后重新分类内点、外点，再优化。

动态类型转换

1

使用到的 g2o 类

顶点：

```cpp
g2o::VertexSE3Expmap  // g2o SE3 类
g2o::VertexSBAPointXYZ  // g2o v3d 类
g2o::VertexSim3Expmap  // g2o Sim3 类
```

边：

```cpp
g2o::EdgeSE3ProjectXYZ  // 空间点到成像平面
g2o::EdgeStereoSE3ProjectXYZ  // 空间点到双目成像平面
g2o::EdgeSE3ProjectXYZOnlyPose  // 空间点到成像平面 Motion-only
g2o::EdgeStereoSE3ProjectXYZOnlyPose  // 空间点到双目成像平面 Motion-only
g2o::EdgeSim3  // 本质图 Sim(3)
g2o::EdgeSim3ProjectXYZ  // Sim3 投影
g2o::EdgeInverseSim3ProjectXYZ  // Sim3 逆投影
```

1

光束法平差

全局光束法平差

局部光束法平差

单帧位姿优化

本质图优化

相似变换群优化

1

### 1.2.4 PnPsolver PnP 求解器

1

### 1.2.5 Sim3Solver Sim(3) 求解器

1

### 1.2.6 Initializer 初始化器

g2o优化、Sim3优化、PnP求解、初始化

1 代码量大

1 数学量大

### 1.2.7 词袋

1 外部库，有就好，改进词袋模型对系统影响不大

描述向量与特征向量的计算

### 1.2._ 显示

1 代码量中

基本为功能代码，可有可无

## 1.3 线程

1

### 1.3.1 Tracking 追踪

1 代码量大

### 1.3.2 LocalMapping 局部建图

1 代码量中

### 1.3.3 LoopClosing 回环检测

1 代码量中

### 1.3.4 Viewer 显示

1

# 2 应用

调用 System 类

**注意**：追踪的结果是 T_cw，世界坐标系相对于相机坐标系的位姿，默认 z 轴向前，x 轴向右，y 轴向下。注意变换顺序。

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

原版的 ORB-SLAM2 发行已经很久了，依赖的软件更新很快，依赖关系复杂。

原版 ORB-SLAM2 主要依赖于以下库：

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

## 与深度学习的结合

SLAM 大多是用 C++ 写的，所以许多 SLAM 与深度学习结合的方式是使用 Caffe。但 Caffe 已经不更新了，对 Python3, Cudnn8 的支持都很差，所以大概率还是用 Pytorch 写。
