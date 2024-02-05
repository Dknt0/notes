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

ORB-SLAM 的用户接口是 System 类。使用 ORB 时，需要在自己的代码中创建 System 对象，用词袋路径、配置文件路径进行初始化，启动三线程。之后只要将输入图像传给 System ，System 会就会返回追踪结果 Tcw。

为支撑 VSLAM 系统运行，ORB 源码除 System 外还有许多类，他们大致可以分为以下三种：数据类、功能类、线程类。数据类用于存放数据，如地图点、关键帧，系统运行过程中会在堆区实例化大量数据类对象，因此数据类中应仅存放重要的信息；数据类还应为功能类、线程类提供增、删、查、改等接口。功能类用于实现特征提取、特征匹配、初始化、优化等算法，他们通常会在线程类中被实例化，作为某线程类的成员变量存在。线程类对应独立运行的线程，负责处理输入数据、调用适当的算法工具类、并与其他线程协作。线程类由 System 类统一管理。

按照自下而上的思路，先了解底层数据结构，再学习功能类，最后学习线程类是如何将整个系统组织起来的。

找出类中关键的成员变量；找出与类关联的类，以及关联的方式与目的，给出理由；确定每种锁的使用情景。

为使表述准确、避免歧义，名词使用英文类名。

> 关于“集合”的描述
> “集合”是一个抽象数学概念，代表若干不重复、无序元素的整体。由于不同“集合”可能具备不同的性质，在 C++ 中会使用不同的容器来存放，如 vector, list, map, set 等。“集合”与 C++ set 可能混淆，于是用“集”来代替抽象“集合”，集合指 C++ std::set

> 读源码，写注释。写一段时间记得编译测试一下看，有的人 (没错就是我)，写个注释都能给源码弄出 bug 来...

```
伪代码
L 循环
? 判断
C 调用
```

> 伪代码概括程序整体结构，不完全准确，忽略了许多细节

## 1.1 数据类

ORB-SLAM 中核心数据库是 Map 与重定位数据库。底层由 KeyFrame, Frame, MapPoint 以及 DBow2 提供的类实现。

Map 由全部 KeyFrame, MapPoint 组成，并包含有抽象的数据结构——生成树 (Spanning Tree) 和共视图 (Covisibility Graph)。生成树与共视图没有专门的数据结构。生成树通过 KeyFrame 中父、子节点成员变量实现；共视图通过 KeyFrame 中共视帧向量实现。论文中提到的本质图 (Essential Graph)，实际上是共视图保留共视大于某阈值的边得到的。

重定位数据库由视觉词典 (Vocabulary) 和 KeyFrameDatabase 实现。其中，视觉辞典是由外部库 DBoW2 实现的，KeyFrameDatabase 基于 KeyFrame。

### 1.1.1 MapPoint 地图点

MapPoint 是空间中的特征点，是 ORB-SLAM 地图基本元素之一。

MapPoint 自身的信息包括空间位置、描述子、观测方向、尺度无关距离。MapPoint 源自于二维特征点，带有描述子信息，而图像特征通常只在某个方向、一定距离范围内可以被观测到，因此需要上述信息进行特征匹配。

一个 MapPoint 可以被多个 KeyFrame 观测到，一个 KeyFrame 也可以观测到多个 MapPoint，需要记录这种观测关系，并提供双向查询的方法。ORB 中 MapPoint 与 KeyFrame 的关联以 KeyFrame 中的 KeyPoint 为准，在 KeyFrame 中建立了指向 MapPoint 的指针向量，按 KeyPoint 顺序进行索引；MapPoint 也建立了 KeyFrame 映射关系，记录观测到当前点的 KeyFrame 以及当前点在 KeyFrame 中对应的 KeyPoint 序号。这种双向关系是通过普通指针实现的，普通指针可以直接作为映射中的 keyword。利用双向关系，有利于实现共视查询：给定一关键帧，遍历其观测地图点，再遍历这些地图点的观测关键帧，并记录共视数，就得到了共视关键帧。

由于一个 MapPoint 可能被多个关键帧观测到，需要选择最优的描述子、计算平均观测方向，并在观测信息更新时同步更新这两个变量。

MapPoint 成员函数中，访问成员变量时，会先上锁，将成员变量内容拷贝，解锁，再使用变量进行计算。

**重要成员变量**

```cpp
/* 时间 序号 位置 */
long unsigned int mnId;  // 地图点序号
cv::Mat mWorldPos;  // 绝对坐标
/* 特征与观测 */
cv::Mat mDescriptor;  // 最佳描述子  与其他描述子汉明距离中位数最小的描述子作为最佳描述子
std::map<KeyFrame*, size_t> mObservations;  // 观测关键帧集  映射<关键帧指针, 此地图点在此关键帧的特征点集合中的序号>
cv::Mat mNormalVector;  // 平均观测方向  相机指向地图点
float mfMinDistance;  // 最近尺度无关距离
float mfMaxDistance;  // 最远尺度无关距离
KeyFrame* mpRefKF;  // 参考关键帧
/* 观测统计 */
int nObs;  // 被观测次数  双目点(包括深度点)算两次观测 单目点算一次观测
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

#### 1.1.1.2 ComputeDistinctiveDescriptors 最佳描述子计算

`void MapPoint::ComputeDistinctiveDescriptors()`

遍历所有观测关键帧中对应关键点的描述子，计算他们之间的汉明距离，选用与其他描述子汉明距离中位数最小的描述子作为最佳描述子

```
1 从观测关键帧中获取地图点对应的描述子
2 计算这些描述子的汉明距离矩阵
3 选用与其他描述子汉明距离中位数最小的描述子作为最佳描述子
```

#### 1.1.1.3 Replace 地图点替换

`void MapPoint::Replace(MapPoint* pMP)`

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

#### 1.1.1.4 UpdateNormalAndDepth 更新法线和深度

`void MapPoint::UpdateNormalAndDepth()`

当观测关系改变后会调用此函数，用于更新地图点平均观测方向和尺度无关距离

```
1 遍历所有观测，累加地图点相对于相机的平均方向，除以观测总数
2 计算相对于参考关键帧的距离，按照金字塔比例确定尺度无关距离
```

### 1.1.2 Frame 帧

Frame 是 SLAM 中的帧，输入图像的信息被提取后，创建 Frame 对象存储其信息。为节省内存 Frame 不会直接存放图像，图像会暂存于 ORBextractor 中。Frame 通常是当前帧，如果满足关键帧条件，则会构造关键帧，存于 Map 中。

Frame 中不会对整张图像矫畸变，而是直接在畸变图像上提取特征，再将提取到的特征点坐标去畸变，这样可以保留大范围的图像信息，也可以节省时间。

特征被提取、校畸变后，其序号会被记录到对应的特征网格中。特征网格的边界是原图像边界去畸变后的坐标，因此可能位于图片外，之后的运算里以这个边界为准。

注意，虽然 Frame 有 MapPoint 观测向量成员变量，但 Frame 中并没有提供 MapPoint 观测的相关函数！

注意，Frame 与 KeyFrame 的 BoW 描述向量和特征向量默认是不创建的，需要从外部调用 `ComputeBoW` 计算词袋信息。 

**重要成员变量**

```cpp
/* 位姿 序号 时间 */
long unsigned int mnId;  // 帧 id
cv::Mat mTcw;  // 相机位姿
/* 特征 */
std::vector<cv::KeyPoint> mvKeys;  // 左图畸变关键点
std::vector<cv::KeyPoint> mvKeysRight;  // 右图畸变关键点
std::vector<cv::KeyPoint> mvKeysUn;  // 左图去畸变关键点
cv::Mat mDescriptors, mDescriptorsRight;  // ORB 描述子  左, 右
std::vector<float> mvuRight;  // 特征点右图 u 坐标，双目点第三维坐标  单目取-1
std::vector<float> mvDepth;  // 深度  单目取-1
std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];  // 去畸变左图网格 (x)(y)(vector<关键点序号>)
/* 地图点观测 */
std::vector<MapPoint*> mvpMapPoints;  // 关联到特征点的地图点  注意 Frame 中没有填充此向量信息
/* 词袋 */
DBoW2::BowVector mBowVec;  // 词袋描述向量  map<单词序号, 单词值>
DBoW2::FeatureVector mFeatVec;  // 视觉特征向量  map<节点id, 关键点序号集 vector<int>>
```

Frame 中无互斥锁

#### 1.1.2.1 构造函数

Frame 构造函数分为五类：默认构造函数、拷贝构造函数、双目构造函数、深度构造函数、单目构造函数。

* 默认构造函数

啥也没干

* 拷贝构造函数

复制信息，id 也是拷贝的

* 双目构造函数

> **双目匹配中使用的左、右特征点都是带畸变的，这样匹配出的视差有问题**

输入一对带畸变的双目灰度图像，构造 Frame 对象

首先在左、右畸变图像上分别用左、右提取器进行特征提取，计算描述子。之后会对特征点进行畸变矫正，得到去畸变像素坐标。

双目视差不是逐像素稠密计算的。构造函数对左、右图像特征点进行特征匹配，再进行匹配点对之间的视差。视差的计算比较复杂，结果为亚像素精度，由`ComputeStereoMatches`函数实现。

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

输入一对带畸变的灰度图、深度图，构造 Frame 对象

ORB 中仅有单目点、近双目点、远双目点，因此会将深度图像中的特征点转为双目特征点的形式——通过一个假设基线长度，计算特征点深度对应的视差。

计算深度图像右点坐标时是基于去畸变特征点坐标的，因此不存在双目中的关于视差问题。

```
1 从左提取器获取金字塔尺度信息
2 在畸变图像上提取 ORB 特征，计算描述子  `ExtractORB` C
  1 调用左图 ORB 提取器，计算畸变特征与其描述子，存于向量中
3 计算关键点的去畸变坐标  `UndistortKeyPoints` C
  1 根据畸变参数，计算左图关键点的去畸变坐标
4 依据深度图像计算特征点右点坐标  `ComputeStereoFromRGBD` C
  1 遍历左图特征点，如果特征点深度有效，则基于左特征点去畸变坐标，计算其右点坐标
5 如果是初始化过程，计算去畸变图像边界  `ComputeImageBounds` C
  1 计算去畸变图像边界位置，根据畸变类型不同，边界可能位于图像外
6 分配特征到网格  `AssignFeaturesToGrid` C
  1 按照去畸变坐标计算特征点对应网格标号，记录特征点序号到网格向量中
```

* 单目构造函数

输入带畸变的单目图像，构造 Frame 对象

```
1 从左提取器获取金字塔尺度信息
2 在畸变图像上提取 ORB 特征，计算描述子  `ExtractORB` C
  1 调用左图 ORB 提取器，计算畸变特征与其描述子，存于向量中
3 计算关键点的去畸变坐标  `UndistortKeyPoints` C
  1 根据畸变参数，计算左图关键点的去畸变坐标
4 右点坐标与深度向量赋值 -1
5 如果是初始化过程，计算去畸变图像边界  `ComputeImageBounds` C
  1 计算去畸变图像边界位置，根据畸变类型不同，边界可能位于图像外
6 分配特征到网格  `AssignFeaturesToGrid` C
  1 按照去畸变坐标计算特征点对应网格标号，记录特征点序号到网格向量中
```

#### 1.1.2.2 ComputeStereoMatches 双目匹配计算

`void Frame::ComputeStereoMatches()`

匹配双目特征点，确定右点坐标与深度。

> **此函数中使用的左、右特征点都是带畸变的，这样匹配出的视差有问题**

ORB-SLAM 中的双目匹配不是逐像素的稠密匹配，而是在特征点在左、右图特征点之间进行。双目匹配点对应满足以下条件：匹配点对位于同一行；左点坐标位于右点之右；视差应在合理范围之内；特征尺度应该相同。也许是考虑标定、畸变带来的误差，在此函数中，对于一个左图特征点，会在右图相邻几行中，遍历视差在合理范围内，且特征尺度（即金字塔层数）相差不超过 1 的特征点，寻找最佳的特征点匹配。上述“合理范围”中，视差范围仅与内参有关，相邻行范围与特征尺度有关，尺度相差范围为固定值 1。

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

#### 1.1.2.3 isInFrustum 截椎体检查

`bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)`

检查 MapPoint 是否在 Frame 截椎体内部、是否在尺度无关范围内、是否满足余弦阈值，如果是，填充 MapPoint 内 Tracking 所需的投影坐标信息。

```
1 计算地图点相对于相机位置 tc
2 检查深度是否为负
3 投影到归一化平面，检查是否在去畸变边界外
4 检查距离是否在尺度无关范围外
5 检查观测方向与光轴夹角余弦是否小于阈值
6 若 2~5 皆否，预测尺度，填充地图点信息
```

#### 1.1.2.4 GetFeaturesInArea 获取区域内特征

`vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel) const`

获取以 x, y 为中心，r 为边的 1/2 的方形区域内的关键点序号

### 1.1.3 KeyFrame 关键帧

ORB 中 KeyFrame 是 Map 的构成元素，用于确定局部地图范围、回环检测与重定位。KeyFrame 基于 Frame 构造，继承了 Frame 中大部分成员变量，包括特征点、词袋描述、地图点匹配等，在此基础上，扩充了地图点、共视图、生成树相关的变量与函数。

Frame 中并没有填充地图点观测向量，所以实际观测是在 KeyFrame 中进行的。KeyFrame 中提供了 MP 观测增、删、查、改相关的函数，比较简单，不多叙述。

注意，Frame 与 KeyFrame 的 BoW 描述向量和特征向量默认是不创建的，需要从外部调用 `ComputeBoW` 计算词袋信息。 

ORB 中 没有为共视图 (Covisibility Graph) 和生成树 (Spanning Tree) 设计专门的数据结构，他们都是通过 KeyFrame 中的成员变量实现的。二者都用于记录 KeyFrame 之间的关联关系，但关系的类型有所不同。生成树的边有向，且为共视图边的子集。

共视图记录了 KeyFrame 与其他 KeyFrame 相互关联，用于回环检验、重定位、位姿图优化。一次共视指两个 KeyFrame 中各存在一个 KeyPoint 关联到同一个 MapPoint，全部的共视 KeyFrame 与共视数记录于 mConnectedKeyFrameWeights 映射中。初次创建 KeyFrame 时，需要通过 KeyFrame 与 MapPoint 间的双向查询确定共视关系，即遍历 KF1 观测到的 MP，再遍历每个 MP 关联的 KF，统计即可得到共视数。KeyFrame 间的共视数越大，关联越强，位姿估计越精确 (但共视过大也代表 KeyFrame 冗余)；对应地，如果两个 KeyFrame 间仅有几个共视 MapPoint，则这种弱共视关系无法提供可靠的信息。因此，需要对共视 KeyFrame 进行排序与筛选，建立共视关系时，用一个固定的共视阈值对共视 KF 进行筛选；排序通过 `UpdateBestCovisibles` 函数进行，每一次改变共视关系时都需要调用，结果存放于 mvpOrderedConnectedKeyFrames 和 mvOrderedWeights 向量中。

生成树描述 KeyFrame 的先后关系与回环关系。一个 KF 只能有一个父 KF，可能有多个子 KF 与回环 KF，因此成员变量中有一个指向父 KF 的指针，以及子 KF 和回环 KF 指针的 vector。生成树相关函数是基本的增、删、查、改，不多叙述。

**重要成员变量**

```cpp
/* 继承自 Frame 中的变量，包括关键点、右点坐标、深度、描述子等 */
const std::vector<cv::KeyPoint> mvKeys;  // 左图关键点
const std::vector<cv::KeyPoint> mvKeysUn;  // 左图去畸变关键点
const std::vector<float> mvuRight;  // 特征点右图 u 坐标，双目点第三维坐标 单目取-1
const std::vector<float> mvDepth;  // 深度 单目取-1
const cv::Mat mDescriptors;  // 左图关键点描述子
DBoW2::BowVector mBowVec;  // 视觉单词向量  map<单词序号, 单词值>
DBoW2::FeatureVector mFeatVec;  // 视觉特征向量  map<节点id, 关键点序号集 vector<int>>
std::vector<MapPoint*> mvpMapPoints;  // 关联到关键点的地图点 按关键点索引，未关联到地图点的关键点赋 nullptr  继承自 Frame, 但 Frame 并没有填充此向量信息
/* 位姿 序号 时间 */
long unsigned int mnId;  // 关键帧序号
cv::Mat Tcw;  // 位姿  世界相对相机
cv::Mat Twc;  // 位姿  相机相对世界
cv::Mat Ow;  // 平移 相机相对世界 wc ==mtwc
cv::Mat Cw;  // 双目相机中点
/* 共视图相关 */
std::map<KeyFrame*,int> mConnectedKeyFrameWeights;  // 共视帧与共视数 map<pKeyFrame, weight>
std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;  // 有序共视帧
std::vector<int> mvOrderedWeights;  // 有序权重
/* 生成树与回环边相关 */
KeyFrame* mpParent;  // 父关键帧
std::set<KeyFrame*> mspChildrens;  // 子关键帧
std::set<KeyFrame*> mspLoopEdges;  // 回环边
```

**互斥锁**

```cpp
std::mutex mMutexPose;  // 位姿锁
std::mutex mMutexConnections;  // 共视图、生成树、回环锁
std::mutex mMutexFeatures;  // 地图点锁
```

#### 1.1.3.1 构造函数

拷贝 Frame 中信息，初始化成员变量，没做什么特殊工作

#### 1.1.3.2 UpdateConnections 创建共视关系

`void KeyFrame::UpdateConnections()`

属于共视图相关函数。依据 KF 观测到的 MP 查询 共视 KF，统计、筛选共视数，填充共视向量。如果 KF 是第一次被添加，则将共视数最多的 KF 作为父 KF。

```
1 遍历观测 MPi, 遍历每个 MPi 关联的 KFj, 统计共视数到共视映射 mConnectedKeyFrameWeights
2 遍历共视映射 mKF，保留共视数超过 15 的 KFi 为共视关键帧，将当前 KF 添加到 KFi 共视图
3 如果共视数均未超过 15，保留最大共视 KF，更新对方共视关系
4 对共视数进行排序，同 `UpdateBestCovisibles`
5 如果当前 KF 初次添加，则将共视数最多的 KF 作为父 KF
```

#### 1.1.3.3 SetBadFlag 设置为坏关键帧

`void KeyFrame::SetBadFlag()`

将 KF 设置为坏，并更新其与其他 KF, MP 间的关联关系。

共视图、地图点观测的更新都是简单的删除，生成树更新时，需要按照一定策略为子节点选择合适的父节点。

```
1 删除共视图中当前 KF 的共视信息
2 删除观测 MP 中当前 KF 的观测信息
3 删除生成树中当前帧的观测信息
  1 将父 KF 添加到候选父节点向量
  2 遍历子 KF, 寻找一对最佳父子关系  L
    1 遍历子节点共视 KF 中同时为候选父节点的 KF，记录最佳共视数与序号
    2 按照最佳父子关系，为子节点设置父 KF，同时将此子节点添加到候选父节点向量
    3 如果为所有子节点都分配了父节点，或候选父节点与子节点不存在共视关系，退出此循环
4 将剩余子节点的父节点设置为当前 KF 的父节点
5 从 Map, KeyFrameDatabase 中清除此节点
```

#### 1.1.3.4 GetFeaturesInArea 获取区域内特征

`vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const`

获取以 x, y 为中心，r 为边的 1/2 的方形区域内的关键点序号。同 Frame, 见 1.1.2.4

### 1.1.4 KeyFrameDatabase 关键帧数据库

关键帧数据库用于回环候选关键帧检测、重定位候选关键帧检测，基于 KeyFrame。可以通过 `add`, `erase` 函数添加、删除 KF。

关键帧数据库记录了视觉词典中的词汇在哪些 KF 中出现过，即逆文件。逆文件与文件相反，文件记录了出现的单词，而记录单词在哪些文件中出现过的结构，就是逆文件。通过逆文件，可以实现回环候选关键帧检测，以及重定位候选关键帧检测。

回环、重定位候选检测本质上是相同的，都是在不利用共视关系的前提下，利用词袋信息寻找相似度最高的关键帧。

**重要成员变量**

```cpp
std::vector<list<KeyFrame*> > mvInvertedFile;  // 逆文件向量  包含单词的关键帧列表  [单词序号]list<关键帧>
```

**互斥锁**

```cpp
std::mutex mMutex;  // 关键帧数据库锁
```

#### 1.1.4.1 DetectLoopCandidates 检测回环候选关键帧

`vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)`

回环候选 KF 是与当前 KF 不构成共视的 KF 中，相似度最高的那一部分。回环检测可以提高建图精度、保证全局一致性，是 VSLAM 和 VO 主要的区分点，相对地，错误的回环结果会导致巨大的建图误差。因此在回环检测时，需要以牺牲一部分召回率为代价，保证较高的准确率。在这个函数中，使用了严格的搜索策略来保证准确率，不仅筛选候选 KF 与当前 KF 的相似性得分，还要求候选 KF 的共视 KF 也是候选 KF，即，当前 KF 与连续几个 KF 都有较高相似性，才会提出回环。

候选 KF 的相同单词数、相似性得分等变量存放于 KF public 成员变量中。

函数中有如下四个不同级别的候选 KF 集：

```cpp
list<KeyFrame*> lKFsSharingWords;  // 与当前 KF 有相同词汇的候选 KF
list<pair<float,KeyFrame*> > lScoreAndMatch;  // 相似性得分-关键帧列表  list<pair<相似性得分, 关键帧>>
list<pair<float,KeyFrame*> > lAccScoreAndMatch;  // 累计相似性得分-关键帧列表  list<pair<累计相似性得分, 关键帧>>
vector<KeyFrame*> vpLoopCandidates;  // 回环候选关键帧
```

为了避免多次计算相似性得分，函数中利用了逆文件。首先，依据当前 KF 的描述向量中的单词，从逆文件中寻找与当前 KF 有相同单词、且不构成共视关系的候选 KF，存放于 `lKFsSharingWords`。之后，保留其中相同单词数超过最大值比例阈值的候选关键帧。然后，依据描述向量计算当前 KF 与候选关键帧的相似性得分，按照输入阈值过滤，存于 `lScoreAndMatch`。接着，对每一个候选 KF，取其 10 个最佳共视 KF，对共视 KF 中同样为回环候选 KF 的，累加其相似性得分，并选取这一组共视 KF 中相似性得分最高的作为最终候选 KF，按照最大值比例阈值过滤，存于 `lAccScoreAndMatch`。注意，这里对每个候选 KF 选择其共视组中最佳的 KF，因此 `lAccScoreAndMatch` 中可能出现重复，所以在函数最后筛选了 `lAccScoreAndMatch` 中不重复的 KF，存放于 `vpLoopCandidates`，返回结果。

```
1 遍历当前 KF 描述向量，从逆文件查询与当前 KF 有相同视觉词汇且不构成共视关系的 KF，作为候选 KF, 存入 lKFsSharingWords
2 计算相同单词数阈值，阈值为最多相同单词数的 0.8 倍
3 按照相同单词数阈值对候选关键帧进行筛选，对满足要求的候选 KF 计算相似性得分，保留得分高于阈值的关键帧，存入 lScoreAndMatch
4 依据共视关系计算累计相似性分数，考虑候选 KF 共视关键帧中同样为候选 KF 的那部分，对他们与被搜索帧的相似性得分求和，保留组共视帧中得分最大的 KF，存入 lAccScoreAndMatch
5 保留所有累计得分高于最高得分 0.75 倍的候选关键帧
6 筛选候选关键帧，防止重复出现
```

#### 1.1.4.2 DetectRelocalizationCandidates 检测重定位候选关键帧

`vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)`

类似于 `DetectLoopCandidates`。

```
1 遍历当前 F 描述向量，从逆文件查询与当前 KF 有相同视觉词汇的 KF，作为候选 KF, 存入 lKFsSharingWords
2 计算相同单词数阈值，阈值为最多相同单词数的 0.8 倍
3 按照相同单词数阈值对候选关键帧进行筛选，对满足要求的候选 KF 计算相似性得分，存入 lScoreAndMatch
4 依据共视关系计算累计相似性分数，考虑候选 KF 共视关键帧中同样为候选 KF 的那部分，对他们与被搜索帧的相似性得分求和，保留组共视帧中得分最大的 KF，存入 lAccScoreAndMatch
5 保留所有累计得分高于最高得分 0.75 倍的候选关键帧
6 筛选候选关键帧，防止重复出现
```

### 1.1.5 Map 地图

地图，是 MapPoint, KeyFrame 的集合

地图是关键帧和地图点的集合。成员函数为基本的增删查改，不多叙述。

> Map 在清除个别 MP, KF 时仅仅将指针从 set 中清除，但并没有从堆区中删除 MP 与 KF 本身，会造成内存泄漏。

**重要成员变量**

```cpp
std::set<MapPoint*> mspMapPoints;  // 地图点集合
std::set<KeyFrame*> mspKeyFrames;  // 关键帧集合
std::vector<MapPoint*> mvpReferenceMapPoints;  // 参考地图点集
std::vector<KeyFrame*> mvpKeyFrameOrigins;  // 关键帧原点
```

**互斥锁**

```cpp
std::mutex mMutexMapUpdate;  // 地图更新锁
std::mutex mMutexPointCreation;  // 地图点创建锁
std::mutex mMutexMap;  // 地图锁
```

## 1.2 功能类

### 1.2.1 ORBextractor 特征提取器

ORB 特征提取器。ORB 特征是加入了旋转不变性的 FAST 特征，通过计算灰度质心得到特征主方向，保证旋转不变性。使用 BRIEF 描述子，描述子计算时抵消了特征主方向。为了保证尺度不变，考虑了从金字塔不同层中提取的特征。

这个提取器是作者从 OpenCV 源码修改得到的，添加了图像金字塔、图像网格划分、四叉树特征筛选等。与 OpenCV 的 ORB 提取相比，特征分布更均匀，能提取到不同尺度的特征，运行速度也更快。

> 源码中特征筛选方法命名为“八叉树”，但每个节点实际只划分四个子节点，所以我称之为四叉树。

ORBextractor 不依赖于 ORB-SLAM 中的其他模块，我们可以把 ORBextractor 的头文件和源文件单独拿出来编译测试。

ORBextractor 特征提取主要通过 `ComputeKeyPointsOctTree` 实现，但也有另一版本的提取函数 `ComputeKeyPointsOld`，后者划分图像网格，在每个网格中提取特征点，在网格中的按特征响应进行排序，筛选最优特征。如果特征总数不够，则从有多余特征的网格中再筛选一些出来。

ORBextractor 在 Tracking 中创建，Tracking 接收到图像后将 ORBextractor 地址传给 Frame 构造函数，在构造函数中完成特征提取。ORBextractor 提取特征后，会暂时保留着上一次帧的金字塔、尺度等信息，Frame 构造函数之后的操作中会从 ORBextractor 获取这些数据使用。

cv::KeyPoint 的成员变量：0 层位置、响应强度、方向（度）、层数

```cpp
1;
```

KeyPoint 尺度级别（层数）越高，特征点深度越小

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

**需重写**

```
1 构建金字塔，金字塔图像预留了 BRIEF 计算边界 `ComputePyramid` C
2 四叉树法提取 ORB 特征  `ComputeKeyPointsOctTree` C
  1 遍历金字塔每一层，逐层提取特征  L
    1 将图像划分大小为 30*30 的网格，遍历每个网格，在网格中提取特征点，提取使用 OpenCV `FAST`，开启 NMS。如果没有提取到特征，则使用小阈值再次提取。每个网格中提取的特征，无论来自大、小阈值，都会先将坐标从网格变换为相对于特征提取边界，再无差别地放入 vToDistributeKeys 向量中。
    2 四叉树法分配特征点  `DistributeOctTree` C
      1 划分四叉树，直到子节点数量大于等于目标特征数，或每个子节点都只包含一个特征
      2 保留每个子节点中响应最强的特征点，存入结果向量，返回
    3 将特征坐标从相对于特征提取边界变换为相对于金字塔图像，添加层数、尺度信息
  2 计算特征主方向。遍历每个特征点，计算灰度质心角  `computeOrientation` C
3 遍历每一层，逐层计算描述子  L
  1 高斯平滑  `GaussianBlur` C
  2 调用 `computeDescriptors` 计算描述子。对每个特征点调用 `computeOrbDescriptor`，取模板点旋转质心角后在图像中的坐标进行比较，得到旋转无关的描述子。
  3 将特征坐标恢复到金字塔 0 层，放入结果向量
```

提取后的特征点包括了在金字塔第 0 层中的坐标、方向、尺度、出自金字塔层数、响应强度。

### 1.2.2 ORBmatcher 特征匹配器

特征匹配器类是不同 ORB 匹配方法的合集，目的是为了寻找不同 KF (F) 的 KP 之间、KF (F) 的 KP 与 MP 之间的匹配，作为下一步调用优化器、求解器的输入。也可以进行冗余 MP 的融合。

KP 或 MP 间的相似程度通过其描述子的汉明距离度量。最简单的匹配方法是对两个点集中所有可能的点对组合计算汉明距离，取其中最相似的点对组合作为匹配结果，即暴力匹配 (Brute-Force)。但暴力匹配相当耗费时间 (O(n^2))，如果能根据已知信息限制匹配范围，匹配速度会大幅度提升，同时也会减小误匹配出现的概率。

ORB 中用于加速匹配的“已知信息”分为两种：几何投影信息，词袋类别信息。

1. **几何投影信息**指已知先验的位姿信息，利用其计算点集 2 (可能来自图像 KP，也可能是 MP 集) 中的点 $\bold{P}_{2j}$ 投影到点集 1 图像平面上的像素坐标 $\bold{p}_{1j}$，并在以 $\bold{p}_{1j}$ 为中心的窗口中寻找与 $\bold{P}_{2j}$ 匹配的特征点。窗口大小是按照特征尺度、视角余弦、以及一个用户输入的阈值确定的。对于有效的双目点，还要考虑投影右点坐标与匹配点之间的差值不超过窗口半径。利用几何投影信息可以加快匹配速度、减少误匹配。

2. **词袋类别信息**利用视觉特征向量，限制两帧间特征匹配仅在视觉词典 k 叉树同一下节点的特征点之间进行。利用词袋类别信息只能加快匹配速度，由于匹配仍是在全图范围内进行，所以不能防止误匹配的发生。

误匹配会严重影响求解器、优化器的计算结果，虽然他们中都有严密的外点剔除策略，但这些策略都基于外点只占少数的假设，所以，在匹配器中要尽可能地防止误匹配的出现。ORB 中的匹配筛选方法有以下几种：

1. **姿态一致性检验**，基于旋转直方图。只用于包含两帧的匹配，因为地图点没有方向。统计匹配 KP 点对间的姿态差到旋转直方图，保留旋转直方图中最大的三列（如果第二、三列远小于第一列，则抛弃之）中对应的匹配点对。

> 代码里的直方图分配有些奇怪，直方图保留了 30 列，但应该是没有用到所有的列。这个一致性应该只是近似成立，对于不同视角下的特征点，相机滚转角改变时，其灰度主方向的改变是不同的。

2. **尺度一致性检验**，基于最佳、次佳匹配阈值（部分函数中使用）。投影匹配中会记录每个 MP 的最佳和次佳 KP 匹配，如果这两个 KP 来自金字塔同一层，且最佳匹配距离远小于次佳匹配，则为最佳匹配 KP 添加这一 MP 观测。

3. **双目右点坐标检验**，投影匹配在比较 MP 与窗口内 KP 时，如果 KP 双目信息有效，则会限制 MP 投影右点坐标与 KP 右点坐标的差，如果大于窗口半径，则认为不是匹配对。

4. **匹配 KP 层数要求**，1。

ORBmatcher 中包含的函数大概可以分为以下几类：

1. 投影匹配函数 `SearchByProjection`，包含四个重载

2. Sim3 互投影匹配 `SearchBySim3`

3. 词袋匹配函数 `SearchByBoW`，包含两个重载

4. 初始化匹配 `SearchForInitialization`

5. 三角化匹配 `SearchForTriangulation`

6. 地图点融合 `Fuse`，包含两个重载

7. 工具函数: 描述子距离计算 `DescriptorDistance`，直方图最大值计算 `ComputeThreeMaxima`，极线距离检查 `CheckDistEpipolarLine`，搜索半径确定 `RadiusByViewingCos`

#### 1.2.2.1 DescriptorDistance 计算描述子汉明距离

`int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)`

计算两个 256 位描述子的汉明距离，每个描述子为 1*256 bit 的 cv::Mat

使用了[并行位操作](http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel)的快速算法

#### 1.2.2.2 SearchByProjection 投影匹配函数

`int SearchByProjection(...)`

按照几何关系，寻找 MP 与其在 F(KF) 图像上投影点附近窗口内特征点间的匹配关系。MP 可以来自于另一 F(KF) 的观测 MP。

这个函数有四种重载，在不同线程中实现不同的功能。其匹配原理是相似的，都是基于已知（或称为先验）的位姿信息，限制图像中 KP 的搜索范围。

没有额外的匹配筛选方法。

##### 1.2.2.2.1 F to Local MP  局部地图投影匹配

`int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th)`

F 匹配到 MPs，Tracking 中用于追踪 LocalMap

这个函数中没有实际的投影计算过程，调用前已经提前计算过了 MP 在 F 内的投影位置，存放在 MP 的成员变量中。应该是在 Tracking 中通过 F(or KF).isInFrustum 计算的。

使用最佳、次佳匹配限制。

```
1 遍历 MP 集  L
  1 依据特征尺度、视角余弦确定窗口大小，获取 F 上 MPi 投影位置窗口范围内的 KP
  2 遍历窗口内 KP，计算与 MPi 的汉明距离，保留最佳和次佳匹配
  3 如果最佳匹配距离小于阈值，且最佳 KP 和次佳 KP 来自金字塔同一层，则为 KP 添加对 MPi 的观测
```

##### 1.2.2.2.2 F to pre-F  相邻帧间投影匹配

`int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)`

Fc 匹配到上一帧 Fl 观测地图点，Tracking 中用于追踪前一帧

使用姿态一致性检验。

```
1 由先验位姿计算 Fc 与 Fl 的相对位姿 Tcl，判断相机前移或后移
2 遍历 Fl 观测 MP 集  L
  1 按照 Tcl 将 MPi 投影到 Fc 图像平面上
  2 依据特征尺度确定窗口大小，获取 Fc 上 MPi 投影位置窗口范围内的 KP
  3 遍历窗口内 KP，计算与 MPi 的汉明距离，保留最佳匹配，为满足阈值的最佳 KP 添加 MP 观测信息
  4 计算 MPi 在 Fl 内对应 KP 与 Fc KP 特征方向差，记录到旋转直方图中
3 旋转一致性检验，保留直方图点数最多三个列中的匹配对
```

##### 1.2.2.2.3 F to KF  重定位投影匹配

`int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint*> &sAlreadyFound, const float th , const int ORBdist)`

Fc 匹配到 KF 观测地图点，Tracking 中用于重定位

将 KF 的观测地图点投影到 Fc 图像平面上，这里要求 Fc 有先验位姿

使用姿态一致性检验

```
1 获取 Fc 位姿 Tcw
2 遍历 KF 观测 MP 集  L
  1 按照 Tcw 将 MPi 投影到 Fc 图像平面上
  2 预测特征尺度，按照预测尺度确定窗口半径
  3 获取 Fc 上 MPi 投影位置窗口范围内的 KP
  4 遍历窗口内 KP，计算与 MPi 的汉明距离，保留最佳匹配，为满足阈值的最佳 KP 添加 MP 观测信息
  5 计算 MPi 在 KF 内对应 KP 与 Fc KP 特征方向差，记录到旋转直方图中
3 旋转一致性检验，保留直方图点数最多三个列中的匹配对
```

##### 1.2.2.2.4 KF to MP with Scw  回环检测投影匹配

`int ORBmatcher::SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)`

使用给定的相似变换 Scw 将 MP 投影到 KF 图像平面上，寻找匹配 **存疑**

对匹配对的要求：深度不为负数；投影点在图像去畸变边界内；图像内 KP 只能与预测级别相等或变远一个级别

```
1 分解 Scw 获取姿态、平移等
2 遍历 MP 集  L
  1 按照 Scw 将 MPi 投影到 KF 图像平面上
  2 预测特征尺度，按照预测尺度确定窗口半径
  3 获取 KF 上 MPi 投影位置窗口范围内的 KP
  4 遍历窗口内 KP，计算与 MPi 的汉明距离，保留最佳匹配到集合中
```

#### 1.2.2.3 SearchByBow 词袋匹配函数

`int ORBmatcher::SearchByBoW(...)`

利用视觉特征向量，寻找 F (KF) KP 与另一个 KF 观测到 MP 间的匹配。限制搜索范围为与属于同一视觉单词节点的特征点和地图点之间。

视觉特征向量 mFeatVec 类型为 map<节点id, 关键点序号集 vector<int>>

注意，F 与 KF 创建时不会自动计算词袋信息，如果需要使用词袋匹配函数，需要先对 F 或 KF 调用 ComputeBoW。

##### 1.2.2.3.1 F to KF 词袋匹配

`int ORBmatcher::SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)`

通过视觉字典节点限制，搜索 F 到 KF 观测到 MP 的匹配。结果返回为 F 的 KP 匹配到 KF 观测 MP，按 F KP 索引。

使用姿态一致性检验

使用最佳、次佳匹配限制

用在 `Tracking::TrackReferenceKeyFrame()` 中

```
1 遍历视觉字典所有视觉词汇节点  L
  1 遍历 F 中属于此节点的 KPi  L
    1 在 KF 所有属于此节点的 KP 对应的 MP 中计与 KPi 的汉明距离，保留最佳和次佳匹配
  2 如果最佳匹配远优于次佳匹配，记录之
2 旋转一致性检验，保留直方图点数最多三个列中的匹配对
```

##### 1.2.2.3.2 KF to KF 词袋匹配

`int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)`

通过视觉字典节点限制，搜索 KF1 到 KF2 观测到 MP 的匹配。结果返回为 KF1 的 KP 匹配到 KF2 观测 MP，按 KF1 KP 索引。

使用姿态一致性检验

使用最佳、次佳匹配限制

```
1 遍历视觉字典所有视觉词汇节点  L
  1 遍历 KF1 中属于此节点的 KPi  L
    1 在 KF2 所有属于此节点的 KP 对应的 MP 中计与 KPi 的汉明距离，保留最佳和次佳匹配
  2 如果最佳匹配远优于次佳匹配，记录之
2 旋转一致性检验，保留直方图点数最多三个列中的匹配对
```

#### 1.2.2.4 Fuse 地图点融合

1

#### 1.2.2.5 SearchForInitialization 初始化匹配

`int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)`

按照 F1 KP 在 F2 中的预先匹配位置搜索在 F2 中的匹配 KP，结果返回 F1 对 F2 KP 的匹配，按照 F1 KP 顺序索引。仅在 `Tracking::MonocularInitialization` 中使用。对于 F1 中位于 0 层的 KP，根据 F1 KP 在 F2 中预匹配位置确定窗口，寻找窗口中最佳匹配点。实际调用时，F1 为参考帧，F2 为与参考帧相邻的下一帧，预匹配位置为 F1 KP 在 F1 中的位置，当相机运动速度不大时，相邻帧间特征点距离不会太远，可以在窗口中找到正确的匹配。

姿态一致性检验。

使用最佳、次佳匹配限制。

```
1 遍历 F1 中位于 0 层的 KP  L
  1 获取 KPi 在预匹配位置附近窗口内的 KP 作为候选
  2 遍历窗口内 KP，计算与 KPi 的汉明距离，保留最佳匹配与次佳匹配
  3 如果最佳匹配距离小于阈值，记录这一对匹配
  4 计算 KPi 与对应 F2 中 KP 的特征方向差，记录到旋转直方图中
2 旋转一致性检验，保留直方图点数最多三个列中的匹配对
```

### 1.2.3 Optimizer 优化器

ORB-SLAM 中非线性最小二乘优化器使用 g2o。g2o 问题由边 (Edge) 和节点 (Vertex) 组成，ORB 中使用了 g2o 为 VSLAM 预先编写好的边类和节点类，存放于三方库文件夹中。

Optimizer 是一系列优化方法的集合，使用时不会创建对象，而是直接调用成员函数。

> 有空可以试着改成 Ceres，需要将矩阵改为 Eigen，使用 double 类型

优化问题通常需要一个好的初值，以保证收敛到全局极小值、提高求解速度。ORB 中根据解析方法，例如 PnPSolver, Sim3Solver，求得初值，然后交给优化器处理。

关键帧对地图点的图优化构建过程中，每一个观测的信息矩阵与对应关键点所在的金字塔层数有关，这个层数并不能直接与地图点深度等价，相当于只考虑了特征提取的误差，而没有考虑深度相机本身的测量噪声。这里可能是一个改进点。

无论是解析方法、还是迭代方法，基于多对匹配关系的计算问题都对外点很敏感。优化方法中，可以在构建残差项时使用核函数，在一定程度上减小外点的影响，但核函数只能将外点残差对总目标函数的贡献从二次方变为接近线性，并不能消除其影响。因此需要一些外点剔除策略，例如基于残差阈值的外点剔除、RANSAC 方法。ORB 中，在解析求解器中使用了 RANSAC，在迭代优化器中使用了基于残差阈值的外点剔除：先进行初步优化，对每一个残差项分别计算误差，剔除外点，再次优化。

ORB 中因子图构建时，对 MP Vertex 开启边缘化。这里的边缘化和 VINS 中的边缘化不同，VINS 中的边缘化指固定过去的 MP, KF 将其转换为先验信息，会导致海赛矩阵不再稀疏。而 ORB 中的边缘化指在求解增量时，利用类海赛矩阵的稀疏性，先求解相机位姿增量，再求解地图点增量，以加快求解速度。

g2o::Edge 类中对 Vertex 的记录是以基类指 (`g2o::HyperGraph::Vertex`) 针保存的，在计算残差时需要从对应 Vertex 中获取参数当前估计值，此时会发生下行转换。理论上来讲，下行转换时应该使用 `dynamic_cast<DerivedT*>(BaseT*)` 运行时类型检查，以保证下行转换的安全，但 g2o 官方给出的代码中仍使用 `static_cast`。推测可能是为了节省时间，并且，Edge 中 Vertex 指针是严格按照序号存放的，一般不会有歧义。

向 g2o 图模型添加 Vertex 和 Edge，需要为他们指定唯一的序号，之后可以通过这个序号从图模型获取 Vertex 和 Edge 的指针，这个序号可以不连续。添加顺序通常为先添加 Vertex，再添加 Edge。BA 问题中的 Vertex 有 KeyFrame 和 MapPoint 两种，且数量很多，ORB 中的做法是 KF Vertex 和 KF 的序号完全对应，MP Vertex 的序号为最大 KF 序号 + MP 序号，这样即可以避免顶点间的序号冲突，也可以在添加 Edge 时，按照序号轻松获取对应 KF, MP Vertex 的指针。

BA 问题有额外的 6 个或 7 个自由度，第七个自由度为单目尺度，剩余六个是地图的整体偏移和旋转。为了避免优化过程引来无意义的偏移，优化过程中会将第 0 个 KF (如果这一个优化问题中包括了第 0 帧) 设置为固定。

按照是否存在右点坐标，KF 对 MP 的观测可以分为单目和双目两类，在构建 BA 问题时，会依据深度是否有效添加单目边和双目边两类边。

g2o 有两种方式将边设置为外点，即不提供残差，不参与优化。调用 removeEdge。设置边的 level 为 1。

**残差公式总结，单目重投影误差，双目重投影误差，位姿图误差**

1 双目重投影误差长啥样？

* 使用到的 g2o 类

```cpp
// Vertex
g2o::VertexSE3Expmap  // g2o SE3 类  BA
g2o::VertexSBAPointXYZ  // g2o v3d 类  BA
g2o::VertexSim3Expmap  // g2o Sim3 类  PGO  这个类可以开启固定尺度选项，等价于 SE3
// Edge
g2o::EdgeSE3ProjectXYZ  // 空间点到成像平面  Mono-BA
g2o::EdgeStereoSE3ProjectXYZ  // 空间点到双目成像平面  Stereo-BA
g2o::EdgeSE3ProjectXYZOnlyPose  // 空间点到成像平面  Motion-only-BA
g2o::EdgeStereoSE3ProjectXYZOnlyPose  // 空间点到双目成像平面  Motion-only-BA
g2o::EdgeSim3  // 本质图 Sim(3)  PGO
g2o::EdgeSim3ProjectXYZ  // Sim3 投影  Sim3-Opt
g2o::EdgeInverseSim3ProjectXYZ  // Sim3 逆投影  Sim3-Opt
```

> 自己测试一下这些类的速度，和 Ceres 做比较
> 尽量还是转 Ceres 吧，毕竟 Google 撑腰

#### 1.2.3.1 BundleAdjustment 光束法平差

`void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)`

BA 优化，同时优化给定的 KeyFrame 位姿和 MapPoint 位置。使用开启边缘化的块求解器 `BlockSolver_6_3`，其中 KF 位姿不能启用边缘化，而 MP 必须开启边缘化，否则编译报错。

开启强制停止标志位

没有外点剔除

g2o 配置如下：

```cpp
g2o::SparseOptimizer optimizer;  // 图模型
g2o::BlockSolver_6_3::LinearSolverType * linearSolver;  // 线性求解器  6*3
linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);  // 块求解器
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);  // 图优化求解器
optimizer.setAlgorithm(solver);
// Vertex
g2o::VertexSE3Expmap;  // Camera Pose
g2o::VertexSBAPointXYZ;  // Point Pos. Marginalized on
// Edge
g2o::EdgeSE3ProjectXYZ;  // Mono-obs.
g2o::EdgeStereoSE3ProjectXYZ  // Stereo-obs.
```

```
1 配置求解器
2 遍历优化 KF 集，添加 KF 顶点
3 遍历优化 MP 集，添加 MP 顶点，开启边缘化  L
  1 KF 对 MP 的观测为双目  ?
    T 添加双目边
    F 添加单目边
4 优化指定轮数
5 从优化结果恢复 KF, MP 数据
```

#### 1.2.3.2 GlobalBundleAdjustemnt 全局光束法平差

`void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)`

从 Map 中获取全部 MP, KP，之后调用 `BundleAdjustment` 求解。

#### 1.2.3.3 LocalBundleAdjustment 局部光束法平差

`void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)`

局部 BA 接收参数列表中的一个 KF，获取 KF 的共视 KF 作为局部 KF，获取局部 KF 观测到的 MP 作为局部 MP，观测到局部 MP，但不为局部 KF 的 KF 作为固定 KF。固定 KF 的顶点设置为固定，这本质上和 VINS 中的边缘化类似，但没有考虑先验的概率分布。上述局部 KP 与局部 MP 添加时需要判断不为坏帧、坏点，并且注意不能重复添加 KF。

使用了基于残差阈值的外点剔除 (这里的外点指错误的观测关系，即因子图的边)。优化分为两步进行，初次优化 5 轮后，剔除外点，再次优化 10 轮。第二次优化结束后会再次检测外点，并清除外点对应 KF 和 MP 间的观测关系。

开启强制停止标志位

g2o 配置如下：

```cpp
g2o::SparseOptimizer optimizer;  // 图模型
g2o::BlockSolver_6_3::LinearSolverType * linearSolver;  // 线性求解器
linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);  // 块求解器
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);  // 图优化求解器
optimizer.setAlgorithm(solver);
// Vertex
g2o::VertexSE3Expmap;  // Camera Pose
g2o::VertexSBAPointXYZ;  // Point Pos. Marginalized on
// Edge
g2o::EdgeSE3ProjectXYZ;  // Mono-obs.
g2o::EdgeStereoSE3ProjectXYZ  // Stereo-obs.
```

```
1  从输入 KF 共视图获取局部 KF
2  从局部 KF 观测关系获取局部 MP
3  遍历局部 MP 观测关系，确定固定 KF
4  求解器配置
5  遍历局部 KF 集添加局部 KF 顶点
6  遍历固定 KF 集添加固定 KF 顶点，开启固定
7  遍历局部 MP 集，添加 MP 顶点，开启边缘化  L
   1 KF 对 MP 的观测为双目  ?
     T 添加双目边
     F 添加单目边
8  初步优化 5 轮
9  计算残差，确定外点，设置外点边级别为 1
10 再次优化 10 轮
11 计算残差，确定外点，清除外点边对应的 KF, MP 间的观测关系
12 从优化结果恢复 KF, MP 结果
```

#### 1.2.3.4 PoseOptimization 单帧位姿优化

`int Optimizer::PoseOptimization(Frame *pFrame)`

即 Motion-only BA。这个图优化问题中只有一个顶点，即待优化关键帧。因子图中的边为对地图点的观测，但这些边都是一元边，MP 坐标作为边的成员变量存在。对于单目观测和双目观测创建不同的边。

单帧位姿优化为 Tracking 中接受到图像时的初步处理，因此观测中可能存在较多外点。在这个函数中，优化分为四步进行，每次优化 10 轮后都会剔除外点，再进行下一次优化。最后一次优化时取消了所有残差项的核函数。

g2o 配置：

```cpp
g2o::SparseOptimizer optimizer;  // 图模型
g2o::BlockSolver_6_3::LinearSolverType * linearSolver;  // 线性求解器
linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);  // 块求解器
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
optimizer.setAlgorithm(solver);
// Vertex
g2o::VertexSE3Expmap;
// Edge
g2o::EdgeSE3ProjectXYZOnlyPose;
g2o::EdgeStereoSE3ProjectXYZOnlyPose;
```

```
1 配置求解器
2 添加 KF 顶点
3 依据 KF 地图点观测添加单目或双目观测边  L
  1 KF 对 MP 的观测为双目  ?
    T 添加双目边
    F 添加单目边
4 进行四步优化  L
  1 优化 10 轮
  2 计算残差，确定外点，设置外点边级别为 1
5 恢复 KF 位姿
```

#### 1.2.3.5 OptimizeEssentialGraph 本质图优化

`void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, const LoopClosing::KeyFrameAndPose &NonCorrectedSim3, const LoopClosing::KeyFrameAndPose &CorrectedSim3, const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)`

ORB 中的本质图优化类似于位姿图优化，但只考虑共视数多于 100 的边。本质图优化只会在检测到回环时调用，图中的边包括本次检测出的回环边，以及生成树和本质图中的边，本质图中的边又包括历史检测出的回环边，以及共视数大于阈值的边。

优化结束后，还需要同步更新地图点位置。

**存疑**  这里给出的修正 Sim3 和未修正 Sim3 的作用不是很理解。在向图中添加 KF 顶点时候，会查询 `CorrectedSim3`，如果有 KF 信息则使用之。在向图添加观测边时，会查询 `NonCorrectedSim3`，如果有则使用之计算帧间变换。

求解器配置：

```cpp
g2o::SparseOptimizer optimizer;  // 图模型
g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();  // 线性求解器
g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);  // 块求解器
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);  // 图优化求解器
// Vertex
g2o::VertexSim3Expmap;
// Edge
g2o::EdgeSim3;
```

```
1 配置求解器
2 添加 KF 节点
3 添加当前回环边
4 添加本质图边，包括生成树、历史回环边、共视图边
5 优化 20 轮
6 依据结果恢复 KF 位姿
7 修正地图点
```

#### 1.2.3.6 OptimizeSim3 相似变换群优化

`int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)`

给出 KF1 MP 对 KF2 MP 的对应关系，求解帧间变换。这里给出两 KF 对 MP 的观测关系，即已知 MP 在 KF 相机坐标系下的位置，所以是 ICP 问题。但这个函数中对于残差的构建比较特别，一般的 ICP 问题残差为三维坐标差，设 KF1 观测到 MP1 坐标 $\bold{P_1}$，KF2 观测到 MP2 坐标 $\bold{P_2}$，帧间变换为 Sim3，则残差为：

$$
\bold{e}=\bold{P_1}-\bold{S_{12}}\bold{P_2}
$$

但 ORB 中的残差为互投影误差，即将 MP2 投影到 KF1，将 MP1 投影到 KF2，并与各自像素坐标做差。

$$
\begin{split}
  \bold{e_1}=\bold{p_1}-\frac{1}{z}\bold{K}\bold{S_{12}}\bold{P_2}
\end{split}
\\
\begin{split}
  \bold{e_2}=\bold{p_2}-\frac{1}{z}\bold{K}\bold{S_{21}}\bold{P_1}
\end{split}
$$

MP 顶点开启固定，只优化帧间变换。

使用外点剔除，分别计算两个重投影误差，只要其中有一个超过阈值，则认为匹配点对是外点，从图中删除，并在最后清除这个地图点观测。

g2o 配置：

```cpp
g2o::SparseOptimizer optimizer;  // 图模型
g2o::BlockSolverX::LinearSolverType * linearSolver;  // 线性求解器
linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);  // 块求解器
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);  // 图优化求解器
optimizer.setAlgorithm(solver);
// Vertex
g2o::VertexSim3Expmap;
// Edge
g2o::EdgeSim3ProjectXYZ;
g2o::EdgeInverseSim3ProjectXYZ;
```

```
1 求解器配置
2 添加帧间变换 Sim3 顶点
3 添加 MP 顶点，按照观测关系添加边
4 初步优化 5 轮
5 计算重投影误差，剔除内点
6 再次优化 10 轮
7 计算重投影误差，清除 KF 对 MP 的观测
8 从优化结果获取帧间变换结果
```

### 1.2.4 PnPsolver PnP 求解器

PnP (Perspective-n-Points) 问题指已知 n 个地图点的空间位置，以及这些点与图像内二维特征点的对应关系 (这些对应关系中可能存在外点)，求相机的位姿。PnP 可以通过 DLT (Direct Linear Transformation) 等求解析解，也可以构造最小二乘问题求迭代解，获得更高精度。

ORB 中的 PnP 求解器基于 RANSAC + EPnP 算法。RANSAC (Random Sample Consensus) 用于剔除原始关联信息中的外点，EPnP 用于快速求解 PnP。RANSAC 需要多次调用 EPnP 进行初步估计与改善。

RANSAC 原算法是先从全部数据中采样，获得一个最小集合，拟合一个模型，然后依据这个模型增加内点，再计算新模型，以此类推，不断扩大内点集合；多次采样，重复上述计算，选取内点数最多的模型。但 ORB 中 RANSAC 每一次迭代只进行一次采样，之后统计内点，如果内点数多于阈值，则进行改善 (Refine 函数)，从全部内点计算模型作为候选。这样速度更快，且由于解析求解器只是为优化提供初值，过高的精度意义不大。

EPnP 是一种非优化方法，复杂度为 O(n)，相比于其他非迭代法（复杂度为 O(n^5), O(n^8) 左右）速度更快、结果更精确、受外点影响更小。[EPnP 论文及源码](https://github.com/cvlab-epfl/EPnP)。

EPnP 中将 n 个空间点用 4 个虚拟控制点的线性组合 Alpha 表示，将对相机位姿的估计转化为对控制点在相机系下的坐标估计，共 12 维。控制点选取原则如下，空间点重心为控制点 0，此点与其余三点构成空间点云主方向，即对点云坐标矩阵 SVD 分解 (主成分分析) 右正交矩阵 U 的列向量乘对应奇异值，注意结果不是单位向量，这样选择基底的结果更鲁棒。用 n 个空间点信息构建矩阵 M，控制点相机系坐标为 M 0 奇异值特征向量 (核向量) 的线性组合，线性组合的系数通过非线性最小二乘法求解。

求得控制点相机坐标后，可以根据 Alpha 求出 n 个空间点相对于相机的坐标，这时相机位姿估计问题转化为 ICP (Iterative Closest Point)，可以基于 SVD 分解求解析结果。

平面场景只需要使用三个控制点，但在代码中没有体现。

EPnP 结果可以为迭代法提供良好初值，保证迭代法的快速收敛。迭代基于 LHM 方法，其残差项不是二维重投影误差，而是在三维空间中的误差，这样可以提高结果精度。EPnP 与 LHM 的组合同时兼顾了速度和精度。这在代码中也没有体现。

> ORB 中 PnP 求解器是在 EPnP 的源码基础上修改的，代码风格相差较大。EPnP 源码只依赖于 OpenCV3 进行矩阵分解，其内部向量、矩阵等通过 double 数组实现。纯数组的矩阵计算看的令人窒息...
> 更令人窒息的是，EPnP 的代码在核向量系数确定这部分和论文中叙述的不一致，实际以代码为准

空间点坐标是控制点坐标的线性组合，控制点坐标是 M 矩阵核向量的线性组合

EPnP 最少需要 4 对空间点-特征点匹配，RANSAC 随机选取初始点对时也是基于此

#### 1.2.4.1 PnPsolver 中 EPnP 算法概述

1

```
1 
```

#### 1.2.4.2 PnPsolver 中 RANSAC 算法概述

1

```
1 
```

### 1.2.5 Sim3Solver Sim(3) 求解器

用于检测到回环后初步位姿估计，给出 KF1, KF2, 以及 KF1 观测 MP 和 KF2 观测 MP 之间的对应关系，求解 KF2 相对与 KF1 位姿，结果为 Sim3 或 SE3。

类似于 PnPSolver 的求解策略，RANSAC + Sim3 求解。Sim3 求解是一个 Sim3 下的 ICP 问题。见论文 [Closed-form solution of absolute orientataion using unit quaternions](https://web.stanford.edu/class/cs273/refs/Absolute-OPT.pdf)

提出了一种对称尺度求解方法，可以不依赖姿态、位置，直接求解尺度 s12

$$
s_{12}=\sqrt{\frac{\sum_{i=1}^{n}{\Vert r'_{1,i} \Vert}}{\sum_{i=1}^{n}{\Vert r'_{2,i} \Vert}}}
$$

其中 $r'_{1,i}$, $r'_{2,i}$ 为空间点在 KF1, KF2 下的去中心化坐标

但是，**代码中并没有用这个公式**，尺度计算仍然是基于姿态的。

姿态、位置的求解类似于 SE3 上的 ICP 问题

最优姿态四元数对应一个 4*4 矩阵最大特征值对应的特征向量，论文中有公式。实际上，姿态部分用 SVD 分解求旋转矩阵会更快一点，ORB 源码是完全按照论文中的方法求解的

位置通过姿态直接计算得出，比较简单

> 论文中公式有错误，对称尺度部分目标函数少了两个根号
> 论文是 1987 年的，比较老
> 其实没有必要用这个论文中的公式，用了反而效率更低，Sim3Solver 这一部分很奇怪

#### 1.2.5.1 Sim(3) 求解

1

```
1 
```

#### 1.2.5.2 RANSAC

1

```
1 
```

### 1.2.6 Initializer 初始化器

开双线程，同步计算基础矩阵和单应矩阵，选择重投影误差较小，且满足误差阈值的，作为初始化结果。计算 F 和 H 时使用 RANSAC 筛选内点。计算结束后会三角化空间点。

计算位姿的同时会对特征点进行三角化。

用到了 RANSAC

初始化时，现进行 RANSAC 采样，200 对样本  `SampleForRansec` 只采样不计算

F 与 H 都是在归一化坐标上计算的，保证数值稳定性。KP 归一化坐标，即对 KP 去均值后除以标准差，得到标准正态分布的数据。标准差计算耗时比较大，ORB 中用误差绝对值均值代替。归一化操作相当于对原始坐标进行一次线性变换 T，基于 归一化坐标求得的 H, F 需要变换回原来的坐标系下。

$$
\bold{H}_{true}=\bold{T_2}^T\bold{H}_{norm}\bold{T_1}
$$

得分是依据重投影误差计算的，得分越高，匹配越好。最高得分为匹配对总数 * th。

从单应矩阵分解位姿

从基础矩阵分解位姿

1 这一部分要参考论文，筛选正确的姿态。这部分大概做不了什么改进。现看线程函数，之后再看这部分吧。

```
1 
```

### 1.2.7 词袋

1 外部库，有就好，改进词袋模型对系统影响不大

描述向量与特征向量的计算

### 1.2.8 显示

1 代码量中

基本为功能代码，可有可无

## 1.3 线程类

ORB-SLAM 中并行运行着三个线程：Tracking, LocalMapping, LoopClosing。这三个线程之间分工协作，互相依赖，保证了 SLAM 系统快速、精确地运行。三个线程均在 System 中创建，其中 Tracking 作为主线程，由输入的图像驱动，另外两个类分别在子线程中运行。

### 1.3.1 Tracking 追踪

追踪线程类，对输入图像进行特征提取，并与局部 MP 进行匹配，求解位姿，并判断当前帧是否满足成为关键帧的条件，如果满足，则传入 LocalMapping 进行进一步处理。

Tracking 是 ORB 中运行速度最快的线程，由用户输入的图像数据驱动。

Tracking 在主线程中运行，以图像数据驱动，所以没有 Run 函数。

Tracking 可以开启纯定位模式，这时将不会进行关键帧检测等操作。

Tracking 中共有三个提取器，左图提取器、右图提取器、单目初始化提取器。后两者根据配置按需创建。其中，单目初始化提取器目标特征点数量是配置文件中的两倍，用于提高初始化成功率。

> 追踪策略

有 TrackReferenceKeyFrame, TrackWithMotionModel, TrackLocalMap, Relocalization 种追踪方式 **原理与使用场合区别**

> 局部地图管理策略

局部地图存放于 Tracking 中

> 关键帧提取策略

**重要成员变量**

```cpp
/* 单目初始化成员变量 */
Frame mInitialFrame;  // 初始化参考帧
std::vector<int> mvIniMatches;  // 当前 F KP 匹配到参考 F KP 按当前 F KP 索引，记录参考 F KP 序号
std::vector<int> mvIniLastMatches;  // 当前 F KP 匹配到上一 F KP 按当前 F KP 索引，记录上一 F KP 序号
std::vector<cv::Point2f> mvbPrevMatched;  // 上一次匹配 KP 坐标
std::vector<cv::Point3f> mvIniP3D;  // 三角化 KP 坐标
/* 帧位姿信息 用于记录轨迹 */
list<cv::Mat> mlRelativeFramePoses;  // F 相对于 KF 位姿 TFfF  按 F 顺序索引
list<KeyFrame*> mlpReferences;  // 参考 KF  按 F 顺序索引
list<double> mlFrameTimes;  // 帧时间戳
list<bool> mlbLost;  // 帧追踪状态
/* 追踪过程变量 */
// 有冗余
int mnMatchesInliers;  // 当前帧匹配 KP 数量
KeyFrame* mpLastKeyFrame;  // 上一 KF
Frame mLastFrame;  // 上一 F
unsigned int mnLastKeyFrameId;  // 上一个 KF id
unsigned int mnLastRelocFrameId;  // 上一次重定位 F id
cv::Mat mVelocity;  // 相机运动速度  六维
KeyFrame* mpReferenceKF;  // 当前参考 KF
std::vector<KeyFrame*> mvpLocalKeyFrames;  // 局部地图 KF
std::vector<MapPoint*> mvpLocalMapPoints;  // 局部地图 MP
list<MapPoint*> mlpTemporalPoints;  // 临时 MP
```

Tracking 成员变量中没有互斥锁

#### 1.3.1.1 构造函数

构造函数中，利用 OpenCV 提供的工具读取 yaml 文件中的参数，初始化成员变量。并按根据感器类型创建需要的 ORBextractor。

#### 1.3.1.2 GrabImage* 处理图像

`cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, double timestamp)`
`cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, double timestamp)`
`cv::Mat Tracking::GrabImageMonocular(const cv::Mat & im, double timestamp)`

对图像进行类型转换后，根据传感器类型，使用对应的方法创建 Frame 对象，即完成特征提取、畸变矫正等操作，之后统一调用 Track 函数进行追踪。共有双目、深度、单目三种类型的处理函数，这些函数在 System 类的 TrackStereo, TrackRGBD, TrackMonocular 调用。

在 GrabImage* 函数中，不同类型的图像都会先转换为灰度图，再提取特征。输入图像可以为灰度图、彩色图，彩色图可以为三通道、四通道，通道顺序可以为 RGB 或 BGR。

GrabImageStereo 会分别使用左、右提取器提取图像中的特征点。

GrabImageRGBD 接受一张彩色/灰度图和深度图，并对彩色/灰度图提取特征。常用的深度图像有两种格式：uint16, float。其中，前者单个像素占用 16 bits，后者占用 32 bits。使用 uint16 灰度图时，灰度值与深度之间比例由深度因子确定，即 1m 对应的像素值，是个大于一的整数。使用 float 深度图像时，通常像素值即为以米为单位的深度。ORB-SLAM 支持使用两种深度图像格式，当使用 float 深度图时，将配置文件中的 DepthMapFactor 设为 1.0。

GrabImageMonocular 中，如果系统没有完成初始化，会调用初始化提取器提取双倍数量的特征；如果系统正常运行，则调用左图提取器。

#### 1.3.1.3 Track 追踪

`void Tracking::Track()`

追踪一帧，是 Tracking 中最核心的函数。可以看作是一个状态机，根据系统状态，按照一定逻辑调用初始化、追踪、重定位等成员函数。

```
1 是否完成初始化  ?
  F 按照传感器类型执行初始化  C `StereoInitialization` or `MonocularInitialization`
  T 是否为纯定位模式  ?
    F 追踪状态是否正常  ?
      T 检查 MP 替换  C `CheckReplacedInLastFrame`
      2 速度信息是否有效  ?
        F 根据参考关键帧跟踪  C `TrackReferenceKeyFrame`
        T 根据匀速运动模型追踪  C `TrackWithMotionModel`
      F 执行重定位  C `Relocalization`
    T 追踪状态是否正常  ?
      F 执行重定位  C `Relocalization`
      T 上一帧追踪到足够数量 MP  ?
        T 速度信息是否有效  ?
          F 根据参考关键帧跟踪  C `TrackReferenceKeyFrame`
          T 根据匀速运动模型追踪  C `TrackWithMotionModel`
        F 根据匀速运动模型追踪  C `TrackWithMotionModel`
        2 执行重定位  C `Relocalization`
        3 优先使用重定位结果
  2 如果帧间追踪成功，追踪局部地图  C 'TrackLocalMap'
  3 更新运动模型速度
  4 关键帧判断  C `NeedNewKeyFrame` and `CreateNewKeyFrame`
  5 按照局部地图匹配结果舍弃当前帧中的外点
  6 如果初始化后 5 个 KF 内追踪失败，重置系统
2 保存当前帧位姿信息，用于结束后恢复轨迹
```

#### 1.3.1.4 StereoInitialization 双目/深度初始化

`void Tracking::StereoInitialization()`

由于深度信息有效，双目/深度 SLAM 的初始化简单很多，不需要借助多视图集合。StereoInitialization 函数要求输入帧左图中提取到的特征点数多于 500 个，如果满足此要求，会通过当前帧创建初始 KF，并为所有深度有效的 KP 创建 MP，得到初始地图，这个初始地图也作为当前局部地图，在下一次匹配时使用。

#### 1.3.1.5 MonocularInitialization & CreateInitialMapMonocular 单目初始化与初始地图创建

`void Tracking::MonocularInitialization()`
`void Tracking::CreateInitialMapMonocular()`

单目初始化，依次接收两张单目相邻图像，如果这两张图像均能提取到足够数量的特征点，且成功三角化的特征点数目超过阈值，则创建初始地图，并执行一次全局 BA 优化。

单目初始化 `MonocularInitialization` 的执行分为两个阶段，接收第一帧作为参考帧，接收第二帧（代码中命名为当前帧）与参考帧进行位姿求解与三角化，两个阶段以初始化器指针是否为空作为标志位。当 Track 第一次向初始化函数传入图像时，创建初始化器，返回。当 Track 再次调用此函数时进入第二阶段，由于相邻帧之间特征点位置相差不大，所以以参考帧中 KP 位置作为预匹配位置，调用特征匹配函数 `SearchForInitialization` 在预匹配位置附近窗口内寻找匹配 KP，之后以这个匹配关系为基础，调用初始化器初始化函数 `Initialize`，通过多视图几何方法计算帧间相对位姿，并三角化特征点，得到其空间坐标。之后，仅使用成功三角化的 KP 对，其他点对认为是外点。

单目初始地图创建 `CreateInitialMapMonocular`，首先从参考帧、当前帧创建 KF，从三角化 KP 创建 MP，添加入地图，然后设置 KF 和 MP 见的观测关系，接着会进行一次全局 BA `GlobalBundleAdjustemnt`，接着以初始地图点位矢长度中位数为单位，对地图点坐标、当前帧坐标进行尺度归一化。至此初始化完成，设置成员变量后返回。

单目初始化伪代码

```
1 阶段一，创建初始化器，记录参考帧
2 阶段二，进行帧间特征点匹配  C `SearchForInitialization`
3 调用初始化器进行初始化  C `Initialize`
4 如果初始化成功，创建初始单目地图  C `CreateInitialMapMonocular`
```

单目初始地图创建伪代码

```
1 从初始帧和当前帧创建 KF，放入地图
2 从三角化 KP 结果创建 MP，放入地图，设置 MP 与 KF 观测关系
3 进行全局 BA 优化  C `GlobalBundleAdjustemnt`
4 进行尺度归一化
```

#### 1.3.1.6 TrackReferenceKeyFrame 根据参考关键帧追踪

1

使用词袋搜索与参考关键帧 MP 的关联

调用 Motion-only BA，初值为上一帧位姿

```
1 
```

#### 1.3.1.7 TrackWithMotionModel 根据运动模型追踪

1

使用投影匹配搜索与参考关键帧 MP 的关联

调用 Motion-only BA，初值为预测位姿

```
1 
```

#### 1.3.1.8 Relocalization 重定位

1

#### 1.3.1.9 TrackLocalMap 追踪局部地图

1

### 1.3.2 LocalMapping 局部建图

ORB 中的中线程。



### 1.3.3 LoopClosing 回环检测

ORB 中的慢线程。



### 1.3.4 Viewer 显示

1

## 1.4 System 系统

系统类是 ORB 向用户提供的接口，用户需要通过视觉词典路径、配置文件路径、传感器类型来初始化系统类，系统类会创建三线程。之后，向系统输入图像，追踪一帧。

**重要成员变量**

```cpp
/* 数据库 */
eSensor mSensor;  // 传感器类型  MONOCULAR=0, STEREO=1, RGBD=2
ORBVocabulary* mpVocabulary;  // 视觉字典
KeyFrameDatabase* mpKeyFrameDatabase;  // 关键帧数据库
Map* mpMap;  // 地图
/* 线程类与工具 */
Tracking* mpTracker;  // 追踪器
LocalMapping* mpLocalMapper;  // 局部建图器
LoopClosing* mpLoopCloser;  // 回环检测器
Viewer* mpViewer;  // 绘图器
FrameDrawer* mpFrameDrawer;  // 帧绘制器  用于绘制 KF
MapDrawer* mpMapDrawer;  // 地图绘制器  用于绘制 MP
/* 线程 */
// 主线程中执行 Tracking
std::thread* mptLocalMapping;  // LocalMapping 线程
std::thread* mptLoopClosing;  //  LoopClosing 线程
std::thread* mptViewer;  // 显示线程
/* Tracking 变量 */
int mTrackingState;  // 追踪状态
std::vector<MapPoint*> mTrackedMapPoints;  // 当前帧观测到的 MP
std::vector<cv::KeyPoint> mTrackedKeyPointsUn;  // 当前帧去畸变 KP
std::mutex mMutexState;  // 状态互斥锁
```

**互斥锁**

```cpp
std::mutex mMutexReset;  // 重置互斥锁
std::mutex mMutexMode;  // 模式互斥锁
std::mutex mMutexState;  // 状态互斥锁
```

### 1.4.1 构造函数

System 构造函数中检查了配置文件路径，加载了视觉字典，初始化了 KeyFrameDatabase，创建了了 FrameDrawer 和 MapDrawer，创建了 Tracking，创建并运行了线程 LocalMapping 和 LoopClosing，按照配置创建并运行显示线程。之后，设置了三个主要线程之间的指针关系。

### 1.4.2 TrackStereo 双目追踪

三种追踪函数基本相同，具体步骤是在三个主要线程类中实现的。

```
1 重定位模式检查
2 退出检查
3 追踪双目图像  C `Tracking::GrabImageStereo`
4 同步 Tracking 中当前帧信息
```

### 1.4.3 TrackRGBD 深度追踪

```
1 重定位模式检查
2 退出检查
3 追踪双目图像  C `Tracking::GrabImageRGBD`
4 同步 Tracking 中当前帧信息
```

### 1.4.4 TrackMonocular 单目追踪

```
1 重定位模式检查
2 退出检查
3 追踪双目图像  C `Tracking::GrabImageMonocular`
4 同步 Tracking 中当前帧信息
```

# 2 过程分析

帧的创建过程，地图点生成、关联？

优化的步骤，初值源于何处？

内存分析？

1

# 3 应用

调用 System 类

**注意**：追踪的结果是 T_cw，世界坐标系相对于相机坐标系的位姿，默认 z 轴向前，x 轴向右，y 轴向下。注意变换顺序。

# 99 技巧

* std::vector 内存管理

vector 是最常用的 STL 数据结构，可以随机访问。vector 中元素在内存中连续存储，可以使用中动态分配内存，如果我们总是让 vector 自适应的分配内存，会浪费很多时间。所以在使用 vector 时，最好使用 reserve 为其预分配足够的内存。例如，在特征四叉树划分时，会为子节点的特征 vector 预分配与父节点一样大小的内存。这是一种空间换时间的方式。

* 矩阵库

ORB 2 中大部分矩阵用的是 cv::Mat，提供对 Eigen 接口。ORB 3 中改为 Eigen，更规范化。

* std::vector<std::pair<..., ...>> 排序

这种类型的向量可以直接将迭代起放入 std::sort 通过 pair.first 进行排序

* 浮点精度问题

ORB 中运算、数据存储大量使用单精度浮点数。对于大多数 32bit 嵌入式设备，double 运算无法做到实时，ORB 设计时可能是考虑了在嵌入式设备上的部署，所以选择了 float。但根据大多数开发者的经验，float 的精度似乎不太够，double 是衡量精度与速度后最好的选择。并且，执行 SLAM 的机器人上通常配备了不次于 PC 的 64bit 处理器，支持 double 运算指令，可以做到实时 (如 VINS-Fusion 成功在无人机上应用了)。

从优化库的角度来说，由于 g2o 没有限制 float 或 double，类型完全由用户自己定义，所以在 g2o 中使用 float 不是问题。但 Ceres 仅支持 double, 并且其开发者在 issue 中回复 "Too much pain and suffering in single precision land."

如果要移植到 Ceres，则必须使用 double。

可以添加宏定义，在 double 和 float 间切换。

* 功能类的临时变量

在功能类，如外点剔除优化、匹配筛选等函数中，常需要用一些临时变量来保存计算中间结果和状态，这些临时变量的数量与问题的规模成正比，且无法预先估计。如，在运行前，我们不会知道一个 KF 会与多少其他 KF 形成“良好”的共视关系。ORB 中会在数据类中为功能类、线程类中使用到的一些临时变量创建成员变量，如 KeyFrame 中有许多仅在 Tracking 或 LocalMapping 中用到的变量，用作标识（如标记当前 KF 正在参与哪一个 KF 的回环检测），或存放过程量。这样会浪费一定内存，但可以解决数据关联问题，加快运行速度。

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
