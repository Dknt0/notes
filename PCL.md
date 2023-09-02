# PCL 入门

> 点云处理，好像很有趣。
> 
> Dknt2023.9.1
> 
> 参考：
> 
> [Introduction &mdash; Point Cloud Library 0.0 documentation](https://pcl.readthedocs.io/projects/tutorials/en/master/#)

PCL (Point Cloud Library) 是一个开源点云处理库，包含了滤波、特征检测、表面重建、点云配准、模型匹配、语义分割等算法。遵循 BSD 协议。PCL 又分为了一系列更小的模块，这些模块可以单独编译，以减小最终项目的大小。

## 1. 模块概述

PCL 中的常用模块如下：

* `Filters`——点云滤波

由于测量误差，原始点云数据中往往存在大量外点，干扰局部点估计。

**外点去除滤波器**——可以使用统计滤波的方法去除稀疏外点，计算每个点与其最临近N点的距离和，得到点云中所有点距离和的分布，假设这个分布是高斯分布，可以根据均值和标注差去除距离过大的外点。

**体素滤波器**——每个体素内保留一个点，保证点云中点的数量不会过大。

* `Features`——特征

特征可用于点云配准。常用特征包括**形状描述子**和**几何特征**，他们都是局部描述子。好的特征应该具有如下特点：刚体变换不变性、采样密度不变性、噪声不敏感

* `Keypoints`——关键点



`Registration`

`KdTree`

`Octree`

`Segmentation`

`Sample Consensus`

`Surface`

`Range Image`

`I/O`

`Visualization`
