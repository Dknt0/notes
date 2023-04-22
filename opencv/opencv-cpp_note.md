# OpenCV C++

API记录格式：

```cpp
/**
 * func ...
 * arg1 ...
 * ...
*/
retT func(arg1, arg2, ...);
```

# 0 OpenCV 模块

`opencv` 包含全部已安装的模块

`core` 矩阵操作函数

`imgcodecs` 图像读写函数

`highgui` 图像显示gui

`imgproc` 常用图像处理函数

# 1 图像、视频读取与显示

**图像显示与保存**

```cpp
#include <iostream>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>

using namespace std;
using namespace cv;

const char imgPath[] = "../img/px4_01.png";

int main(int argc, const char** argv) {
    Mat img = imread(imgPath, IMREAD_COLOR); // 读取图像

    if (img.empty()) {
        cout << "Could not read the image: " << imgPath << endl;
        return 1;
    }

    namedWindow("img view", WINDOW_GUI_NORMAL);
    imshow("img view", img); // 显示图像
    char k = waitKey(0); // 获取键值

    if (k == 's') {
        imwrite("../img/px4_01.jpg", img); // 保存图像
    }

    return 0;
}
```

**视频显示与保存**

# 2 core模块

## 2.1 矩阵与向量

OpenCV Mat由**矩阵头**和**指针**组成，有浅拷贝与深拷贝之分。

**图像浅拷贝**

```cpp
Mat A, B;
A = imread(...);

// 以下两种方式都是浅拷贝
Mat B(A);
C = A;
```

**ROI**可以通过浅拷贝实现

```cpp
Mat D(A, Rect(10, 10, 100, 100) ); // using a rectangle
Mat E = A(Range::all(), Range(1,3)); // using row and column boundaries
```

**图像深拷贝**

```cpp
Mat F = A.clone();
Mat G;
A.copyTo(G);
```

**矩阵创建与初始化**

```cpp
Mat M(3, 2, CV_8UC3, Scalar(0, 0, 255)); // 创建3*2矩阵，并初始化
M.create(4,4, CV_8UC(2)); // 创建矩阵，不初始化

Mat E = Mat::eye(4, 4, CV_64F); // 对角矩阵
Mat O = Mat::ones(2, 2, CV_32F); // 全1矩阵
Mat Z = Mat::zeros(3,3, CV_8UC1); // 全0矩阵
randu(R, Scalar::all(0), Scalar::all(255)); // 随机矩阵，给出上下限

Mat C = (Mat_<double>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0); // 初始化器
C = (Mat_<double>({0, -1, 0, -1, 5, -1, 0, -1, 0})).reshape(3); // 列表初始化
```

其他数据结构

```cpp
Point2f P(5, 1); // 二维点
Point3f P3f(2, 6, 7); // 三维点

Vec3b intensity = img.at<Vec3b>(y, x); // 彩色像素
uchar blue = intensity.val[0];

Vec3f intensity = img.at<Vec3f>(y, x); // 获取图像梯度
```

**读写矩阵元素**。OpenCV中，读写图像像素时一定要给出像素的类型。

```cpp
/* C数组格式访问，快速 */
uchar* p;
for (int i = 0; i < nRows; ++i) {
    p = I.ptr<uchar>(i); // 行
    for (int j = 0; j < nCols; ++j) {
        p[j] = table[p[j]]; // 列
    }
}

/* 迭代器，安全 */
const int channels = I.channels();
switch(channels)
{
case 1:
    {
        MatIterator_<uchar> it, end;
        for( it = I.begin<uchar>(), end = I.end<uchar>(); it != end; ++it)
            *it = ...;
        break;
    }
case 3:
    {
        MatIterator_<Vec3b> it, end;
        for( it = I.begin<Vec3b>(), end = I.end<Vec3b>(); it != end; ++it)
        {
            (*it)[0] = ...;
        }
    }
}

/* 随机访问，随机 */
I.at<uchar>(i,j) = ...;
```

矩阵操作。OpenCV中的矩阵本质上是二维数组，数组索引是先行数后列数，对应图像坐标先y后x。根据二维数组的特性，我们可以对其行进行扩充，注意维数需要对应。这在添加描述子时非常使用。

```cpp
test.push_back(row);//添加一行至test
```

## 2.2 图像卷积

```cpp
Mat kernel = (Mat_<char>(3, 3) << 0, -1, 0,
                                  -1, 5, -1,
                                  0, -1, 0); // 创建卷积核，用于锐化
filter2D(img, dst, img.depth(), kernel); // 2D图像卷积
```

## 2.3 图像基本操作

## 2.4 加权求和

> 基本绘图

> UI，但不是很需要

> 图像基础操作，像素操作、属性获取、ROI提取、通道拆分合并

> 逻辑运算

> 色彩空间变换、几何变换、阈值化处理、**滤波**、形态学变换、图像梯度、边缘检测、霍夫变换、图像金字塔、角点检测、**特征提取、特征匹配**

> 光流

> **相机校准、畸变矫正**、三维重建

> 机器学习，聚类、检测器

# _. SLAM 中的 OpenCV

## _.1 特征提取与匹配

SLAM中常使用ORB特征，提取特征后计算OBRIEF描述子。特征提取步骤如下：检测特征、计算描述子、特征匹配、匹配点对筛选。这些步骤都可以通过OpenCV提供的函数完成。

创建特征检测器、描述子计算器、特征匹配器，他们本质上是`cv::Ptr`指向了不同类型的对象。

```cpp
// 特征检测器
Ptr<FeatureDetector> detector = ORB::create(500); // 检测500个特征点
// 描述子计算器
Ptr<DescriptorExtractor> descriptor = ORB::create();
// 特征匹配器
Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
```

使用特征检测器**检测特征**：

```cpp
// 特征点存放在向量中
vector<KeyPoint> keypoints_1;
detector->detect(img_1, keypoints_1);
```

> 注意，特征检测的结果往往是不均匀的，需要将图片分为多个小块处理。

其中`KeyPoint`对象包含了特征点在图像中的位置、方向、尺度、置信度等信息。

```cpp
// 特征点使用举例
KeyPoint kpt;
kpt.pt.x; // x坐标
kpt.pt.y; // y坐标
kpt.angle; // 方向
```

计算特征点的描述子。

```cpp
// 描述子存放在Mat中，矩阵的一行存放一个256维描述子（8*32）
Mat descriptors_1;
descriptor->compute(img_1, keypoints_1, descriptors_1);
```

依据描述子进行特征匹配。得到第二组描述子中与第一组最相近的点。DMatch中的匹配按照第一组描述子的顺序排列。

```cpp
// 检测结果，即匹配点对，存放在DMatch中
vector<DMatch> matches;
matcher->match(descriptors_1, descriptors_2, matches);
```

DMatch包含了在两个描述子中的索引值，与距离信息（相似程度）。

```cpp
DMatch mch;
mch.queryIdx; // 第一组描述子中的索引
mch.trainIdx; // 第二组描述子中的索引
mch.distance; // 匹配距离
```

匹配点对中包含大量误匹配，因此要进行匹配点对筛选。下面的程序中使用STL库的算法求出最大、最小距离，然后按最小距离设置阈值，筛选得到匹配点对。

```cpp
auto min_max = minmax_element(matches.begin(), matches.end(),
                                  [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
double min_dist = min_max.first->distance;
double max_dist = min_max.second->distance;
for (int i = 0; i < descriptors_1.rows; i++) {
    if (matches[i].distance <= max(2 * min_dist, 30.0)) {
        good_matches.push_back(matches[i]);
    }
}
```

使用如下函数绘制两张图片的显示结果。

```cpp
drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
```

## _.2 特征点深度恢复

使用ptr获取像素点的值

```cpp
ushort depth = imgsDepth[0].ptr<ushort>(static_cast<int>(keypoint.pt.y))[static_cast<int>(keypoint.pt.x)];
```
