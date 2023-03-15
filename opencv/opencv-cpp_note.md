# OpenCV


API记录格式：
```cpp
/**
 * func ...
 * arg1 ...
 * ...
*/
retT func(arg1, arg2, ...);
```

# 0 OpenCV模块

`opencv` 包含全部已安装的模块

`core` 定义了OpenCV的基础功能

`imgcodecs` 图像读写函数

`highgui` 图像显示




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

## 2.2 图像卷积





> 基本绘图



> UI，但不是很需要


> 图像基础操作，像素操作、属性获取、ROI提取、通道拆分合并

> 逻辑运算


> 色彩空间变换、几何变换、阈值化处理、**滤波**、形态学变换、图像梯度、边缘检测、霍夫变换、图像金字塔、角点检测、**特征提取、特征匹配**

> 光流

> **相机校准、畸变矫正**、三维重建

> 机器学习，聚类、检测器




