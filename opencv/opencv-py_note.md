<!-- 研一上学期教学实践，OpenCV简要说明 -->
<!-- Author: Dknt -->
<!-- Date: 2022.12 -->
# OpenCV-Python简介

OpenCV是一个跨平台的开源图像处理库。简述历史，Intel，俄罗斯OpenCV团队，柳树车库。OpenCV包含了一系列图像处理和机器学习算法，支持C++、Python、Java、Matlab等编程语言，支持基于CUDA的GPU加速。
OpenCV-Python是OpenCV的Python API，结合了C++和Python两种语言的优点。

## 1. Introduction to OpenCV

> 介绍OpenCV的安装过程

OpenCV-Python有两种安装方式：源码编译与安装预编译包。源码编译相对复杂，我们通过第二种方法，使用pip工具安装。打开终端，输入以下命令：

```shell
pip install numpy
pip install opencv-python
```

安装numpy用于支持矩阵运算，OpenCV中的大部分数据结构都是numpy.ndarray类型。opencv-python为OpenCV-Python的主要模块。  
也可以通过如下命令安装OpenCV-Python的主要模块和拓展模块。
```shell
pip install opencv-contrib-python
```

OpenCV早期使用C语言编写，之后用C++重写。在python中使用OpenCV时，需要导入cv2包，cv2代表底层使用C++ API，cv代表底层使用C API，通常将cv2导入为cv，以保证向后兼容。可以通过以下代码测试是否安装成功。

```py
#!/usr/bin/python3
import cv2 as cv
print(cv.__version__)
```
运行，如果输出版本号，则安装成功。

**什么是数字图像？**

**Numpy简要教程**

> cv2 as cv的原因：因为别人都这么写，我们也这么写，显得专业。

## 2. Gui Features in OpenCV

### 2.1 图像读写与显示

> 图像读取、显示

**图像读取**的函数是`cv.imread(filename: str, flags: int)`，第一个参数是需要读取图片的名字，第二个参数确定导入格式，返回值为图片。可以导入的格式如下：

* `IMREAD_COLOR` 按RGB8格式导入图片，即彩色图片，有三个颜色通道，每个通道是8位。这是默认的导入格式。
* `IMREAD_UNCHANGED` 按图片原格式导入，若图片存在alpha通道，则导入。
* `IMREAD_GRAYSCALE` 将图片导入为灰度图。

导入后图片在python中的格式为`numpy.ndarray`，也就是说，我们自己创建的ndarray矩阵也是图片。OpenCV支持读取bmp、pbm、jpeg、png等格式的图片。需要注意，OpenCV中彩色图片的通道顺序为BGR，而不是RGB。

OpenCV中，窗口用于显示图片。**窗口创建**的函数是`cv.namedWindow(winname: Any, flags: int)`，第一个参数为窗口名，第二个参数为窗口类型。通常，默认类型可以满足我们的需求。

**窗口大小更改**的函数是`cv.resizeWindow(winname: Any, width: Any, height: Any)`，第一个参数为窗口名，第二三个参数分别为窗口宽度、高度。

**关闭所有窗口**的函数为`cv.destroyAllWindows()`。

**图片显示**函数为`cv.imshow(winname: Any, mat: Any)`，第一个参数为窗口名，第二个参数为要显示的图片。

等待**键盘输入**函数为`cv.waitKey(delay)`，参数为等待的时间，单位为毫秒。图片显示的时间很短，只有一瞬间，我们需要加入延时，才能看到图片。当参数为0时为无限等待，做摄像头显示时等待时间为1，视频文件显示时为25。

**图片保存**的函数为`cv.imwrite(filename: str, img: Mat)`，第一个参数为要保存图片的名字（包含格式），第二个参数为图片。

图像读取、显示、保存的例子如下：

> src/img_rs.py

```py
#!/usr/bin/python3
import cv2 as cv
import sys

img = cv.imread('image/image_display.jpg', cv.IMREAD_COLOR)
if img is None:
    sys.exit('Failed to import image!')
cv.namedWindow('image', cv.WINDOW_FREERATIO)
cv.imshow('image', img)
while True:
    key = cv.waitKey(0)
    if key == ord('q'):
        break
    elif key == ord('w'):
        cv.imwrite('image/image_display.png', img)
        break
```

在这个例程中，我们读取并显示了图片，如果我们按下q（当图片显示窗口被激活时）退出，按下w保存图片并退出，否则程序将一直循环等待，直到我们按下这两个按键中的一个。


### 2.2 视频读写与显示

视频是由许多张图片组成的，许多时候，我们并不关心相邻帧之间的联系，这时视频处理的过程就是重复处理单张图片直到视频结束。我们需要一些工具，用于捕获视频流中的图片，以及将若干张图片保存为一个视频。

> 读摄像头、视频

为了读取视频流，需要**创建视频捕获对象**`cv.VideoCapture()`，参数可以是设备编号或视频名称（字符串）。设备编号为整数，例如，0为默认摄像头。

创建对象后，可以通过`cap.read()`（假设我们的对象叫cap）**逐帧捕获图片**，这个函数返回一个布尔变量与图片，布尔变量代表捕获是否成功。

捕获完成后，我们需要使用函数`cap.release()`**释放摄像头**。

一个摄像头读取的例子如下：

> src/camera_read.py

```py
#!/usr/bin/python3
import cv2 as cv
import time
import sys

cap = cv.VideoCapture(0)
if not cap.isOpened():
    sys.exit('Cannot open camera.')
while True:
    ret, frame = cap.read()
    if not ret:
        print('Can\'t receive frame.')
        break
    cv.imshow('image', frame)
    key = cv.waitKey(1)
    if key == ord('q'):
        break
    if key == ord('w'):
        cv.imwrite('image/' + str(time.time()) + '.jpg', frame)
cap.release()
cv.destroyAllWindows()
```

程序读取并显示默认摄像头，按下s时保存一张图片，按下q时退出并释放所有摄像头。

> 保存视频

为了保存视频，我们需要**创建视频保存对象**`cv.VideoWriter()`。创建该对象需要如下参数：视频名称（字符串）、FourCC码、帧率、分辨率（含宽高两个元素的元组）。保存黑白视频时，需要添加第五个参数为False，是否为彩色图片，默认为True。
FourCC码是用于指定视频编解码器的4字节代码。它依赖于平台。在Linux下可使用XVID，在Windows下使用DIVX。

向**视频写入图片**时，需调用`out.write()`（假设我们的对象叫out）函数。

同样的，使用后，需要**释放视频保存对象**`out.release()`。
一个视频保存的例子如下：

```python
#!/usr/bin/python3
import cv2 as cv
import sys

cap = cv.VideoCapture(0)
if not cap.isOpened():
    sys.exit('Cannot open camera.')
fourcc = cv.VideoWriter_fourcc(*'XVID')
out = cv.VideoWriter('image/output.avi', fourcc, 20.0, (640, 480))
while True:
    ret, frame = cap.read()
    if not ret:
        print('Can\'t receive frame.')
        break
    cv.imshow('image', frame)
    out.write(frame)
    key = cv.waitKey(1)
    if key == ord('q'):
        break
cap.release()
out.release()
cv.destroyAllWindows()
```

### 2.3 基本绘图工具

> 直线、圆、正方形、椭圆、文字

Opencv提供了一系列画图工具，例如绘制直线、圆、正方形、文字等，这些工具常用于图像处理结果标注。
这些函数有一些相同的参数：

* `img` 需绘制图形的图像。
* `color` 图形颜色，对于彩色图像格式为元组(blue, green, red)。
* `thickness` 线宽。当写为-1时填充封闭图形。
* `lineType` 线格式，8连通或4连通。输入cv.LINE_AA会使曲线更为平滑。

**绘制直线**
```py
cv.line(img, pt1, pt2, color, thickness, lineType)
```

* `pt1`为起点坐标
* `pt2`为终点坐标

**绘制圆**
```py
cv.circle(img, center, radius, color, thickness, lineType)
```

* `center`为圆心坐标
* `radius`为半径

**绘制矩形**
```py
cv.rectangle(img, pt1, pt2, color, thickness, lineType)
```

* `pt1`为左上角顶点坐标
* `pt2`为右下角顶点坐标

> 通过rectangle画出来的矩形与图像边框平行

**绘制椭圆弧**
```py
cv.ellipse(img, center, axes, angle, startAngle, endAngle, color, thickness, lineType)
```

* `center`为椭圆中心坐标
* `axes`为两个轴长度
* `angle`为旋转角度（逆时针方向）
* `startAngle`和`endAngle`为圆弧起始位角度和终止角度（顺时针为正）

**绘制多边形**
```py
cv.polylines(img, pts, isClosed, color, thickness, lineType)
```

* `pts`为顶点列表
* `isClosed`为是否封闭。

> polylines也可以用于绘制多条直线。

**绘制文字**
```py
cv.putText(img, text, org, fontFace, fontScale, color, thickness, lineType)
```

* `text`为需要书写的字符串
* `org`为书写位置；`fontFace`为字体，字体宏定义`cv.FONT_HERSHEY_*`
* `fontScale`文字大小。

一个使用上述函数绘图的例子如下：

```py
#!/usr/bin/python3
import numpy as np
import cv2 as cv

img = np.zeros((512, 512, 3), np.uint8)

cv.line(img, (100, 200), (300, 200), (255, 0, 0), 20)
cv.circle(img, (100, 100), 30, (0, 255, 0), -1)
cv.rectangle(img, (65, 65), (135, 135), (255, 255, 0), 1)
cv.ellipse(img, (300, 350), (150, 90), 0, 0, 360, (0, 255, 255), 3, cv.LINE_AA)
pts = np.array([[50, 400], [100, 450], [200, 450], [150, 400]], np.int32)
pts = pts.reshape((1,-1,1,2))
# pts = np.array([[[[ 50, 400]], [[100, 450]], [[200, 450]], [[150, 400]]])

cv.polylines(img, pts, True, (255, 0, 255), 4)
cv.putText(img, 'Green circle', (65, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1, cv.LINE_AA)

cv.imshow('test', img)
cv.waitKey(0)
```

绘制结果如图：
*图*

### 2.4 基础UI

> 用的不多，通常只用OpenCV来处理图像，不需要他做UI

OpenCV提供了一系列交互接口，方便用户直接在OpenCV窗口中更改配置。

> 鼠标事件

OpenCV支持许多鼠标事件，例如单击、双击、右键等。具体的支持类型可以执行如下脚本查看。

```py
import cv2 as cv
events = [i for i in dir(cv) if 'EVENT' in i]
print( events )
```

事件的处理基于回调函数。我们需要编写回调函数，并使用`cv.setMouseCallback()`注册鼠标回调函数。当事件发生时，OpenCV会将事件类型和鼠标的位置传递给回调函数，由回调函数完成事件的处理。

**注册鼠标回调函数**

```python
cv.setMouseCallback(windowName, onMouse)
```

* `windowName` 窗口名称，字符串
* `onMouse` 回调函数，格式为`draw_circle(event, x, y, flags, param)`

> 控件：跟踪条

跟踪条可以作为按钮和开关使用，OpenCV不提供按钮和开关控件。

**创建跟踪条**

```python
cv.createTrackbar(trackbarName, windowName, value, count, onChange)
```

* `trackbarName` 跟踪条名称，字符串
* `windowName` 窗口名称，字符串
* `value` 默认值
* `count` 最大值
* `onChange` 回调函数，格式为`fun(x)`

**获取跟踪条位置**

```python
cv.getTrackbarPos(trackbarname, winname)
```

* `trackbarname` 跟踪条名称，字符串
* `windowName` 窗口名称，字符串


## 3. Core Operations

### 3.1 图像基础操作

这一部分的内容基本都是基于Numpy完成的，良好的Numpy基础有助于优化OpenCV代码。

**像素值读写**

因为图像是numpy.ndarray，像素是矩阵中的元素，因此可以直接通过索引读写像素。

```py
pixel = img[100, 100] # 读像素点
pixel_blue = img[100, 100, 0] # 读像素点蓝色通道
img[100, 100] = [255, 255, 0] # 修改像素点
img[100, 100, 0] = 0 # 修改像素点蓝色通道
```

对于单个像素单个通道，更快的读写方式为：

```py
img.item(10,10,2) # 读像素点红色通道
img.itemset((10,10,2),100) # 写像素点红色通道
```

**获取图像属性**

包括行数、列数、通道数、图像数据类型、像素点数等。

```py
img.shape # 返回图像行数、列数、通道数
img.size # 返回像素个数
img.dtype # 返回图像数据类型
```

> 许多OpenCV中的bug是由于图像数据类型不正确导致的。

**兴趣区域ROI**

定位、目标检测时常常会用到ROI区域，这样可以提高算法效率。例如在检测眼睛时，可以先检测人脸，再在人脸区域内检测眼睛。因为眼睛很小，在整张图片中直接检测小目标会比检测大目标浪费更多时间。

ROI是通过索引实现的，例如：

```
temp = img[280:340, 330:390]
img[273:333, 100:160] = temp
```

在上述第一行代码中，我们创建了ROI区域，并将这个区域拷贝到第二行中的位置。

**拆分与合并图像通道**

使用拆分函数可以将彩色图像的三个通道分解，使用合并函数实现反过程。

```py
b,g,r = cv.split(img) # 拆分
img = cv.merge((b,g,r)) # 合并
```

> 合并函数很耗费时间，仅在必要时使用。

**生成边框**

```py
cv.copyMakeBorder(src, top, bottom, left, right, borderType, dts)
```

* src 图像
* top, bottom, left, right 填充宽度
* borderType 填充类型
* * cv.BORDER_CONSTANT 常数填充，使用这一方法是，需在下一个参数写填充颜色
* * cv.BORDER_REFLECT 对称填充，如：fedcba|abcdefgh|hgfedcb
* * cv.BORDER_REFLECT_101 如：hgfedcb|abcdefgh|gfedcba
* * cv.BORDER_REPLICATE 如：aaaaaa|abcdefgh|hhhhhhh
* * cv.BORDER_WRAP 如：cdefgh|abcdefgh|abcdefg
* dts 常数填充的颜色

卷积会是图像的长、宽减小卷积核长、宽减一的大小，所以在进行卷积前，通常会为原图像生成边框，以保证卷积运算后图像大小不变。

### 3.2 图像算术逻辑运算

**图像加法**

加法可以通过`cv.add()`或直接将图片相加实现。相加的两张图片应该具有相同的大小与通道数，也可以将图片与标量相加。

注意：`cv.add()`是饱和加法运算，而Numpy加法是模加法，差别如下：

```py
>>> x = np.uint8([250])
>>> y = np.uint8([10])
>>> print( cv.add(x,y) ) # 250+10 = 260 => 255
[[255]]
>>> print( x+y )          # 250+10 = 260 % 256 = 4
[4]
```

所以，图片相加使用`cv.add()`。

**图像混合**

图像按权重相加，实现透明的相加效果。使用函数`cv.addWeighted(img1, α, img2, ß, γ)`实现如下功能：

$$
dst = \alpha*img1+\beta*img2+\gamma
$$

**位运算**

与、或、非、异或逐元素位运算。

```py
cv.bitwise_and(src1, src2, dts, mask) # 与
cv.bitwise_or(src1, src2, dts, mask) # 或
cv.bitwise_not(src1, dts, mask) # 非
cv.bitwise_xor(src1, src2, dts, mask) # 异或
```

位运算实例：

```py
#!/usr/bin/python3
import numpy as np
import cv2 as cv

img_cat = cv.imread('image/cat_1.jpg')
img_cv = cv.imread('image/opencv-logo-white.png')

img_cat_roi = img_cat[0 : img_cv.shape[0], 0 : img_cv.shape[1]]

img_cv_gray = cv.cvtColor(img_cv, cv.COLOR_BGR2GRAY)
ret, mask = cv.threshold(img_cv_gray, 10, 255, cv.THRESH_BINARY)
mask_inv = cv.bitwise_not(mask)
cat_mask = cv.bitwise_and(img_cat_roi, img_cat_roi, mask = mask_inv)
cv_mask = cv.bitwise_and(img_cv, img_cv, mask = mask)

img_sum = cv.add(cat_mask, cv_mask)
img_cat[0 : img_cv.shape[0], 0 : img_cv.shape[1]] = img_sum

cv.namedWindow('image', cv.WINDOW_GUI_NORMAL)
cv.imshow('image', img_cat)
cv.waitKey(0)
```

### 3.3 算法速度工具

**算法运行时间测量**

`cv.getTickCount()` 获得当前时钟周期数
`cv.getTickFrequency()` 获得时钟频率

可以通过如下方式计算运行时间：
```py
e1 = cv.getTickCount()
# your code execution
e2 = cv.getTickCount()
time = (e2 - e1)/ cv.getTickFrequency()
```

**优化代码**

OpenCV默认使用优化代码，通过`cv.useOptimized()`查看。可以通过`cv.setUseOptimized(False)`关闭优化。


**Ipython测量运行时间**

Ipython提供魔术指令`%timeit`用来测量时间，他会将函数运行多次，并返回运行总时间。
Ipython还提供了其他一系列魔术指令，非常方便。

**提高代码效率的建议**
* 避免在Python中使用循环
* 尽量将算法矢量化
* 利用缓存一致性？
* 除非必要，不要复制矩阵

## 4. Image Processing in OpenCV

> **这一章包含大部分的传统图像算法，需要重点描述**

### 4.1 色彩空间变换

OpenCV支持超过150种色彩空间。色彩空间变换，和阈值化处理可以用来进行颜色识别。

**色彩空间变换**

```py
cv.cvtColor(input_image, flag)
```

* `input_image` 输入图片
* `flag` 变换类型
* * `cv.COLOR_BGR2GRAY` BGR变灰度图
* * `cv.COLOR_BGR2HSV` BGR变HSV

> HSV，色度、饱和度、亮度。OpenCV中色度的范围是0~179
> 目标颜色色度范围计算：由目标颜色BRG值对应的色度加减10得到，饱和度和亮度可取较大范围，如50~255

**图像区间判断**

`cv.inRange(src, lowerBound, upperbBound)`

* src 图像
* lowerBound 下限
* upperbBound 上限

> 类似于三维的阈值化处理

一个颜色检测的实例如下：

> 

```py
#!/usr/bin/python3
import numpy as np
import cv2 as cv

cap = cv.VideoCapture(0)

while True:
    ret, img = cap.read()

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])

    mask = cv.inRange(hsv, lower_blue, upper_blue)

    img = cv.bitwise_and(img, img, mask=mask)

    cv.imshow('test', img)
    if cv.waitKey(1) == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
```

### 4.2 图像几何变换

图像二维几何变换，本质上是将变换矩阵与像素位置相乘。变换矩阵大小有2\*3和3\*3两种大小，对应以下两种函数。2*3的变换矩阵是非齐次形式，它的齐次形式中最后一行为(0,0,1)。


```py
cv.warpAffine(src, M, dsize)
```

* `src` 图像
* `M` 变换矩阵，2*3，float64类型
* `dsize` 元组，变换后图像大小(width, height)

```py
cv.warpPerspective(src, M, dsize)
```

* `src` 图像
* `M` 变换矩阵，3*3，float64类型
* `dsize` 元组，变换后图像大小(width, height)

**图像缩放**

图像缩放可以使用`cv.resize`函数实现，这一函数提供不同的缩放方法，可以缩放到指定大小，或按指定比例缩放。

```py
img2 = cv.resize(img, dsize, fx, fy)
```

* `img` 原图像
* `dsize` 目标大小，(width, height)，按比例缩放时写None
* `fx`, `fy` 缩放比例，按大小缩放时省略

**图像平移**

平移时使用的变换矩阵如下，其中$t_x$，$t_y$为移动的距离。

$$
\begin{bmatrix}
    1 & 0 & t_x \\
    0 & 1 & t_y
\end{bmatrix}
$$

一个仿射变换的例子如下：

```py
#!/usr/bin/python3
import numpy as np
import cv2 as cv

img = cv.imread('image/cat_1.jpg')
M = np.array([[1, 0, 50], [0, 1, 50]], np.float32)
img2 = cv.warpAffine(img, M, [img.shape[1], img.shape[0]])
cv.imshow('test', img2)
cv.waitKey(0)
```

**图像旋转**

OpenCV提供基于图像中某点旋转特定角度的旋转，可以通过如下命令生成变换矩阵。

`M = cv.getRotationMatrix2D(center, angle, scale)`

* `center` 中心(y, x)
* `angle` 角度，单位为度
* `scale` 变换尺度

之后可以使用`cv.warpAffine`实现旋转。

**仿射变换（错切）**

仿射变换后，原图像中的平行线依然平行。仿射变换矩阵第三行为(0, 0, 1)，由变换前后的三组点确定。

```py
pts1 = np.float32([[50,50],[200,50],[50,200]])
pts2 = np.float32([[10,100],[200,50],[100,250]])
M = cv.getAffineTransform(pts1, pts2)
```

**投影变换**

投影变换后，原图像中的直线仍为直线。投影变换矩阵由变换前后的四组点确定，这些点中任意三个不共线。

```py
pts1 = np.float32([[56,65],[368,52],[28,387],[389,390]])
pts2 = np.float32([[0,0],[300,0],[0,300],[300,300]])
M = cv.getPerspectiveTransform(pts1,pts2)
```

> 教材："Computer Vision: Algorithms and Applications", Richard Szeliski


翻转图片`cv.flip()`。


### 4.3 阈值化处理

阈值化处理是对**灰度图**进行的，分为普通阈值化处理与自适应阈值化处理。

**普通阈值化**

普通阈值化中的阈值是全局阈值，对图像中的每个像素点都运用相同的阈值。

```py
ret, thresh = cv.threshold(src, thresh, maxval, type)
```

* `src` 原图像
* `thresh` 阈值
* `maxval` 上限，常取255
* `type` 阈值化方法
* * `THRESH_BINARY` 二值化，大于阈值为255
* * `THRESH_BINARY_INV` 二值化，小于阈值为255
* * `THRESH_TRUNC` 截断，小于阈值
* * `THRESH_TOZERO` 阈值归零，小于阈值为0
* * `THRESH_TOZERO_INV` 阈值归零，大于阈值为0
* * `THRESH_OTSU` Otsu算法计算阈值
* `ret` 阈值
* `thresh` 阈值化结果

**自适应阈值化**

自适应阈值化对每个区域使用不同的阈值。在局部光照条件不同的图像中，使用自适应阈值能取得更好的效果。

```py
thresh = adaptiveThreshold(src, maxValue, adaptiveMethod, thresholdType, blockSize, C)
```

* `src` 原图像
* `maxValue` 最大值
* `adaptiveMethod` 自适应方法
* * `ADAPTIVE_THRESH_MEAN_C` 阈值为邻域均值减去相对阈值
* * `ADAPTIVE_THRESH_GAUSSIAN_C` 阈值为邻域高斯加权均值减去相对阈值，通常效果更好
* `thresholdType` 阈值化方法，类型同上
* `blockSize` 邻域大小
* `C` 相对阈值
* `thresh` 阈值化结果

**Otsu二值化**

Otsu通过图像直方图确定最优阈值，使用Otsu方法确定阈值时，参数中的全局阈值无效。使用方法如下。

```py
cv.threshold(img,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
```

> 噪声存在时，**适当的高斯滤波**能使直方图中的峰值更明显，可以为Otsu二值化效果带来很大提升。

### 4.4 图像平滑

图像平滑是指使用各种低通滤波器对图像进行卷积（Convolution），可以有效抑制噪声干扰，是重要的图像前处理步骤。

> 低通滤波器用于消除噪音，高通滤波器用于边缘检测

`cv.filter2D(img, ddepth, kernel)`

* `src` 原图像
* `ddepth` 图像深度，-1代表与原图像相同
* `kernel` 卷积核，长宽为**奇数**

> 图像深度是指用于存储一个像素的色彩信息所需要的bit，例如RGB888格式的深度为24


**均值滤波（Averaging）**

均值滤波卷积核中每个元素的值都相同，所有元素和为1。使用`cv.blur()`实现。

```py
res = cv.blur(img, (kwidth, kheight))
```

**高斯滤波（Gaussian Blurring）**

高斯滤波是最常用的滤波方法，能有效去除高斯白噪声。

```py
gaussian = cv.GaussianBlur(img, (kwidth, kheight), sigmaX, sigmaY)
```

* `sigmaX` x方向标准差
* `sigmaY` y方向标准差。如果之给出x方向标准差，则y与之相同；如果标准差为0，则依据卷积核大小自动计算标准差。

> 可以通过`cv.getGaussianKernel()`获得高斯卷积核

**中值滤波（Median Blurring）**

中值滤波的滤波核为非线性函数，选择范围内像素点的中值，对于椒盐噪声有良好的抑制作用。

> 椒盐噪声，在图像中随机出现黑点（椒）或白点（盐）。

```py
median = cv.medianBlur(img, kernel_size)
```

**双边滤波（Bilateral Filtering）**

双边滤波能在滤去噪声的同时保留图像的边缘，其原理是同时考虑图像的空域信息和值域信息。双边滤波卷积核是**空域核**和**值域核**的叠加，空域核为高斯核，值域核依据像素颜色差值计算，与中心点颜色相近的像素点拥有更大权重，颜色差别大像素点权重较小。双边滤波的**时间复杂度较高**。双边滤波无法过滤高频噪声，对椒盐噪声的滤除效率不高。

```py
bift = cv.bilateralFilter(src, d, sigmaColor, sigmaSpace)
```

* `d` 卷积核大小
* `sigmaColor` 值域标准差
* `sigmaSpace` 空域标准差

> 参考：https://zhuanlan.zhihu.com/p/127023952

> **噪声的添加**

### 4.5 形态学变换

形态学变换（Morphological Transformations）是一系列应用于二值图的、基于区域形状的简单变换。最基本的形态学变换是腐蚀和膨胀，其他操作都是在这两种操作的基础上进行加减、位运算得到的。

> Blob(Binary large object)分析，是指从图像中提取前景（foreground），即二值图中白色区域，再对该二值区域进行面积、周长重心等特征的分析。

**腐蚀**

腐蚀（Erosion）使前景变小。使用核遍历图像，如果核内所有像素均为1，则保留中心点为1，否则中心点为0。常用于过滤小的白噪声，分离相连区域。

```py
kernel = np.ones((5, 5), np.uint8)
erosion = cv.erode(img, kernel, iterations = 1) # iterations为迭代次数
```

> 腐蚀与膨胀的卷积核为二值卷积核，且可以不全为1

**膨胀**

膨胀（Dilation）使前景变大。使用核遍历图像，如果核内至少有一个像素为1，则中心点为1，否则中心点为0。会增大噪声影响。

```py
dilation = cv.dilate(img, kernel, iterations = 1)
```

> 以下形态学运算都调用cv.morphologyEx()，加上不同的类型实现

**开运算**

开运算（Opening）即先腐蚀再膨胀。用于去除噪声，分离前景。

```py
opening = cv.morphologyEx(img, cv.MORPH_OPEN, kernel)
```

**闭运算**

闭运算（Closing）即先膨胀再腐蚀。用于填充前景内空隙、合并前景。

```py
closing = cv.morphologyEx(img, cv.MORPH_CLOSE, kernel)
```

**形态学梯度**

形态学梯度（Morphological Gradient）是膨胀与腐蚀的差值。

```py
gradient = cv.morphologyEx(img, cv.MORPH_GRADIENT, kernel)
```

**白色顶帽变换**

白色顶帽变换（White Top-hat）是原图像与开运算的差值。

```py
tophat = cv.morphologyEx(img, cv.MORPH_TOPHAT, kernel)
```

**黑色顶帽变换**

黑色顶帽变换（Black Top-hat）是闭运算与原图像的差值。

```py
blackhat = cv.morphologyEx(img, cv.MORPH_BLACKHAT, kernel)
```

> 其他常见形态学变换：骨架抽取、极线腐蚀、击中击不中变换、颗粒分析、流域变换

**形态学卷积核**

形态学变换可以使用不是正方形的卷积核，通过以下函数获取结构元素：

```py
cv.getStructuringElement(type, size)
```

* type 形状
* * `MORPH_RECT` 矩形
* * `MORPH_ELLIPSE` 椭圆
* * `MORPH_CROSS` 十字
* size 大小，1*2元组

> 参考：https://homepages.inf.ed.ac.uk/rbf/HIPR2/morops.htm

### 4.6 图像梯度

图像梯度的计算可以使用三种不同的算子：Sobel算子、Scharr算子、Laplacian（拉普拉斯）算子。Sobel算子是高斯滤波和微分运算的叠加，对噪声有良好的抑制作用。其中Sobel算子、Scharr算子是x或y方向梯度，Laplacian算子为全向梯度绝对值，在每个方向上各自为Sobel算子。

**Sobel算子**

```py
sobel = cv.Sobel(src, ddepth, dx, dy, ksize)
```

* `src` 原图像
* `ddepth` 图像深度。-1为保持不便
* `dx` x方向标志
* `dy` y方向标志
* `ksize` 卷积核大小。如果为-1，使用3*3的Scharr算子

**Scharr算子**

```py
scharr = Scharr(src, ddepth, dx, dy, ksize)
```

* `src` 原图像
* `ddepth` 图像深度，-1为保持不变
* `dx` x方向标志
* `dy` y方向标志
* `ksize` 卷积核大小

**Laplacian算子**

```py
laplacian = cv.Laplacian(src, ddepth)
```

* `src` 原图像
* `ddepth` 图像深度，-1为保持不便

梯度运算中，黑白边（从黑到白）结果为正值，白黑边结果为负值。如果输出类型为无符号数，白黑边的梯度会被记为0，导致梯度信息的丢失。因此在梯度运算时，应先将结果输出为带符号的类型，再取绝对值映射到RGB888上。例如：

```py
sobelx64f = cv.Sobel(img, cv.CV_64F, 1, 0, ksize=5)
abs_sobel64f = np.absolute(sobelx64f)
sobel_8u = np.uint8(abs_sobel64f)
```

### 4.7 Canny边缘检测

Canny是一种很常用的边缘检测算法，步骤如下：

1. 去噪。对图像进行5*5高斯滤波。噪声对边缘检测会造成很大干扰。
2. 使用Sobel核计算x、y方向梯度，并计算梯度强度与方向，方向被舍入到水平、垂直、两个对角方向。
3. 非极大值抑制（Non-maximum Suppression）。将像素与其梯度方向上相邻两个像素的梯度强度作比较，如果该像素为极大值，则记为边缘，否则不是边缘。
4. 滞后阈值化（Hysteresis Thresholding）。对梯度强度设置上下两个阈值，上一步得到的边缘中，梯度强度大于上阈值的，认为是强边缘；强度小于下阈值的，认为不是边缘；梯度强度在上下阈值中间的像素点，如果与强边缘相邻，则认为是边缘，否则认为不是边缘。

```py
edges = cv.Canny(img, threshold1, threshold2)
```

* `img` 原图像
* `threshold1` 下阈值
* `threshold2` 上阈值

### 4.8 图像金字塔

### 4.9 轮廓

### 图像直方图

### 频域变换

### 模板匹配

### 4.13 霍夫直线变换

霍夫变换是一种常用的形状检测算法，只要将形状可表达为带参数的公式，就可以使用霍夫变换进行检测。霍夫变换将图像中的图形映射为参数空间中的点，用图像中的点为参数空间中的点投票，参数空间中得票多的点为图形。

OpenCV提供两种霍夫直线变换函数。`cv.HoughLines()`——霍夫变换，`cv.HoughLinesP()`——概率霍夫变换。后者效果更好，随机选点投票，计算速度更快，且直接返回检测到直线的两端点。

```py
lines = cv.HoughLines(image, rho, theta, threshold)
```

* `image` 原图像，二值图，通过阈值化或Canny检测得到
* `rho` 距离精度
* `theta` 角度精度
* `threshold` 投票阈值，位于线上点最少数量
* `rho, theta = line[i][0]` 返回值为检测到直线，用角度和原点距离表示

```py
lines = cv.HoughLinesP(image, rho, theta, threshold)
```

* `image` 原图像，二值图，通过阈值化或Canny检测得到
* `rho` 距离精度
* `theta` 角度精度
* `threshold` 投票阈值，位于线上点最少数量。因为概率霍夫变换只使用部分边缘点，相同条件下阈值应比霍夫变换小
* `x1, y1, x2, y2 = line[i][0]` 返回值为检测到直线，用端点坐标表示

### 4.14 霍夫圆变换

圆由中心、半径决定，参数空间是三维的。为减少运算量，对圆形的检测采用霍夫梯度法。输入为8位单通道灰度图，**不是边缘二值图**。

```py
circles = cv.HoughCircles(image, method, dp, minDist, param1, param2, minRadius, maxRadius)
```

* `image` 原图像，8位单通道灰度图
* `method` 检测方法，常用`HOUGH_GRADIENT`
* `dp` 检测检测器比例，1为与原图像相同，2为原图像一半
* `minDist` 被检测到圆心的最小距离，用于NMS
* `param1` Canny高阈值，低阈值为高阈值的一半
* `param2` 参数空间阈值，
* `minRadius` 最小半径
* `maxRadius` 最大半径
* `i = circles[0, i]`  `(i[0], i[1])`圆心位置，`i[2]`半径

### 分水岭算法图像分割

### 使用 GrabCut 算法进行交互式前景提取？

## 5. Feature Detection and Description

> *特征的理解*

机器视觉是特征工程，传统模式识别、图像拼接等算法都需设计良好的特征作为算法基础；在基于深度学习的机器视觉中，特征设计这一步骤被取代为计算机通过迭代寻找最佳特征，这些特征往往是多层次的，有些甚至是人无法理解的。但总之，从图像到特征到语义是机器视觉的基本步骤。因此需要寻找唯一性强、容易检测、易于比较的特征来表示图像。特征工程的两大问题：特征检测、特征表示。

### Harris角点检测

**Harris角点检测**。Harris角点具备平移不变性、旋转不变性，但不具备尺度不变性。

```py
dst = cv.cornerHarris(img, blockSize, ksize, k)
```

* img 原图像，float32灰度图
* blockSize 角点检测区域大小
* ksize Sobel算子卷积核大小，常用3
* k Harris算法分数公式中的常量，常取0.04~0.06，值越大，检测到的角点越少

例子：

```py
import numpy as np
import cv2 as cv
filename = 'chessboard.png'
img = cv.imread(filename)
gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
gray = np.float32(gray)
dst = cv.cornerHarris(gray,2,3,0.04)
#result is dilated for marking the corners, not important
dst = cv.dilate(dst,None)
# Threshold for an optimal value, it may vary depending on the image.
img[dst>0.01*dst.max()]=[0,0,255]
cv.imshow('dst',img)
if cv.waitKey(0) & 0xff == 27:
    cv.destroyAllWindows()
```

> `0.01*dst.max()`是Harris角点检测的结果处理方法

**亚像素精度角点检测**。首先寻找Harris角点，然后迭代。

```py
import numpy as np
import cv2 as cv
filename = 'chessboard2.jpg'
img = cv.imread(filename)
gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
# find Harris corners
gray = np.float32(gray)
dst = cv.cornerHarris(gray,2,3,0.04)
dst = cv.dilate(dst,None)
ret, dst = cv.threshold(dst,0.01*dst.max(),255,0)
dst = np.uint8(dst)
# find centroids
ret, labels, stats, centroids = cv.connectedComponentsWithStats(dst)
# define the criteria to stop and refine the corners
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)
corners = cv.cornerSubPix(gray,np.float32(centroids),(5,5),(-1,-1),criteria)
# Now draw them
res = np.hstack((centroids,corners))
res = np.int0(res)
img[res[:,1],res[:,0]]=[0,0,255]
img[res[:,3],res[:,2]] = [0,255,0]
cv.imwrite('subpixel5.png',img)
```

> 不是很明白有啥用，可能能提高精度

> Shi-Tomasi 角点检测器和良好的跟踪功能？

### SIFT尺度不变特征变换

SIFT特征具备平移不变性、旋转不变性、尺度不变性。

> 太复杂了，直接用吧。

```py
#!/usr/bin/python3
import numpy as np
import cv2 as cv

img = cv.imread('image/cat_1.jpg')
gray= cv.cvtColor(img,cv.COLOR_BGR2GRAY)
sift = cv.SIFT_create()
kp = sift.detect(gray,None)
des = sift.compute(gray,kp)
cv.drawKeypoints(gray,kp,img,flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv.imshow("test", img)
cv.waitKey(0)
```

例程中sift为SIFT检测对象。调用检测函数`detect()`，返回关键点列表，关键点是特殊的数据结构，包括位置、角度、强度等。
调用计算函数`compute()`计算特征描述子，8*16=128维向量。

> SURF（加速稳健特征）简介？

> 角点检测的FAST算法？

> BRIEF（二进制鲁棒独立基本特征）？

> ORB（定向快速旋转 BRIEF）？

### 特征匹配

> 特征匹配+单应性查找对象？

## 6. Video analysis (video module)

> 均值转换和凸轮转换？

### 光流法

光流法实例

```py
#!/usr/bin/python3
import numpy as np
import cv2 as cv
import argparse

cap = cv.VideoCapture(0)
# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

# Create some random colors
color = np.random.randint(0, 255, (100, 3))

# Take first frame and find corners in it
ret, old_frame = cap.read()
old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)

# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)
while(1):
    ret, frame = cap.read()
    if not ret:
        print('No frames grabbed!')
        break
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # calculate optical flow
    p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    # Select good points
    if p1 is not None:
        good_new = p1[st==1]
        good_old = p0[st==1]
    # draw the tracks
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
        frame = cv.circle(frame, (int(a), int(b)), 5, color[i].tolist(), -1)
    img = cv.add(frame, mask)
    cv.imshow('frame', img)
    k = cv.waitKey(30) & 0xff
    if k == 27:
        break
    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    p0 = good_new.reshape(-1, 1, 2)
cv.destroyAllWindows()
```

## 7. Camera Calibration and 3D Reconstruction

### 相机校准

> 姿态估计
> 对极几何？
> 来自立体图像的深度图？

## 8. Machine Learning

> K邻近算法
> 支持向量机
### K均值聚类

## 9. Computational Photography

> 图像去噪
> 图像修复
> 高动态范围 (HDR)

## 10. Object Detection (objdetect module)

### 级联检测器

级联检测器人脸检测实例

```py
#!/usr/bin/python3
import cv2 as cv
import argparse

def detectAndDisplay(frame):
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame_gray = cv.equalizeHist(frame_gray)
    #-- Detect faces
    faces = face_cascade.detectMultiScale(frame_gray)
    for (x,y,w,h) in faces:
        center = (x + w//2, y + h//2)
        # frame = cv.ellipse(frame, center, (w//2, h//2), 0, 0, 360, (255, 0, 255), 4)
        frame = cv.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 4)
        faceROI = frame_gray[y:y+h,x:x+w]
        #-- In each face, detect eyes
        eyes = eyes_cascade.detectMultiScale(faceROI)
        for (x2,y2,w2,h2) in eyes:
            eye_center = (x + x2 + w2//2, y + y2 + h2//2)
            radius = int(round((w2 + h2)*0.25))
            frame = cv.circle(frame, eye_center, radius, (255, 0, 0 ), 4)
    cv.imshow('Capture - Face detection', frame)

face_cascade = cv.CascadeClassifier()
eyes_cascade = cv.CascadeClassifier()
#-- 1. Load the cascades
# print(face_cascade_name)
# if not face_cascade.load(cv.samples.findFile(face_cascade_name)):
if not face_cascade.load('/home/dknt/opencv-4.6.0/data/haarcascades/haarcascade_frontalface_alt.xml'):
    print('--(!)Error loading face cascade')
    exit(0)
if not eyes_cascade.load('/home/dknt/opencv-4.6.0/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml'):
    print('--(!)Error loading eyes cascade')
    exit(0)

#-- 2. Read the video stream
cap = cv.VideoCapture(0)
if not cap.isOpened:
    print('--(!)Error opening video capture')
    exit(0)
while True:
    ret, frame = cap.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break
    detectAndDisplay(frame)
    if cv.waitKey(1) == ord('q'):
        break
```

> 级联检测器训练

## 11. OpenCV-Python Bindings

> 解释OpenCV-Python的工作原理



---
参考链接
1.https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html
2.https://github.com/opencv/opencv/tree/4.x/doc/py_tutorials
