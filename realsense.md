# Realsense SDK

> 设备：D435i

2023 年 6 月，librealsense 还没有支持 Ubnutu 22，需要进行源码编译。

下载源码，使用 cmake 时打开选项 `FORCE_RSUSB_BACKEND=true`。

1

```cpp
rs2::context;
rs2::device_list;
rs2::device;
rs2::pipeline;  // 管道
rs2::frameset;
rs2::profile;
rs2::stream;
rs2::stream_profile;
rs2::align;
rs2_intrinsics;
```

# 获取图像

1

# OpenCV

1

# PCL

1

# 固件配置





# 标定与调参


## 参数查看

使用命令 `rs-sensor-control` 在命令行中查看与设置相机参数

## 使用 kalibr 标定相机参数

> 参考: https://blog.csdn.net/weixin_41954990/article/details/127928852



