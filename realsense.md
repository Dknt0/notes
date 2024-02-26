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

# 标定与调参

1
