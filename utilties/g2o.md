# g2o 学习记录

> g2o 专用于图优化的非线性最小二乘库，相比 Ceres，应用场景更少，但在 SLAM 问题中似乎能提供更快的速度。g2o 支持 float 型浮点数，而 Ceres 只支持 double。
> 
> ORB-SLAM 中使用了 g2o。
> 
> 版本号 20230806
> 
> Dknt 2023.12

# 0 基础

CMake 中使用 g2o

```cmake
find_package(G2O REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${G2O_INCLUDE_DIRS}
)
target_link_libraries(target_name
    ${G2O_LIBRARY}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
)
```

# 1 基本使用

g2o 中的图优化问题由顶点和边组成。顶点代表一组参数块，即待优化变量，这组变量可能属于欧式空间，也可能属于某个流形，流形上的加法需要由用户自定义。边代表残差项，即观测，用户需要给出误差的计算公式，g2o 中的边分为一元边、二元边、多元边。

> g2o 对用户的的接口不像 Ceres 那么友好。残差、雅可比等重要参数是以成员变量形式存在的，而 Ceres 都写在了参数列表中。

1

> g2o 提供了类似于 Ceres 的自动求导

# 2 图优化顶点

顶点中重要的成员变量 _estimate

1

顶点的单独使用，获取当前估计值

将顶点设置为固定

开启顶点边缘化

# 3 图优化边

1

边的单独使用，计算当前误差值

# 4 求解器配置

1
