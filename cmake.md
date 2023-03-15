# cmake常用命令

> 文档记录了我遇到过的cmake命令。
> 
> 能用就行，不求甚解。
> 
> dknt
> 
> 2023.2

## 1 编译C++程序

cmake按照CMakeLists.txt对项目进行编译。

一个CMakeLists.txt中至少包含如下语句：

```cmake
# 声明cmake最低版本
cmake_minimum_required( VERSION 2.8 )
# 声明cmake工程
project( HelloWorld )
# 添加可执行程序 程序名 源文件路径
add_executable( test test.cpp )
```

执行后，生成名为test的可执行文件。

## 2 编译库

编译**静态链接库**，并将可执行文件链接到库：

```cmake
# 编译静态链接库
add_library( hello lib/lib_hello.cpp )
# 将可执行文件链接到库
target_link_libraries( test hello )
```

编译后生成`hello.a`库文件。

编译**动态链接库**，并将可执行文件链接到库：

```cmake
# 编译动态链接库（共享库）
add_library( hello_shared SHARED lib/lib_hello.cpp )
# 将可执行文件链接到库
target_link_libraries( test hello_shared )
```

编译后生成`hello.so`库文件。

## 3 头文件路径

添加头文件路径：

```cmake
# 添加Eigen库路径
include_directories( "/usr/include/eigen3" )
```

#

常用库的添加方式：

```cmake
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(target ${OpenCV_LIBS})

# Eigen
include_directories("/usr/include/eigen3")

# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
target_link_libraries(target ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(target ${CERES_LIBRARIES})
```

# 源码编译软件时技巧

建立build目录，`cmake ..`之后，可以通过`cmake . -L`来查看配置信息与全部可能的选项
