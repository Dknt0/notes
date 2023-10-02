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

# _ 编译技巧

* `cmake_gui`参数配置

建立build目录，`cmake ..`之后，可以通过`cmake . -L`来查看配置信息与全部可能的选项

直接 `cmake-gui ..` 更香

* 指定路径下寻找库

不同软件可能会依赖于同一库的不同版本。如 OpenCV，我们常需要在计算机上编译几个不同版本的 OpenCV，但只能安装其中一个到默认路径下。我们可以将其他的版本编译后安装在其他位置，也可以不安装，保留在 build 目录下，在项目 CMakeLists.txt 中添加即可。这样 cmake 会在指定路径下搜索指定包的`FindXXX.cmake`或`XXXConfig.cmake`。

如：

```cmake
set(OpenCV_DIR "/home/dknt/Software/opencv/opencv/build_460")
find_package(OpenCV REQUIRED)

message("OpenCV Version: ")
MESSAGE(${OpenCV_VERSION})
```

有些库还需要安装 package-config 文件，默认在`/usr/lib/pkgconfig`下。需要自定义路径否则会与系统路径下的文件冲突。在编写 CMakeLists.txt 时，需要修改 config 文件默认搜索路径。

```shell
export PXG_CONFIG_PATH=PXG_CONFIG_PATH:${}
```

有问题！
