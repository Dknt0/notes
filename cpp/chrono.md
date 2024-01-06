# C++ Chrono

`Chrono` 是 boost 库的一部分，提供时间管理功能，包括多线程下的定时、计时、延时等。

延时：

```cpp
#include <chrono>
#include <thread>

// 延时 1.5 秒
std::this_thread::sleep_for(std::chrono::seconds(1.5));

// 等同于如下写法
using namespace chrono_literals;
std::this_thread::sleep_for(1.5s);
```

计时

```cpp
#include <chrono>

std::chrono::time_point<std::chrono::steady_clock> t1, t2;  // 时间点对象
std::chrono::duration<double> time_used;  // 时间间隔对象

// 记录时间点
t1 = std::chrono::steady_clock::now();
...
t2 = std::chrono::steady_clock::now();
// 计算时间差
time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

std::cout << "Time used: " << time_used.count() << std::endl;
```

获取当前时间

```cpp
#include <chrono>
// 用当前时间设置随机数种子
srand(std::chrono::steady_clock::now().time_since_epoch().count());
// 获取 0~RAND_MAX 的 int32 随机数
int r = rand();
```
