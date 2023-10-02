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


