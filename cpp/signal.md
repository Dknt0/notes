# C++ 信号处理

信号是 OS 传递给进程的中断信号，默认用于终止程序。C++ 预定义的信号宏及其意义如下

| 信号      | 描述                     |
| ------- | ---------------------- |
| SIGABRT | 程序的异常终止，如调用 **abort**。 |
| SIGFPE  | 错误的算术运算，比如除以零或导致溢出的操作。 |
| SIGILL  | 检测非法指令。                |
| SIGINT  | 程序终止(interrupt)信号。     |
| SIGSEGV | 非法访问内存。                |
| SIGTERM | 发送到程序的终止请求。            |

我们可以通过调用 `std::signal(signal, handler)` 为信号绑定其他的处理函数，通过`std::raise(signal)`主动发出信号。

信号中只有 0~31 有效，7 和 29 没有预定义，可以当作用户内中断使用。0 为用户保留，调用 kill(pid, 0)

例程如下：

```cpp
#include <csignal>
#include <iostream>
// 7 或 29 没有预定义，可以直接用，31 以上无效
#define USER_SIG 29
void signal_handle_1(int signal_number) {
  std::cout << "SIGINT received." << std::endl;
  std::cout << "Raise signal " + USER_SIG << std::endl;
  std::raise(USER_SIG);
}
void signal_handle_2(int signal_number) {
  std::cout << "Signal " << signal_number << " received." << std::endl;
  exit(0);
}
int main() {
  // 程序终止 interrupt 信号
  std::signal(SIGINT, signal_handle_1);
  // 自定义信号
  std::signal(USER_SIG, signal_handle_2);
  std::cout << "Waiting for singal..." << std::endl;
  while (true) {
    sleep(1);
  }
  return 0;
}
```
