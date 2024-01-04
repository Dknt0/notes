# C++ 多线程

> 参考:
> 
> [菜鸟教程C++多线程](https://www.runoob.com/cplusplus/cpp-multithreading.html)
> 
> [Introduction · C++并发编程](http://shouce.jb51.net/cpp_concurrency_in_action/)

# 1 Linux pthread 库

Linux下使用POSIX编写多线程程序。

需要包含头文件`#include <pthread.h>`

**创建线程**`pthread_create`

```cpp
pthread_create(thread, attr, start_routine, arg);
```

* `thread`: 指向线程标识符指针。

* `attr`: 不透明的属性对象，可以被用来设置线程属性。

* `start_routine`: 线程运行函数起始地址，一旦线程被创建就会执行。函数返回值为空指针类型，参数也为一个空指针类型。

* `arg`: 运行函数的参数。它必须通过把引用作为指针强制转换为 void 类型进行传递。如果没有传递参数，则使用 NULL。

**终止线程**`pthread_exit`

用于显式地结束一个线程

```cpp
pthread_exit(status)
```

示例如下：

```cpp
#include <iostream>
#include <pthread.h>
#include <unistd.h>

using namespace std;

void* pubTest1(void* args){
    cout << 1.111111 << endl;
    return nullptr;
}

void* pubTest2(void* args){
    cout << "pub test." << endl;
    cout.precision(2);
    return nullptr;
}

int main(){
    pthread_t id1, id2;

    int ret1 = pthread_create(&id1, NULL, pubTest1, NULL);
    usleep(1000);
    int ret2 = pthread_create(&id2, NULL, pubTest2, NULL);
    usleep(1000);
    pthread_create(&id1, NULL, pubTest1, NULL);

    if(ret1 != 0 && ret2 != 0){
        cerr << "线程创建失败" << endl;
    }

    pthread_exit(NULL);
    return 0;
}
```

> 多线程编程要考虑线程之间的同步和互斥。
> 使用std::cout并不合适，因为它全局只存在一个。

**连接和分离线程**

```cpp
pthread_join(threadid, status) 
pthread_detach(threadid) 
```

# 2 C++ 11 多线程

## 2.1 thread 多线程

thread 是 C++ 11 中引入的一个与平台无关的线程库。

可以向线程对象传入函数指针、仿函数、匿名函数。基本使用如下：

```cpp
#include <thread>
// 以函数指针为例
void func(int a)
{
    std::cout << "hello " << std::this_thread::get_id() << std::endl;
    std::cout << a << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "world " << std::this_thread::get_id() << std::endl;
}
// 开线程，第一个参数是函数指针，第二个是参数
std::thread t1(func, 1);
// 等待线程执行结束
t1.join();
```

> 参数必须是右值？

promise future 用于实现线程异步并发

> 类成员函数作为线程函数使用时，需要传入指向对象的指针，可以是`this`

## 2.2 mutex 互斥锁

实现临界资源互斥访问。示例如下：

```cpp
#include <mutex>
// 创建锁
std::mutex m1;
// 上锁
m1.lock();
// 解锁
m1.unlock();
```

## 2.3 promise & future 异步并发

实例如下：

```cpp
#include <future>
// 创建 promise
std::promise<bool> prom = std::promise<bool>();
std::future<bool> future = prom.get_future();

// 等待 future
if (future.wait_for(seconds(3)) == std::future_status::timeout) {
    cerr << "No autopilot found" << endl;
    return;
}

// 阻塞，等待 future
future.get();
```

<mark>注意</mark>：C++ 类内非静态成员函数不能作为线程函数使用，会报错。解决方案主要有两种：

1. 使用匿名函数，引用传参，这样可以在匿名函数中访问类成员变量

2. 编写如下静态成员函数，传入 this 指针。尽量传递指针，报错少。

```cpp
// 这个例子是 pthread 库的，thread 库返回值为 void
void * Test::insert_pth(void* __this)
{
    Test * _this =(Test *)__this;
    sleep(2);
    _this->sum+=1;
    printf("%d insert.....\n",_this->sum);
}
```

一种非阻塞式等待`future`的方法：

```shell
wait_for(...);
```

## 2.3 unique_lock 独占锁

独占锁用于管理线程锁`std::mutex`对象，在创建`std::unique_lock`时为其传入一个`std::mutex`对象，独占锁会阻塞等待锁被解开，并上锁执行后面的程序，当独占锁生命周期间结束时，会自动释放线程锁。

示例如下：

```cpp
std::mutex myMutex;
{
    std::unique_lock<std::mutex> lck(myMutex);
    // do sth
}
// lck 生命周期结束，myMutex 被释放
```