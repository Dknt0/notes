# Linux下C++多线程

> 参考: [菜鸟教程C++多线程](https://www.runoob.com/cplusplus/cpp-multithreading.html)

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