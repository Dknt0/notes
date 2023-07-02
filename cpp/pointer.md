# C++ 智能指针

> 参考：
> 
> https://zhuanlan.zhihu.com/p/526147194
> 
> Dknt 2023.7

C++ 中的智能指针帮助程序员管理动态分配的内存，避免**内存泄漏**。

内存泄漏：堆区的内存使用后没有被释放。

内存泄漏只会发生在指针指向堆区内存时，而栈区的对象会在生命周期结束时自动被释放。因此，我们可以利用对象的这种特性，在其析构函数中加入`delete`，实现对堆区内存的自动管理。

智能指针有以下几种：

C++ 98: `auto_ptr`

C++ 11: `unique_ptr`, `shared_ptr`, `weak_ptr`

# 1. auto_ptr

> C++ 11 中用`unique_ptr`代替了`auto_ptr`，不建议再使用`auto_ptr`。

C++ 98 中的智能指针模板，定义了管理指针的对象，可以将`new`获得的地赋给`auto_ptr`。当对象过期时，其析构函数会使用`delete`来释放内存。

特性：

* 基于排他所有权模式。两个指针不能指向同一个资源，复制或赋值都会改变资源的所有权

例子如下：

```cpp
#include <memory>
// 实例化对象
std::auto_ptr<std::string> str(new std::string("Hello pointer!"));
```

`auto_ptr`常用的三个函数如下：

1. get()

用于获取智能指针托管的指针地址

```cpp
std::auto_ptr<Test> test(new Test);
// 获取地址
Test* tmp = test.get();
```

2. release()

取消智能指针对动态内存的托管，改为程序员管理

```cpp
std::auto_ptr<Test> test(new Test);
// 取消智能指针对动态内存的托管
Test *tmp2 = test.release();
delete tmp2; // 之前分配的内存需要自己手动释放
```

3. reset()

重置智能指针托管的内存地址，如果地址不一致，原来的会被析构掉

```cpp
auto_ptr<Test> test(new Test);
// 释放掉智能指针托管的指针内存，并将其置NULL
test.reset();
// 释放掉智能指针托管的指针内存，并将参数指针取代之
test.reset(new Test());
```

使用注意：

* 别整`auto_ptr`的`auto_ptr`

* `auto_ptr`一般不会定义为全局变量

* 不要进行`auto_ptr`之间的赋值

局限：

* 复制或者赋值都会改变资源的所有权

        如`p1 = p2`，会释放`p1`指向的内存，同时`p2`会指向`NULL`

* 在STL容器中使用`auto_ptr`存在着重大风险，因为容器内的元素必须支持可复制和可赋值

        不能使用`vector<auto_ptr>`这样的容器

* 不支持对象数组的内存管理

# 2. unique_ptr 独占指针

为了解决`auto_ptr`的问题，C++ 11 中引入了`unique_ptr`。

特性：

* 基于排他所有权模式：两个指针不能指向同一个资源

* 无法进行左值unique_ptr复制构造，也无法进行左值复制赋值操作，但允许临时右值赋值构造和赋值

* 在容器中保存指针是安全的

右值构造和赋值的例子如下：

```cpp
unique_ptr<string> p1(new string("I'm Li Ming!"));
unique_ptr<string> p2(new string("I'm age 22."));
// 使用 move 把左值转成右值
unique_ptr<string> p3(std::move(p1));
p1 = std::move(p2);
```

赋值的结果同`auto_ptr`，`p1`释放原内存，`p2`指向`NULL`

容器中保存指针的例子如下：

```cpp
// 会自动调用delete [] 函数去释放内存
unique_ptr<int[]> array(new int[5]); // 支持这样定义
```

类似于`auto_ptr`，存在下面三种成员函数：

```cpp
unique_ptr<Test> test(new Test);

test.get(); // 获取地址
test.release(); // 放弃对对象的控制权
test.reset(); // 重置
```

由于排他性，`auto_ptr`与`unique_ptr`智能指针存在内存管理陷阱

```cpp
auto_ptr<string> p1;
string *str = new string("智能指针的内存管理陷阱");
p1.reset(str);    // p1托管str指针
{
    auto_ptr<string> p2;
    p2.reset(str);    // p2接管str指针时，会先取消p1的托管，然后再对str的托管
}

// 此时p1已经没有托管内容指针了，为NULL，在使用它就会内存报错！
cout << "str：" << *p1 << endl;
```

# 3. shared_ptr 共享指针

`shared_ptr`用于解决`unique_ptr`排他性带来的局限性。

`shared_ptr`利用静态成员记录引用特定内存对象的智能指针数量，当复制或拷贝时，引用计数加1，当智能指针析构时，引用计数减1，如果计数为零则释放这片内存。

构造方法：

```cpp
#include <memory>
// 1. 构造空指针
shared_ptr<Person> sp1;

// 2. 构造指针，同时指向对象
shared_ptr<Person> sp2(new Person(2)); // 申请内存
shared_ptr<Person> sp3(sp1); // 两指针指向同一个对象

// 3. 构造指向数组的空指针 C++17
shared_ptr<Person[]> sp4;

// 4. 构造指向数组的指针 C++17
shared_ptr<Person[]> sp5(new Person[5] { 3, 4, 5, 6, 7 });

// 5. 构造空指针，接受一个删除器
shared_ptr<Person> sp6(NULL, DestructPerson());

// 6. 构造指针，接受一个删除器
shared_ptr<Person> sp7(new Person(8), DestructPerson());
```

初始化：

```cpp
// 1. 构造函数
shared_ptr<int> up1(new int(10));  // int(10) 的引用计数为1

// 2. 使用 make_shared()
shared_ptr<Person> up5 = make_shared<Person>(9);
```

使用`make_shared`初始化对象，分配内存效率更高，推荐使用这种方法。

常用函数：

```cpp
shared_ptr<Person> sp1(new Person());
// 获取引用次数
sp1.use_count();
// 重置
sp1.reset(); // 重置为空
sp1.reset(sp2);
// 交换
std::swap(p1,p2); // 交换p1 和p2 管理的对象，原对象的引用计数不变
p1.swap(p2); // 交换p1 和p2 管理的对象，原对象的引用计数不变
```

`shared_ptr`使用陷阱。如果两个对象互相持有对方的`shared_ptr`，会导致这两个对象都无法被自动释放。因此要避免互相持有对方`shared_ptr`的情况，或使用`weak_ptr`。

# 4. weak_ptr 弱指针

`weak_ptr`是用来协助`shared_ptr`工作的智能指针，它只可以从一个`shared_ptr`或另一个`weak_ptr`对象构造, 它的构造和析构不会引起引用记数的增加或减少。

我们不能直接对`weak_ptr`解引用，当需要访问对象时，需要使用`lock`函数临时将`weak_ptr`转为`shared_ptr`使用。

```cpp
shared_ptr<Boy> spBoy(new Boy());
shared_ptr<Girl> spGirl(new Girl());

// 弱指针的使用
weak_ptr<Girl> wpGirl_1; // 定义空的弱指针
weak_ptr<Girl> wpGirl_2(spGirl); // 使用共享指针构造
wpGirl_1 = spGirl; // 允许共享指针赋值给弱指针
```

弱指针转换为共享指针：

```cpp
// 在必要的使用可以转换成共享指针
shared_ptr<Girl> sp_girl;
sp_girl = wpGirl_1.lock();
// 使用之后需要将共享指针置空
sp_girl = NULL;
```

弱指针也可以用于获取引用计数：

```cpp
wpGirl_1.use_count()
```

使用`expired`判断弱指针是否还有托管对象。如果有返回`false`，无返回`true`。

```cpp
wpGirl_1.expired()
```
