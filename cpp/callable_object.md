# C++ 可调用对象

> 参考：
> 
> https://blog.csdn.net/qq_38410730/article/details/103637778
> 
> [C++ lambda表达式详细讲解2-隐式捕获与显式捕获](https://blog.csdn.net/readyone/article/details/110948770)
> 
> Dknt 2023.6

可调用对象主要用于回调函数设计中。C++11 提供了`std::function`和`std::bind`两个方法来对可回调对象进行统一和封装。

C++ 中主要有以下几种可调用对象：函数、函数指针、匿名函数、bind 对象、仿函数。

前二者继承自 C，类型不安全。

# 1. 匿名函数

匿名函数也叫 Lambda 表达式。由于lambda是匿名的，所以保证了其不会被不安全的访问。

```cpp
auto f = [&](int a, int b) -> int {
    return a + b;
}; // 定义 Lambda 表达式  引用捕获全部外部变量  显式给出返回值类型
f(1, 2);
```

匿名函数的闭包，使匿名函数可以用值或引用的方式捕获外部变量，例如：

```cpp
[]        // 未定义变量.试图在Lambda内使用任何外部变量都是错误的.
[x, &y]   // x 按值捕获, y 按引用捕获.
[&]       // 用到的任何外部变量都隐式按引用捕获
[=]       // 用到的任何外部变量都隐式按值捕获
[&, x]    // x显式地按值捕获. 其它变量按引用捕获
[=, &z]   // z按引用捕获. 其它变量按值捕获
```

# 2. std::function

`std::function`是一个可调用对象包装器，是一个类模板，它可以用统一的方式处理函数、函数对象、函数指针、匿名函数，并允许保存和延迟它们的执行。它比普通函数指针更加的灵活和便利，是对现有可调用对象的一种类型安全的包裹。

> `std::function`不能容纳类成员函数指针，因为不包含`this`。这个问题可以通过`std::bind`解决。

例如：

```cpp
// std::function 包含在这个头文件下，我称之为泛函
#include <functional>

typedef std::functional<int(int, int)> comfun;

// 普通函数
int add(int a, int b) { return a + b; }

// 匿名函数
auto mod = [](int a, int b) { return a % b; };

// 仿函数
class divide {
private:
    int operator()(int a, int b) { return a / b; }
};

// 转化为 std::function 对象后，以相同的方式保存、执行
comfun a = add;
comfun b = mod;
comfun c = divide();
a(1, 2); b(1, 2); c(1, 2);
```

# 3. std::bind

通用的函数适配器，接受一个可调用对象，生成一个新的可调用对象来适应原对象的参数列表。调用`bind`返回一个`std::function`对象。

**推荐使用这种方式生成回调函数。**

`std::bind`可以使用`std::placeholders::_1`等占位符，在调用新对象时传入参数。占位符对应参数通过引用传递方式传入。

`bind`绑定类成员函数时，第一个参数表示对象的成员函数的指针，第二个参数表示对象的地址，这是因为对象的成员函数需要有`this`指针。并且编译器不会将对象的成员函数隐式转换成函数指针，需要通过&手动转换。

例子如下：

```cpp
#include <functional>

// 类，不是仿函数
class A {
public:
    void fun_3(int k,int m) {
        std::cout << "print: k = "<< k << ", m = " << m << std::endl;
    }
};

// 函数，参数值传递
void fun_1(int x, int y, int z) {
    std::cout << "print: x = " << x << ", y = " << y << ", z = " << z << std::endl;
}

// 函数，参数引用传递，注意不能传入常量
void fun_2(int &a, int &b) {
    ++a;
    ++b;
    std::cout << "print: a = " << a << ", b = " << b << std::endl;
}

int main(){
    auto f1 = std::bind(fun_1, 1, 2, 3);
    f1(); // 输出 1 2 3

    auto f2 = std::bind(fun_1, std::placeholders::_1, std::placeholders::_2, 3);
    f2(2, 3); // 输出 2 3 3

    auto f2 = std::bind(fun_1, std::placeholders::_2, std::placeholders::_1, 3);
    f2(2, 3); // 输出 3 2 3

    int m = 2;
    int n = 3;
    auto f4 = std::bind(fun_2, std::placeholders::_1, n); // 表示绑定fun_2的第一个参数为n, fun_2的第二个参数由调用f4的第一个参数（_1）指定。
    f4(m); // print: a=3,b=4
    std::cout << "m = " << m << std::endl; // m=3  说明：bind对于不事先绑定的参数，通过std::placeholders传递的参数是通过引用传递的,如m
    std::cout << "n = " << n << std::endl; // n=3  说明：bind对于预先绑定的函数参数是通过值传递的，如n

    A a;
    auto f5 = std::bind(&A::fun_3, &a, std::placeholders::_1, std::placeholders::_2); //使用auto关键字
    f5(10, 20);    //调用a.fun_3(10,20),print: k=10,m=20
}
```
