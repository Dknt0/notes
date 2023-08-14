# C++ 流对象

> 参考：
> 
> https://blog.csdn.net/d7986/article/details/79373482
> 
> Dknt 2023.7

# _ 流控制符

用于格式化输出。

包含头文件:

```cpp
#include <iomanip>
using namespace std;
```

常用控制符如下

| 控   制   符                    | 作           用                 |
| ---------------------------- | ----------------------------- |
| dec                          | 设置整数为十进制                      |
| hex                          | 设置整数为十六进制                     |
| oct                          | 设置整数为八进制                      |
| setbase(n)                   | 设置整数为n进制(n=8,10,16)           |
| setfill(n)                   | 设置字符填充，c可以是字符常量或字符变量          |
| setprecision(n)              | 设置浮点数的有效数字为n位                 |
| setw(n)                      | 设置字段宽度为n位                     |
| setiosflags(ios::fixed)      | 设置浮点数以固定的小数位数显示               |
| setiosflags(ios::scientific) | 设置浮点数以科学计数法表示                 |
| setiosflags(ios::left)       | 输出左对齐                         |
| setiosflags(ios::right)      | 输出右对齐                         |
| setiosflags(ios::skipws)     | 忽略前导空格                        |
| setiosflags(ios::uppercase)  | 在以科学计数法输出E与十六进制输出X以大写输出，否则小写。 |
| setiosflags(ios::showpos)    | 输出正数时显示"+"号                   |
| setiosflags(ios::showpoint)  | 强制显示小数点                       |
| resetiosflags()              | 终止已经设置的输出格式状态，在括号中应指定内容       |
