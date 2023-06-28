# OpenGL

> 我也不知道为什么学这个，但还是学一下吧。
> 
> Dknt 2023.6
> 
> 参考：
> 
> https://learnopengl.com/

# 1 基础

OpenGL 是一个计算机图形学 API。确切地说，他不是 API，而是由 [Khronos Group](http://www.khronos.org/) 开发的一套标准。不同的显卡厂商根据这一套标准开发了适用于自己硬件的 OpenGL 库，通常包含在显卡驱动中。

> 也就是说，OpenGL 是需要显卡支持的。但也有用 CPU 实现的 OpenGL 库——[Mesa](https://github.com/Mesa3D/mesa)。
> 
> 通常，OpenGL 出现错误时，可以通过更新显卡驱动解决。

有两种 OpenGL 编程方法：核心模式（Core-profile）和立即渲染模式（Immediate mode）。OpenGL 3.2 之后开始支持核心模式。核心模式更自由，更高效，但难度也更大，需要使用者对图形学编程有更深入地了解。教程讲述核心模式编程。

OpenGL 是一个有很多参数的状态机，状态通常指上下文（Context）。使用 OpenGL 时，我们经常需要使用一些状态改变函数（state-changing）来改变上下文、一些状态使用函数（state-using）来根据当前上下文去执行某操作。

OpenGL 底层是用 C 语言写成的。由于 C 语言中结构不能很好地转换为其他高级语言中的类，开发者对 OpenGL 进行了一些抽象。其中之一就是 OpenGL 对象。OpenGL 对象是一些选项的集合，是 OpenGL 状态的一个子集。

## 1.1 窗口创建

使用 OpenGL 的第一步是创建**上下文**和**窗口**，具体步骤因操作系统而异。存在一系列工具程序库，如 GLUT, SDL, SFML 和 GLFW。教程使用 GLFW（Graphics Library Framework）。
