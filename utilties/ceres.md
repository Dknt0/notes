# Ceres 学习记录

> Google Ceres 非线性最小二乘优化库，十分的强大，十分用的到。
> 
> 相比 g2o，Ceres 应用范围更广，所以先看这个吧。
> 
> See: http://ceres-solver.org/
> 
> 我使用的是从 Ceres 2.0.0 源码 doc 中编译得到的文档。不同版本 Ceres 之间存在较大区别。
> 
> Dknt 2023.11

# 0 基础

Ceres 主要用于解决约束条件下的多元非线性最小二乘问题：

$$
\begin{split}\min_{\mathbf{x}} &\quad \frac{1}{2}\sum_{i} \rho_i\left(\left\|f_i\left(x_{i_1}, ... ,x_{i_k}\right)\right\|^2\right) \\
\text{s.t.} &\quad l_j \le x_j \le u_j\end{split}
$$

Ceres 也可以解决一般的无约束优化问题，但这不是我们使用他的主要目的。

Cmake 中使用：

```cmake
find_package(Ceres REQUIRED)
include_directories(
    ${Ceres_INDLUDES}
)
target_link_libraries(target_name
    ${CERES_LIBRARIES}
)
```

# 1 基本使用

示例：

```cpp
#include <ceres/ceres.h>
// 残差仿函数
class CostFunctor {
public:
    template <typename T>
    bool operator ()(const T* const x, T* residual) const {
        residual[0] = (10.0 - x[0]) * (-10.0 - x[0]);
        return true;
    }
};
int main(int argc, char **argv) {
    // 日志
    google::InitGoogleLogging(argv[0]);
    // 优化变量与其初值
    double initial_x = 0.001;
    double x = initial_x;
    // 损失函数
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    // 优化问题
    ceres::Problem problem;
    problem.AddResidualBlock(cost_function, nullptr, &x);
    // 求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    // 求解问题
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "Initial value: " << initial_x << " result: " << x << std::endl;
    return 0;
}
```

TODO 问题求解流程 添加残差块、参数块，配置问题，求解问题

# 3 残差块

通过 AddResidual 向优化问题添加残差块

## 3.1 残差导数

优化问题的求解需要求残差函数的导数，Ceres 提供三种求导方式：

- `Automatic Differentiation`——自动求导

- `Analytic`——解析导数

- `Numeric Derivatives`——数值导数

数值导数误差大、耗时多。官方推荐使用自动求导，Ceres 自动求导给予对偶数原理，利用了 C++ 泛型编程的性质。但如果希望代码最优化，还是要推倒解析导数，并且运用一些编程技巧。如果优化变量位于流形上，而不是欧式空间，我们将不得不推到解析导数，并配合 LocalParameterization (Ceres 2.1 后为 Manifold)使用。

### 3.1.1 自动求导

TODO 原理, 对偶数

TODO 数值导数、解析导数向自动导数的转换

```cpp
class AutoCostFunctor {
public:
    template <typename T>
    bool operator ()(const T* const x, T* residual) const {
        residual[0] = (10.0 - x[0]);
        return true;
    }
};
ceres::CostFunction *cost_function1 =
    new ceres::AutoDiffCostFunction<AutoCostFunctor, 1, 1>(
        new AutoCostFunctor);
```

### 3.1.2 解析导数

使用解析导数时我们需要直接继承 CostFuntion 类，见第 3.1 节。解析导数并不属于残差仿函数。

```cpp
class QuadraticCostFunction : public ceres::SizedCostFunction<1, 1> {
public:
    QuadraticCostFunction() {
        std::cout << "Creating QuadraticCostFunction." << std::endl;
    }
    virtual ~QuadraticCostFunction() {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        const double x = parameters[0][0];
        // 计算残差
        residuals[0] = 10 - x;
        // 计算雅可比
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            jacobians[0][0] = -1;
        }
        return true;
    }
};
QuadraticCostFunction *cost_function3 = new QuadraticCostFunction;
```

### 3.1.3 数值导数

Ceres 提供的数值求导方法：前向差分、中点法、三次样条线求导。

```cpp
class NumericDiffCostFunctor {
public:
    bool operator ()(const double* const x, double *residual) const {
        residual[0] = (10.0 - x[0]);
        return true;
    }
};
ceres::CostFunction *cost_function2 =
    new ceres::NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 1>(
        new NumericDiffCostFunctor);
```

1

## 3.2 代价函数 CostFunction

用户编写残差类时需要继承自`ceres::CostFunciton`类。

```cpp
class ceres::CostFunction {
 public:
  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) = 0;
  const vector<int32>& parameter_block_sizes();
  int num_residuals() const;

 protected:
  vector<int32>* mutable_parameter_block_sizes();
  void set_num_residuals(int num_residuals);
};
```

残差类可以由残差仿函数按照自动求导或数值求导规则由类模板构造，也可以按照解析导数由用户手动编写。

我们在构造残差函数时，常继承自`ceres::SizedCostFunction`类，参数列表如下：

```cpp
ceres::SizedCostFunction<残差向量长度, 第一个参数块长度, 第二个参数块长度, ...>;
```

### 3.1.1 CostFunction::Evaluate

在残差类中，我们重点需要实现`Evaluate`函数。这个函数用于计算当前状态 (parameters) 下的误差 (residuals) 和雅可比 (jacobians)。

```cpp
bool Evaluate(double const* const* parameters,
              double* residuals,
              double** jacobians);
```

* `parameters` 为状态量，是一个常二维数组。第一层数组为指向参数块的指针数组，包含 parameter_block_sizes_.size() 个元素。第二层数组为参数块本身。

* `residuals` 为误差量，残差计算需要由用户实现。

* `jacobians` 为雅可比，是一个二维数组。第一层为指针数组，这些数组指向每个参数块对应雅可比。第二层数组为雅可比矩阵，以 **row-major** 的方式存放在数组中。用户在计算雅可比时需要注意以下几点：

* *  $\text{jacobians}[i][r * \text{parameter\_block\_sizes}[i] + c]  = \frac{\displaystyle \partial \text{residual}[r]}{\displaystyle \partial \text{parameters}[i][c]}$ 
  
  * jacobians 和 jacobians[i] 都有可能是 nullptr。**当传入空指针时，用户应忽略雅可比的计算**。例如：
  
  ```cpp
  ... // In function Evaluate()
  if (jacobians) {
      if (jacobians[0]) { // Compute the first jacobian }
      if (jacobians[1]) { // Compute the second jacobian }
      ...
  }
  return true;
  ```
  
  * **当使用 Manifold 时，通常在残差的 Evaluate 直接计算对切空间的雅可比**。因为根据扰动定义方式的不同，雅可比的计算很可能与除流形自身以外的其他变量有关（比如用旋转矩阵旋转一个向量的结果对旋转矩阵左扰动的导数，与被旋转向量的坐标也有关），这时我们无法在 Manifold::PlusJacobian 中计算雅可比。
  
  * Ceres 对矩阵的存储方式默认是 row-major 的，Eigen 默认为 column-major，当我们希望在 Ceres 中使用 Eigen 时，需要 row-major 的 Eigen 矩阵，例如：
  
  ```cpp
  double** jacobians;
  Eigen::Map<Eigen::Matrix<double, num_rols, num_cols, 
                           Eigen::RowMajor>> J_0(jacobians[0]);
  ... // We can use J_0 like a common Eigen matrix
  ```

## 3.3 核函数 LostFunction

_ 核函数

核函数用于去除外点。Ceres 提供如下核函数：

- 柯西核函数

1 ...

在向优化问题添加残差项时，可以选择添加核函数。如果不使用核函数，则传入`nullptr`

```cpp
// 使用柯西核函数
problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), &m, &c);
// 不使用
...
```

# 4 参数块

一个`double`数组可以作为参数块传入问题中，称为平凡的。当我们在流形上进行优化时，需要自己实现流形上的运算规则。Ceres 2.1 前后的流形类存在较大区别，2.1 本身

## 4.1 局部参数 LocalParameterization

> Ceres 2.0 之前



`ceres::LocalParameterization`基类

```cpp
class ceres::LocalParameterization {
 public:
  virtual ~LocalParameterization() {}
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const = 0;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const = 0;
  virtual bool MultiplyByJacobian(const double* x,
                                  const int num_rows,
                                  const double* global_matrix,
                                  double* local_matrix) const;
  virtual int GlobalSize() const = 0;
  virtual int LocalSize() const = 0;
};
```





**重要技巧**：在 LocalParameterization 的雅克比计算一般简单放置为单位阵与零，真正的雅克比计算通常实现在 CostFunction 的 Evaluate 中 。

> 参考
> 
> [GitHub - QiukuZ/ceres_R_LocalParam: ceres rotation matrix local parameterization](https://github.com/QiukuZ/ceres_R_LocalParam)



1

> Eigen::Map 使用技巧

## 4.2 流形 Manifold

> Ceres 2.2 之后

Ceres 2.1 之后，原 `ceres::Parameterization` 类变为了 `ceres::Manifold`，接口也有所改变。

```cpp
class ceres::Manifold {
 public:
  virtual ~Manifold();
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const = 0;
  virtual bool PlusJacobian(const double* x, double* jacobian) const = 0;
  virtual bool RightMultiplyByPlusJacobian(const double* x,
                                           const int num_rows,
                                           const double* ambient_matrix,
                                           double* tangent_matrix) const;
  virtual bool Minus(const double* y,
                     const double* x,
                     double* y_minus_x) const = 0;
  virtual bool MinusJacobian(const double* x, double* jacobian) const = 0;
  virtual int AmbientSize() const = 0;
  virtual int TangentSize() const = 0;
};
```





1

> 2.2.0 中 Ceres::Manifold 的使用
> 
> 和 LocalParameterization 不完全相同，额外定义了 Minus 和 MinusJacobian 虚函数，需要由用户实现

TODO 求解器配置

# 5 求解器配置
