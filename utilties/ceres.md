# Ceres 学习记录

> Google Ceres 非线性最小二乘优化库，十分的强大，十分用的到。
> 
> 相比 g2o，Ceres 应用范围更广，所以先看这个吧。
> 
> 参考
> 
> http://ceres-solver.org/
> 
> Dknt 2023.11

# 0 基础

Ceres 解决约束条件下的多元非线性最小二乘问题：

$$
\begin{split}\min_{\mathbf{x}} &\quad \frac{1}{2}\sum_{i} \rho_i\left(\left\|f_i\left(x_{i_1}, ... ,x_{i_k}\right)\right\|^2\right) \\
\text{s.t.} &\quad l_j \le x_j \le u_j\end{split}
$$

最好安装 ROS 对应的 Ceres 预编译版本。

Cmake 使用：

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
// 函子类
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

# 2 导数

Ceres 提供三种求导方式：

* `Automatic Differentiation`——自动求导

* `Analytic`——解析导数

* `Numeric Derivatives`——数值导数

当优化问题涉及流形、四元数时，求导是一个十分复杂的问题。数值导数有误差、耗时多，推荐使用自动求导，在足够自信的情况下使用解析导数。

## 2.1 自动求导

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

## 2.2 解析导数

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

## 2.3 数值导数

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

# 3 核函数

核函数用于去除外点。Ceres 提供如下核函数：

* 柯西核函数

1

在向优化问题添加残差项时，可以选择添加核函数。如果不使用核函数，则传入`nullptr`

```cpp
// 使用柯西核函数
problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), &m, &c);
// 不使用
```


