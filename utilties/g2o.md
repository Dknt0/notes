# g2o 笔记

> g2o 专用于图优化的非线性最小二乘库，相比 Ceres，应用场景更少，但在 SLAM 问题中似乎能提供更快的速度。g2o 支持 float 型浮点数，而 Ceres 只支持 double。
> 
> ORB-SLAM 中使用了 g2o。
> 
> 版本号 20230806
>
> 参考：
>
> [g2o 内部实现](https://www.jianshu.com/p/36f2eac54d2c)
> 
> Dknt 2023.12

# 0 基础

CMake 中使用 g2o，基于默认安装的 `g2oConfig.cmake`

> g2o 的 cmake 奇奇怪怪的，下面这个配置将就能用

```cmake
find_package(g2o REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    "/path_to_g2o_install/include"
)
target_link_libraries(target_name
    g2o::core
    g2o::stuff
    g2o::solver_dense
    ...
)
```

# 1 基本使用

g2o 中的图优化问题由顶点和边组成。顶点代表一组参数块，即待优化变量，这组变量可能属于欧式空间，也可能属于某个流形，流形上的加法需要由用户自定义。边代表残差项，即观测，用户需要给出误差的计算公式，g2o 中的边分为一元边、二元边、多元边。

> g2o 对用户的的接口不像 Ceres 那么友好。残差、雅可比等重要参数是以成员变量形式存在的，而 Ceres 都写在了参数列表中。

曲线拟合例程

```cpp
#include <Eigen/Core>
#include <chrono>
#include <iostream>
// 核心 api
#include <g2o/core/g2o_core_api.h>
// 顶点与一元边
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
// 块求解器
#include <g2o/core/block_solver.h>
// 线性代数求解器
#include <g2o/solvers/dense/linear_solver_dense.h>
// 最小二乘算法
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#define N_MEASUREMENTS 100
// 顶点，参数块  <切空间维数, 参数块类型>
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  CurveFittingVertex() {}
  virtual bool read(std::istream& is) override { return true; }
  virtual bool write(std::ostream& os) const override { return true; }
  virtual void setToOriginImpl() override { _estimate << 0, 0, 0; }
  virtual void oplusImpl(const double* v) override {
    Eigen::Map<const Eigen::Vector3d> v_vec(v);
    _estimate += v_vec;
  }
 private:
};
// 边，残差块  <残差维数, 观测类型, 连接顶点>
class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  CurveFittingEdge(double x) : BaseUnaryEdge(), x_(x) {}
  virtual bool read(std::istream& is) override { return true; }
  virtual bool write(std::ostream& os) const override { return true; }
  virtual void computeError() override {
    const CurveFittingVertex* v =
        static_cast<const CurveFittingVertex*>(_vertices[0]);
    const Eigen::Vector3d param = v->estimate();
    _error(0, 0) =
        _measurement - std::exp(param[0] * x_ * x_ + param[1] * x_ + param[2]);
  }
  virtual void linearizeOplus() override {
    const CurveFittingVertex* v =
        static_cast<const CurveFittingVertex*>(_vertices[0]);
    const Eigen::Vector3d param = v->estimate();
    double y = std::exp(param[0] * x_ * x_ + param[1] * x_ + param[2]);
    _jacobianOplusXi[0] = -x_ * x_ * y;
    _jacobianOplusXi[1] = -x_ * y;
    _jacobianOplusXi[2] = -y;
  }
 private:
  double x_;  // 输入
};
int main(int argc, char** argv) {
  /**
   * 观测方程如下
   *    y = exp(a*x^2 + b*x + c) + err
   */
  const double a_gt = 1.0;
  const double b_gt = 2.0;
  const double c_gt = 1.0;
  /* 准备数据 */
  srand(std::chrono::steady_clock::now()
            .time_since_epoch()
            .count());  // 随机数种子
  std::vector<double> x_v(N_MEASUREMENTS);
  std::vector<double> y_v(N_MEASUREMENTS);

  for (size_t i = 0; i < y_v.size(); ++i) {
    x_v[i] = static_cast<double>(rand()) / static_cast<double>(RAND_MAX / 1);
    y_v[i] = std::exp(a_gt * x_v[i] * x_v[i] + b_gt * x_v[i] + c_gt);
  }
  /* 配置求解器 */
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>
      BlockSolverType;  // 块求解器  <优化变量维数, 残差维数>
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;  // 线性代数求解器
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      std::make_unique<BlockSolverType>(
          std::make_unique<LinearSolverType>()));  // 图优化求解器
  /* 构造图模型 */
  g2o::SparseOptimizer optimizer;  // 图模型
  optimizer.setAlgorithm(solver);  // 设置求解器对象
  optimizer.setVerbose(true);      // 打开输出
  /* 添加顶点 */
  CurveFittingVertex* v = new CurveFittingVertex;
  v->setId(0);
  v->setEstimate(Eigen::Vector3d(2, -1, 5));  // 设置初值
  optimizer.addVertex(v);
  /* 添加边 */
  for (size_t i = 0; i < N_MEASUREMENTS; ++i) {
    auto edge = new CurveFittingEdge(x_v[i]);
    edge->setId(i);
    edge->setVertex(0, v);  // 连接顶点以及序号
    edge->setMeasurement(y_v[i]);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    optimizer.addEdge(edge);
  }
  /* 优化 */
  optimizer.initializeOptimization();
  optimizer.optimize(30);
  std::cout << "GT: a = " << a_gt << "  b = " << b_gt << "  c = " << c_gt
            << std::endl;
  std::cout << "Result: a = " << v->estimate()[0]
            << "  b = " << v->estimate()[1] << "  c = " << v->estimate()[2]
            << std::endl;
  return 0;
}
```

> 注意以上代码中，Vertex 基类指针转派生类时，使用的是 `static_cast`。下行转换类型不安全，但官方给出的例程就是这么写的，使用时需注意

> g2o 提供了类似于 Ceres 的自动求导

# 2 图优化顶点 Vertex

g2o 顶点是最小二乘问题中的参数块，重点需要实现**变量增量更新方法**。g2o 预先编写了许多顶点类，包括一些流形上的加法。

可以通过顶点类实现对优化变量的查、改。

顶点可以被设置为固定，或开启边缘化。g2o 中的边缘化指计算增量时，先固定边缘化参数，为非边缘化参数求解增量，再计算边缘化参数增量。通常，边缘化参数数量比较多，并且相互之间不存在关联，即其对应的海赛矩阵块是一个块对角矩阵。例如 SLAM 中求解 BA 时，通常会将地图点开启边缘化，这样可以加速求解。

顶点中重要的成员变量 _estimate，即当前估计值

1 列举一些常用的顶点

# 3 图优化边 Edge

g2o 中的边是最小二乘问题中的残差块，重点需要实现**残差计算**和**参数块雅可比计算**。按照连接顶点个数的不同，边分为一元边、二元边、多元边。VSLAM 中通常只用到二元边。g2o 预先编写了许多边类，包括一些扰动雅可比计算。

1 列举一些常用的边

# 4 求解器配置

g2o 优化器由四个级别的求解器组成：

1. 图模型 `Optimizer` —— 因子图，需要向其中添加顶点和边
2. 图优化求解器 `GraphSolver` ——
3. 块求解器 `BlockSolver` —— 用于求解雅可比矩阵和海赛矩阵
4. 线性求解器 `LinearSolver` —— 求解线性方程组

他们之间的层级关系如下：

```cpp
Optimizer.setAlgorithm(make_unique<GraphSolver>(make_unique<BlockSolver>(make_unique<LinearSolver>())));
```


1
