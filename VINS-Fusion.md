# VINS-Fusion

> HKUST Aerial Robotics group's open source VIO framework
> 
> Reference:
> 
> [GitHub - HKUST-Aerial-Robotics/VINS-Fusion: An optimization-based multi-sensor state estimator](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
> 
> Dknt 2023.11

Camera extrinsic parameters estimation. IMU bias estimation.

Using optical flow and Shi-Tomasi corners.

Moving initialization, recovery when failed.

IMU pre-integration.

Tightly coupled optimization based on sliding window.

Loop closing.

4-DoF pose graph optimization (x, y, z, yaw) considering that IMU provides accurate roll and pitch angles.

VINS-Fusion can be divided into 3 processes:

* `vins_estimator` VIO

* `loop_fusion` Loop closing

* `global_fusion` global position sensor (e.g. GNSS) fusion

The `vins_estimator` is the basic VIO process using sliding window method to evaluate current camera pose, which is crucial for us. Loop closing and global fusion are optional.

**VINS can calibrate camera extrinsic parameters at run time.**

VINS-Fusion relies on ROS, it can not run without ROS1.

> On the KITTI official leaderboard (as of November 27, 2023), the accuracy of VINS-Fusion surpasses that of ORB-SLAM2. This suggests that the mid-term data fusion in ORB-SLAM may not have played a significant role. The key to odometry accuracy lies in the sliding window, while the key to global trajectory accuracy lies in loop closure detection. Moreover, global BA (Bundle Adjustment) does not significantly improve accuracy compared to pose graph optimization, yet it consumes a substantial amount of computational power.

# 0 Framework and coding style

Global variables are widely used in the source code of VINS, instead of class public members, and there is no locks for them. It is better to preserve them in a single class.

# 1 Visual Odometry `vins_estimator`

There are 3 .cpp file under `vins_estimator/src`, among them 1 ROS interface node and 2 test nodes on KITTI dataset.

The other folds correspond to the following modules:

* `estimator` Estimator class, Feature management, Parameter loading (global)

* `factor` Residual, manifold classes in Ceres.

* `featureTracker` Feature tracking and optical flow.

* `initial` Initialization tool.

* `utility` ROS publishers, math tools.

1 TODO

Keyframe Selection policies:
1. 
2. 

## 1.1 Odometry?

`vins_estimator/src/rosNodeTest.cpp`。

Read config file path from command line, initialize `Estimator` and Publishers.

The system class `Estimator` is driven by data. There are 3 types of data: image, IMU, features, which are passed to estimator through `inputImage`, `inputIMU` and `inputFeature。`.

`Estimator` provides interfaces to reset system and change running mode.

## 1.2 Estimator Class?

`Estimator` is similar to `Tracking` in ORB-SLAM.

`Estimator` can set `processMeasurements` as a single thread or as data-driven.

1

The feature in vins has the type of `pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>`.

inputFeature may not be called.

1

In `CostFunction` information matrixes are set.

1

> Marginalization?

1

## 1.3 FeatureTracker Class

VINS is based on optical flow using Shi-Tomasi features,

Features in VINS have the type of`map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>`

1

# 2 Loop Closing `loop_fusion`

1

# 3 Global Sensor Fusion `global_fusion`





