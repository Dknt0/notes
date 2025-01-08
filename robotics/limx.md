LimX TRON1 Note
===

> Reference: [tutorial](https://github.com/limxdynamics/tutorial-docs)
>
> Dknt 2024.12

# 1 Hardware

The IP of the integrated computer is `10.192.1.2`. You can use both wired and wireless connection to connect to it.

Wired connection. Connect to the robot using a cable, then change your wired config. IP: `10.192.169.x`. Netmask: `255.255.255.0`. Gateway: `10.192.1.2`.

> It is said that we need to change computer IP to `10.192.1.200`. But I think this is not necessary.

Wireless connection. Example name of Wifi: `P441C_01`, `PF_P441C_01`. Default password: `12345678`.

When connected, open the url http://10.192.1.2:8080 to check robot information. You can see the robot type from the SN number. For instance, if the SN number is `PF_P441C_037`, the robot type is `PF_P441C`.

Default user name: `guest`. Password: `123456`.

# 2 Development Environment Installation

The develop kits depend on ROS 1 Noetic. Make sure you are working on a computer with Ubuntu 20.04 and ROS Noetic installed.

> The SDK itself is provided as shared libraries.

Install dependencies:

```shell
sudo apt install ros-noetic-urdf ros-noetic-kdl-parser ros-noetic-urdf-parser-plugin ros-noetic-hardware-interface ros-noetic-controller-manager ros-noetic-controller-interface ros-noetic-controller-manager-msgs ros-noetic-control-msgs ros-noetic-gazebo-* ros-noetic-rqt-gui ros-noetic-rqt-controller-manager ros-noetic-plotjuggler* ros-noetic-joy-teleop ros-noetic-joy cmake build-essential libpcl-dev libeigen3-dev libopencv-dev
```

Then create a ROS workspace, install the following packages in the `src` folder in your workspace.

```shell
# Motion control development interface
git clone https://github.com/limxdynamics/pointfoot-sdk-lowlevel.git
# Gazebo simulator
git clone https://github.com/limxdynamics/pointfoot-gazebo-ros.git
# Robot model describing file
git clone https://github.com/limxdynamics/robot-description.git
# Visualized debug tool
git clone https://github.com/limxdynamics/robot-visualization.git
# RL deployment
git clone https://github.com/limxdynamics/rl-deploy-ros-cpp.git
```

Then build the workspace.

There is a virtual joystick provided by LimX.

```shell
# Virtual joystick
git clone https://github.com/limxdynamics/robot-joystick.git
```

Available robot types are those directories in the `robot-description` package. Use `tree` to show them:

```shell
tree -L 1 src/robot-description/pointfoot
```

When using the package, you should set the type of your target robot as a global variable to make sure the simulator works properly.

```shell
export ROBOT_TYPE=PF_TRON1A
```

# 3 Simulation

Run the example simulation and control program.

```shell
# Start a simulation
roslaunch pointfoot_gazebo empty_world.launch
# Run the control program
rosrun pointfoot_sdk_lowlevel pf_groupJoints_move
# Run PlotJuggler to visualize robot states
roslaunch robot_visualization pointfoot_plot_sim.launch
# Run RViz to visualize robot states
roslaunch robot_visualization pointfoot_rviz_hw.launch
```

## 3.1 C++ Low Level API

Simulator IP: `127.0.0.1`.

Real robot IP: `10.192.1.2`.

Robot states contain the position, velocity and torque of each joint.

Control commands are saved as `datatypes.RobotCmd`, which contains the following attributes:

* `stamp` - time stamp in ns.
* `mode` - control mode. 0: effort control; 1: velocity control; 2: position control.
* `q` - joint position in rad.
* `dq` - joint velocity in rad/s.
* `tau` - effort in N\*m.
* `Kp` - stiffness N\*m/rad.
* `Kd` - damping N\*m/(rad\*s).

Subscription robot state order:

| Index | Point-foot   | Wheel-foot    | Bipedal       |
| ----- | ------------ | ------------- | ------------- |
| 0     | abad_L_Joint | abad_L_Joint  | abad_L_Joint  |
| 1     | hip_L_Joint  | hip_L_Joint   | hip_L_Joint   |
| 2     | knee_L_Joint | knee_L_Joint  | knee_L_Joint  |
| 3     | abad_R_Joint | wheel_L_Joint | ankle_L_Joint |
| 4     | hip_R_Joint  | abad_R_Joint  | abad_R_Joint  |
| 5     | knee_R_Joint | hip_R_Joint   | hip_R_Joint   |
| 6     | -            | knee_R_Joint  | knee_R_Joint  |
| 7     | -            | wheel_R_Joint | ankle_R_Joint |

Joystick map:

| Axes Name      | Axes Index | Buttons name | Buttons Index |
| -------------- | ---------- | ------------ | ------------- |
| left_horizon   | 0          | X            | 0             |
| left_vertival  | 1          | 〇           | 1             |
| right_horizon  | 2          | 口           | 2             |
| right_vertival | 3          | △            | 3             |
|                |            | L1           | 4             |
|                |            | L2           | 6             |
|                |            | R1           | 7             |
|                |            | R2           | 5             |
|                |            | SELECT       | 8             |
|                |            | START        | 9             |
|                |            | UP           | 12            |
|                |            | DOWN         | 13            |
|                |            | LEFT         | 14            |
|                |            | RIGHT        | 15            |
|                |            | MENU         | 16            |
|                |            | BACK         | 17            |

API brief description:

```cpp
// Get the instance of PointFoot
PointFoot* pf = PointFoot::getInstance();
// Initialization
pf->init("your-robot-ip");
// Get the number of motors
uint32_t motor_num = pf->getMotorNumber();
// Register an IMU callback
pf->subscribeImuData([&](const ImuDataConstPtr& msg) { /* Do something */ });
// Register a Robot state callback
pf->subscribeRobotState([&](const RobotStateConstPtr& msg) { /* Do something */ });
// Publish robot command, including position, velocity, torque, and pos/vel stiffness.
pf->publishRobotCmd(RobotCmd(motor_num));
// Register a joystick event callback
pf->subscribeSensorJoy([&](const SensorJoyConstPtr&joy) { /* Do something */ });
// Register a diagnose callback
pf->subscribeDiagnosticValue([&](const DiagnosticValueConstPtr&msg) { /* Do something */ });
// Set robot light
pf->setRobotLightEffect(limxsdk::PointFoot::STATIC_RED);
```

## 3.2 RL Motion Control

We are talking about deploying a model trained in Legged Gym.

Install `onnxruntime` library version v1.10.0 [here](https://github.com/microsoft/onnxruntime/releases/tag/v1.10.0).

The default weight file is located at `rl-deploy-ros-cpp/robot_controllers/config/pointfoot/${ROBOT_TYPE}/policy`.

Run the simulator, then run the controller:

```shell
roslaunch robot_hw pointfoot_hw_sim.launch
```

Run the following script to convert a model to onnx:

```shell
python legged_gym/scripts/export_policy_as_onnx.py --task=pointfoot_rough --load_run=Dec13_09-32-34_ --checkpoint=15000
```

# 4 Deployment

Run a real world deployment test:

```shell
rosrun pointfoot_sdk_lowlevel pf_groupJoints_move 10.192.1.2
```

To run data visualization, you should first change the agent IP in `pointfoot_plot_hw.launch` or `pointfoot_rviz_hw.launch`. Then Run the command:

```shell
# Run PlotJugler
roslaunch robot_visualization pointfoot_plot_hw.launch
# Run RViz
roslaunch robot_visualization pointfoot_rviz_hw.launch
```

The robot automatically records all its state as ROS bags. You can download the data bag from `10.192.1.2:8090`, then visualize it using `PlotJuggler`. If the bag ended with `active`, you should re-index it:

```shell
rosbag reindex your_file.bag.active
mv your_file.bag.active your_file.bag
```

# 5 System Structure

The low-level communication of TRON1 is based on EtherCat. However, the implementation details are hidden for us. The SDK provides a low-level API, which can be used to do **PD joint control** in position, velocity and effort mode. The only way we can control the motor is the `publishRobotCmd` function.

The cpp deployment code is implemented based on the `hardware_interface` of ROS.

The developers write two customized `hardware_interface` classes: `HybridJointInterface` and `ContactSensorInterface`. See `rl-deploy-ros-cpp/robot_common`.

Interfaces in the point foot hw: `jointStateInterface_`, `hybridJointInterface_`, `imuSensorInterface_` and `contactSensorInterface_`. They provide resources to the controller named `robot_controller/PointfootController`.

## 5.1 Robot Hardware Interface

LimX developers provide a library to read and write all components of the robot, such as drivers, encoders, sensors and joystick, hiding the EtherCat communication details.

**Observation**: Callback functions are registered in LimX subscription `subscribeRobotState` and `subscribeImuData`, in which the sensor data are saved in realtime buffers `realtime_tools::RealtimeBuffer<data-type>` in a non-RT mode. Then, the `read` function simply retrieves the data from the buffers without EtherCat related things.

**User Command**: A callback is registered in `subscribeSensorJoy`, which check the joystick state then start or stop the controller calling `startBipedController` and `stopBipedController`. It also computes a robot velocity command then publishes it to the controller via ROS topic.

**Failure Handle**: A callback is registered in `subscribeDiagnosticValue`. It updates the calibration state, and shut the robot down via `stopBipedController` when error occurs.

**read()** just copy the joint and IMU data from RT buffer to the data buffer in RobotHW. The contact sensor is not handled.

**write()** constructs the robotCmd_ according to joint buffer in RobotHW, then publish it to the controller using LimX API `publishRobotCmd`.

In `setupJoints`, `setupImu` and `setupContactSensor`, the hardware handles, which take the pointer to the data buffer in RobotHW as parameters, are created then registered into the corresponded hardware interfaces.

> Is the urdf file really necessary in `setupJoints`?

`startBipedController` calls `switch_controllers_client_` with `controller_manager_msgs::ListControllers` and `list_controllers_client_` with `controller_manager_msgs::SwitchController` to check then start a controller named `/controllers/pointfoot_controller`.

`stopBipedController` calls `switch_controllers_client_` to stop the controller, then use LimX API `publishRobotCmd` to set the robot to damping mode.

## 5.2 RobotHWLoop

`RobotHWLoop` is a class to run the control loop. It contains a `controller_manager` as member variable. The **control frequency** is set to 500Hz and the **cycle time error threshold** is 0.002s by default. `RobotHWLoop` starts a loop thread in constructor, periodically executing the control loop: `HardwareInterface::read`, `ControllerManager::update` and `HardwareInterface::write`.

> `std::chrono::high_resolution_clock` is used.

## 5.3 Robot Controller

> Parameters in `robot_controllers/config/pointfoot/${ROBOT_TYPE}/params.yaml` are worth checking.

Controllers are ROS plugins. They inherit from `controller_interface::MultiInterfaceController<hardware-interface-classes>`. In the `init()` function, the pointer of `RobotHW` is given as parameter, then `loadRLCfg()` and `loadModel` are called. The controller can get hardware interface or handle via `robot_hw->get<interface-class>()`.

`PointfootController` inherits from `ControllerBase`.

In `loadRLCfg()`, the controller reads the parameters from the ROS parameter server, not directly from the yaml file.

In `loadModel()`, Ort environment and session are created, then the input and output parameters are retrieved.

In `starting()`, the default joint position are set to the current joint position.

In `update()`, functions `handleStandMode()` and `handleWalkMode()` according to the `mode_`.

In `handleStandMode()`, the robot move from the current position to the stand position. But in the stand position the joint positions are all zero.

In `handleWalkMode()`, `computeObservation()` and `computeActions` are called if the decimation is meet, i.e., the policy inference is not updated in every control cycle. Then the target joint positions are computed taking maximum torque as constraint then set in joint handle.

In `computeObservation()` the orientation is get from the IMU. Then the observation is computed.

The controller is loaded via `controller_manager/controller_manager` in the launch file.

> What is `power_limit` and `contact_threshold`?
> `imu_orientation_offset`
