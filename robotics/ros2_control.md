ROS2 Control
===

> To write a new simulation framework.
> Dknt 2025.07
> 
> Reference:
> [ros2_control Documentation](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)

## 0 Installation

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros2-control-cmake ros-jazzy-rqt-controller-manager ros-jazzy-gz-ros2-control
```

# 1 Components and Usage

## 1.1 Controller Manager

Controller Manager is the main component in the ros2_control framework. It manages lifecycle of controllers, access to the hardware interfaces and offers services to the ROS-world. It's better to use the default `ControllerManager`.

The main thread of CM attempts to configure `SCHED_FIFO` with a priority of `50` by default.

Execution of the control-loop is managed by `update()` function.

Helper scripts:

```bash
# Load, configure and start a controller on startup
ros2 run controller_manager spawner
# Stops and unloads a controller
ros2 run controller_manager unspawner
# Activate and configure a hardware component
ros2 run controller_manager hardware_spawner
# GUI tool
ros2 run rqt_controller_manager rqt_controller_manager
```


### Realtime setting

Add user to `realtime` group:

```bash
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```

Add the following limits to the realtime group in `/etc/security/limits.conf`:

```conf
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
```

Pass realtime permissions to docker container:

```bash
docker run -it --cap-add=sys_nice --ulimit rtprio=99 --ulimit memlock=-1 --rm --net host <IMAGE>
```

There are 3 realtime policies on Linux: `SCHED_FIFO`, `SCHED_RR` and `SCHED_DEADLINE`. The priority range of the first two is from `0` to `99`, where `0` is the lowest priority (but in the kernel `0` is the hightest one). `SCHED_DEADLINE` can be seen as priority `-1`.

---

**Resource Manager**. `read()`, `write()`.

## 1.2 Controller

For our task, we do not need to write a controller by ourselves.

`update()`.

Commonly used controllers:

- `joint_state_broadcaster`
- `forward_position_controller`

How to write customized controllers?

---

## 1.3 Hardware Components

System. Sensor. Actuator.

### 1.3.1 System

### 1.3.2 Actuator

### 1.3.3 Sensor

---

**User Interface**.

**Hardware Description in URDF**.





