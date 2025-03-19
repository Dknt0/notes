X-Humanoid Note
===

> X-Humanoid ROS1 SDK.

# RL Control Plugin

The RL control of X-Humanoid SDK is written in a `nodlet` plugin, which overrides an `onInit()` method. `libtorch` is used for network inference. The inference is done in `RobotFSM`, which is close-source.

Interface:

```
out:
    /BodyControl/motor_ctrl
in:
    /BodyControl/motor_state
    /BodyControl/imu
    /sbus_data  # Joystick
    /cmd_vel
```

# Close Chain Leg Motor Mapping

The `BodyControl` class initializes all hardware resources and enters a loop to publish messages in the msg queue.

> How to control the close chain leg motor?

