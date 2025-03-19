Mujoco Note
===

> I really don't want to use Mujoco. I prefer Gazebo, instead. But we need a simulator to test our model right now.
>
> Dknt 2025.02.22

# Installation

Install the biranry package of mujoco from the [official website](https://mujoco.org/).

Then install the python package:

```bash
pip install mujoco mujoco-python-viewer
```

For more information, refer to [documentation](https://mujoco.readthedocs.io/en/stable/overview.html).

# 1 Basic Usage

Run a simulation:

```bash
simulate robot.xml
```

## 2.1 Convert URDF to XML

Add these lines in the `<robot>` tag of the URDF file:

```xml
<mujoco>
    <compiler meshdir="../meshes/" balanceinertia="true" discardvisual="false" />
</mujoco>
```

```bash
compile robot.urdf robot.xml
```

