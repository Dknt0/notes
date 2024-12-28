Isaac Gym Note
===

Isaac Gym is a GPU based RL simulator, which runs simulation on the GPU and stores tensors in GPU directly. It uses PhysX simulation engine.

Key features:

1. Support for importing URDF and MJCF files
2. GPU accelerated tensor API for evaluating environment state and applying actions
3. Support for a variety of environment sensors - position, velocity, force, torque, etc
4. Runtime domain randomization of physics parameters
5. Jacobian / inverse kinematics support

> As I have started to learn reinforcement learning, Isaac Gym is no longer supported. But most of legged RL practitioners still use it.
>
> Reference:
> Isaac Gym Documentation (from the downloaded package).
>
> Dknt 2024.12

# 0 Installation

Create a conda environment:

```shell
conda create -n legged-gym python==3.8
conda install pytorch torchvision torchaudio pytorch-cuda=12.4 -c pytorch -c nvidia
```

Install `isaac_gym`:

```shell
wget https://developer.nvidia.com/isaac-gym-preview-4
tar -xf IsaacGym_Preview_4_Package.tar.gz
cd isaacgym/python && pip install -e .
```

Then try to run some examples:

```shell
cd examples
python 1080_balls_of_solitude.py
```

> Troubleshooting: Refer to Legged Gym Note.

# 1 Simulation Programming

## 1.1 Simulation Setup

Isaac Gym provide a procedural and data-oriented API to exchange information between the core implementation written in C++ and client scripts written in Python. The proxy of API can be retrieved from `acquire_gym`.

```py
# The core API is defined in the gymapi module
from isaacgym import gymapi
# All of API functions can be accessed from the gym singleton
gym = gymapi.acquire_gym()
```

The following options are always available in Isaac Gym after `gymutil.parse_arguments()` is called:

```py
from isaacgym import gymutil
args = gymutil.parse_arguments()
```

1. `--help` Print out CL options
2. `--physx` Use PhyX as the physics backend
3. `--flex` Use Flex as the physics backend
4. `--sim_device` Choose the device to run simulation. Can be `cpu` or `cuda`. Default is `cuda:0`
5. `--pipeline` Choose pipeline for tensor operations. Can be `cpu` or `gpu`. Default is `gpu`.
6. `--graphics_device_id` Specify the device id used for graphics

To create a simulation, you should use `create_sim`: 

```py
# Call create_sim method to create a simulation
sim_params = gymapi.SimParams()  # sim_params can be customized
# sim object allows to load assets, create environments, and interact with the simulation
# If graphics_device_id=-1, the simulation will run in headless mode
# The third argument is physics backend, which can be SIM_PHYSX or SIM_FLEX
sim = gym.create_sim(compute_device_id, graphics_device_id, gymapi.SIM_PHYSX, sim_params)
# (Optional) Initialize the internal data structures used by the tensor API
gym.prepare_sim(sim)
```

An example `sim_params` is shown below:

```py
# Set common parameters
sim_params.dt = 1 / 60
sim_params.substeps = 2
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
# Set PhysX-specific parameters
sim_params.physx.solver_type = 1
sim_params.physx.num_position_iterations = 6
sim_params.physx.num_velocity_iterations = 1
sim_params.physx.contact_offset = 0.01
sim_params.physx.rest_offset = 0.0
# (Optional) Use GPU pipeline, enable Tensor API
sim_params.use_gpu_pipeline = True  # the tensors returned will reside on GPU
sim_params.physx.use_gpu = True
# Set Flex-specific parameters
sim_params.flex.solver_type = 5
sim_params.flex.num_outer_iterations = 4
sim_params.flex.num_inner_iterations = 20
sim_params.flex.relaxation = 0.8
sim_params.flex.warm_start = 0.5
```

To create a ground plane, we should call `add_ground`:

```py
# configure the ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)  # z-up!
plane_params.distance = 0
plane_params.static_friction = 1
plane_params.dynamic_friction = 1
plane_params.restitution = 0  # elasticity
# create the ground plane
gym.add_ground(sim, plane_params)
```

## 1.2 Assets

Loading an asset file creates a `GymAsset` object including the definiton of all the bodies, collision shapes, visual attachments, joints, and degrees of freedom (DOFs). `GymAsset` can be instanced multiple times a simulation with different poses and individualized properties.

Assets are loaded from `load_asset` with asset root path and asset file relative path.

```py
asset_root = "../../assets"
asset_file = "urdf/franka_description/robots/franka_panda.urdf"
asset = gym.load_asset(sim, asset_root, asset_file)
```

Assets can also be loaded with specified parameters:

```py
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
asset_options.armature = 0.01
asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
```

### 1.2.1 Assets API

TODO

`get_asset_rigid_body_count`

`get_asset_rigid_shape_count`

`get_asset_dof_count`

`get_asset_dof_properties`

Contains joint position, velocity limits.

`get_asset_rigid_shape_properties`

Rigid shape property contains friction coefficient.

`get_asset_dof_names`

`get_asset_rigid_body_names`


### 1.2.2 Inertial Properties

Isaac Gym can override inertial properties of rigid bodied from file. Refer to documentation.

### 1.2.3 Convex Decomposition

Isaac Gym supports automatic convex decomposition of triangle meshes based on V-HACD for collision shapes. PhysX requires convex meshes for collisions. Without CD, each triangle mesh shape is approximated using a single convex hull, which is not accurate.

```py
# CD is disabled by default
asset_options.vhacd_enabled = True
# Set CD parameters
asset_options.vhacd_params.resolution = 300000
asset_options.vhacd_params.max_convex_hulls = 10
asset_options.vhacd_params.max_num_vertices_per_ch = 64
```

### 1.2.4 Procedural Assets

Simple geometric assets can be created procedurally:

```py
asset_options = gym.AssetOptions()
asset_options.density = 10.0
# Create asset
box_asset = gym.create_box(sim, width, height, depth, asset_options)
sphere_asset = gym.create_sphere(sim, radius, asset_options)
capsule_asset = gym.create_capsule(sim, radius, length, asset_options)
```

## 1.3 Environments and Actors

An **environment** consists of a collection of **actors** and **sensors** that are simulated **together**. Actors within an environment interact with each other physically. An Isaac Gym simulation can contain multiple environments. Once we finish populating one environment and start populating the next one, we **can no longer** add actors to the previous environment.

We should create an environment by calling `create_env` first, before adding actors in it. Each environment has its own coordinate space. New environments are added as a 2D grid one row at a time.

```py
spacing = 2.0
lower = gymapi.Vec3(-spacing, 0.0, -spacing)
upper = gymapi.Vec3(spacing, spacing, spacing)
# The last parameter 8 is number of envs in a row
env = gym.create_env(sim, lower, upper, 8)
```

We add an actor to an environment using `create_actor` with specified source asset, desired pose, and a few other details.

```py
pose = gymapi.Transform()
pose.p = gymapi.Vec3(0.0, 1.0, 0.0)
pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107)

# Arguments after pose are optional. They are name, collision_group and collision_filter
actor_handle = gym.create_actor(env, asset, pose, "MyActor", 0, 1)
```

- `collision_group` - Two bodies collide with each other only if they belong to the same collision group. the value -1 is used for a special group that collides with all other groups in all environments.
- `collision_filter` - Two bodies will not collide if their collision filters have a common bit set. This is used to filter out self-collisions.

An example script to initialize environments recursively:

```py
# set up the env grid
num_envs = 64
envs_per_row = 8
env_spacing = 2.0
env_lower = gymapi.Vec3(-env_spacing, 0.0, -env_spacing)
env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)
# cache some common handles for later use
envs = []
actor_handles = []
# create and populate the environments
for i in range(num_envs):
    env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
    envs.append(env)
    # Random hight
    height = random.uniform(1.0, 2.5)
    # Pose of actor
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, height, 0.0)
    pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
    # Create actor
    actor_handle = gym.create_actor(env, asset, pose, "MyActor", i, 1)
    actor_handles.append(actor_handle)
```

### 1.3.1 Actors Numpy API

Each actor has an array of rigid bodies, joints, and DoFs.

Functions operating on actors are named `get_actor_*`, `set_actor_*`, or `apply_actor_*`. These functions are using numpy arrays to store the data.

The following functions get the property counters of an actor.

```py
num_bodies = gym.get_actor_rigid_body_count(env, actor_handle)
num_joints = gym.get_actor_joint_count(env, actor_handle)
num_dofs = gym.get_actor_dof_count(env, actor_handle)
```

#### 1.3.1.1 Actors DOF API

Table of DOF properties:

| Name      | Data type           | Description                          |
|-----------|---------------------|--------------------------------------|
| hasLimits | bool                | DOF motion is limited or unlimited   |
| lower     | float32             | Lower limit.                         |
| upper     | float32             | Upper limit.                         |
| driveMode | gymapi.DofDriveMode | DOF drive mode, see below.           |
| stiffness | float32             | Drive stiffness.                     |
| damping   | float32             | Drive damping.                       |
| velocity  | float32             | Maximum velocity.                    |
| effort    | float32             | Maximum effort (force or torque).    |
| friction  | float32             | DOF friction.                        |
| armature  | float32             | DOF armature.                        |

There are four dof modes:

1. `DOF_MODE_NONE` - Underactuated.
2. `DOF_MODE_EFFORT` - Force control.
3. `DOF_MODE_VEL` - Velocity control with non-zero `damping`.
4. `DOF_MODE_POS` - Position control with non-zero `stiffness` and `damping`.

`DOF_MODE_VEL` and `DOF_MODE_POS` engage a PD controller, which applies DOF forces that are proportional to `posError * stiffness + velError * damping`.

We use following syntaxes to set or apply controls:

```py
# Configure the joint control mode (once)
props = gym.get_actor_dof_properties(env, actor_handle)
props["driveMode"].fill(gymapi.DOF_MODE_EFFORT)  # One actor can use various drive modes
props["stiffness"].fill(1000.0)
props["damping"].fill(200.0)
gym.set_actor_dof_properties(env, actor_handle, props)

# Apply efforts for all DoFs of an actor (every frame)
efforts = np.full(num_dofs, 100.0).astype(np.float32)
gym.apply_actor_dof_efforts(env, actor_handle, efforts)

# Set velocities for all DoFs of an actor (not necessarily every frame)
vel_targets = np.random.uniform(-math.pi, math.pi, num_dofs).astype('f')
gym.set_actor_dof_velocity_targets(env, actor_handle, vel_targets)

# Set positions for all DoFs of an actor (not necessarily every frame)
targets = np.zeros(num_dofs).astype(np.float32)
gym.set_actor_dof_position_targets(env, actor_handle, targets)
```

Efforts must be applied every frame. Applying efforts multiple times during each frame is cumulative, but they are reset to zero at the beginning of the next frame.

> A control value from the array will only be applied to its corresponding DOF if that DOF was configured to use that drive mode.

We can work with the state of an actor:

```py
# Get the state of all DoFs of an actor
dof_states = gym.get_actor_dof_states(env, actor_handle, gymapi.STATE_ALL)
# Accessing the state
dof_states["pos"]   # all positions
dof_states["vel"]   # all velocities
# Set the state of all DoFs of an actor
gym.set_actor_dof_states(env, actor_handle, dof_states, gymapi.STATE_ALL)
```

#### 1.3.1.2 Actors Rigid Body API

Rigid body states includes position (Vec3), orientation (Quat), linear velocity (Vec3), and angular velocity (Vec3). We can get the state of rigid bodies from actor_handle, env, or sim:

```py
body_states = gym.get_actor_rigid_body_states(env, actor_handle, gymapi.STATE_ALL)
body_states = gym.get_env_rigid_body_states(env, gymapi.STATE_ALL)
body_states = gym.get_sim_rigid_body_states(sim, gymapi.STATE_ALL)
```

The state can be accessed like this:

```py
body_states["pose"]["p"]        # all positions (Vec3: x, y, z)
body_states["pose"]["r"]        # all orientations (Quat: x, y, z, w)
body_states["vel"]["linear"]    # all linear velocities (Vec3: x, y, z)
body_states["vel"]["angular"]   # all angular velocities (Vec3: x, y, z)
```

We can set the state of rigid bodies like this:

```py
gym.set_actor_rigid_body_states(env, actor_handle, body_states, gymapi.STATE_ALL)
gym.set_env_rigid_body_states(env, body_states, gymapi.STATE_ALL)
gym.set_sim_rigid_body_states(sim, body_states, gymapi.STATE_ALL)
```

We use `find_actor_rigid_body_index` to find the index of a rigid body in an actor.

```py
# Get the index in buffer from get_actor_rigid_body_states
i1 = gym.find_actor_rigid_body_index(env, actor_handle, "body_name", gymapi.DOMAIN_ACTOR)
# Get the index in buffer from get_env_rigid_body_states
i2 = gym.find_actor_rigid_body_index(env, actor_handle, "body_name", gymapi.DOMAIN_ENV)
# Get the index in buffer from get_sim_rigid_body_states
i3 = gym.find_actor_rigid_body_index(env, actor_handle, "body_name", gymapi.DOMAIN_SIM)
```

### 1.3.2 Aggregates

> Aggregates is available in PhysX backend.

Aggregate, a collection of actors, provides a modest performance boost. To place actors into an aggregate, we should call `create_actor` between `begin_aggregate` and `end_aggregate`. Only actors from the same env can be included in an aggregate.

```python
gym.begin_aggregate(env, max_bodies, max_shapes, True)
gym.create_actor(env, ...)
gym.create_actor(env, ...)
gym.create_actor(env, ...)
...
gym.end_aggregate(env)
```

## 1.4 Running the Simulation

After setting up, the simulation can be run in a loop.

```py
while True:
    gym.simulate(sim)
    gym.fetch_results(sim, True)
```

## 1.5 Viewer

We create a viewer by calling `create_viewer`:

```py
cam_props = gymapi.CameraProperties()
viewer = gym.create_viewer(sim, cam_props)
```

To update the viewer, call `step_graphics` and `draw_viewer` during every iteration:

```py
gym.step_graphics(sim)
gym.draw_viewer(viewer, sim, True)
```

Example script of a simulation loop with viewer is shown below:

```py
# Create viewer
cam_props = gymapi.CameraProperties()
viewer = gym.create_viewer(sim, cam_props)
while not gym.query_viewer_has_closed(viewer):
    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)
    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)
    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)
```

When a viewer is created, press `Tab` to toggle on and off the GUI panel. The GUI has 4 tabs: `Actor`, `Sim`, `Viewer`, and `Perf`.

* `Actor` tab provides the ability to select an environment and an actor within that environment. We can get information, change the color, override the pose of an actor. 
* `Sim` tab shows the physics simulation parameters.
* `Viewer` tab allows customizing common visualization options.
* `Pref` tab shows 

We can register mouse/keyboard events for the viewer. See `examples/projectiles.py`. An example:

```py
gym.subscribe_viewer_keyboard_event(viewer, gymapi.KEY_SPACE, "space_shoot")
gym.subscribe_viewer_keyboard_event(viewer, gymapi.KEY_R, "reset")
gym.subscribe_viewer_mouse_event(viewer, gymapi.MOUSE_LEFT_BUTTON, "mouse_shoot")
...
while not gym.query_viewer_has_closed(viewer):
    ...
    for evt in gym.query_viewer_action_events(viewer):
        ...
```

## 1.5 Cleanup

Release the sim and viewer object at exit:

```py
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
```

# 2 Tensor API

Isaac Gym also provides a tensor API, which makes it possible to set up experiments that run fully on GPU. The Gym tensor API works with **global** tensors that hold the values of all actors in the simulation.

The Gym tensor API uses simple *tensor descriptors*, which specify the device, memory address, data type, and shape of a tensor. The tensor descriptors can be converted to PyTorch tensors (and only PyTorch tensors) by `gymtorch.wrap_tensor`, and all of the existing tensor PyTorch utilities can then be used. The physics tensors acquired hold a copy of the simulation state.

To use tensor API, we must use `SIM_PHYSX` with GPU pipeline and call `prepare_sim` before using any tensor API functions. When creating a simulation, add these parameters:

```py
sim_params.use_gpu_pipeline = True  # the tensors returned will reside on GPU
sim_params.physx.use_gpu = True

sim = gym.create_sim(compute_device_id, graphics_device_id, gymapi.SIM_PHYSX, sim_params)
# Initialize the internal data structures used by the tensor API
gym.prepare_sim(sim)
```

## 2.1 Root State Tensor

We use `acquire_actor_root_state_tensor` to get the root state tensor of all actors. The state of each root body is a 13-dim float32 tensor, including 3d-position, 4d-orientation, 3d-linear velocity, and 3d-angular velocity.

```py
# _root_tensor is generic tensor descriptor
_root_tensor = gym.acquire_actor_root_state_tensor(sim)
# Convert the descriptor to a PyTorch float32 tensor
root_tensor = gymtorch.wrap_tensor(_root_tensor)
```

These functions only need to be called once, because their contents can be updated using `refresh_actor_root_state_tensor` later.

```py
# Refresh the root state Pytorch tensor
gym.refresh_actor_root_state_tensor(sim)
```

The root state tensor can then be used to set the state of actors, which is useful during resets.

```py
# Set the state through the tensor descriptor
gym.set_actor_root_state_tensor(sim, _root_tensor)
# Set the state through the PyTorch tensor
gym.set_actor_root_state_tensor(sim, gymtorch.unwrap_tensor(root_tensor))
```

To update the state of a *subset* of actors, we use `set_actor_root_state_tensor_indexed`. For instance:

```py
# Create an index tensor and set the root state of the specified actors
actor_indices = torch.tensor([0, 17, 42], dtype=torch.int32, device="cuda:0")
gym.set_actor_root_state_tensor_indexed(sim, _root_states, gymtorch.unwrap_tensor(actor_indices), 3)
```

> **Troubleshooting**
>
> 1. Python's **garbage collection** may cause some memory issues when sharing PyTorch tensors with Gym. Refer to documentation.
> 2. The setter and refresh functions should be called only once between two `simulate` calls.

## 2.2 DOF State Tensor

The shape of the Dof state tensor is `(num_dofs, 2)`, in which the first column is the position and the second column is the velocity. The tensor begins with all the DOF of actor 0, followed by all the DOF of actor 1, and so on. Common used functions:

```py
# Get the tensor descriptor
_dof_tensor = gym.acquire_dof_state_tensor(sim)
# Convert the descriptor to a PyTorch float32 tensor
dof_tensor = gymtorch.wrap_tensor(_dof_tensor)
# Refresh the DOF state Pytorch tensor
gym.refresh_dof_state_tensor(sim)
# Set the state through the tensor descriptor
gym.set_dof_state_tensor(sim, _dof_tensor)
# Set the state through the PyTorch tensor
gym.set_dof_state_tensor(sim, gymtorch.unwrap_tensor(dof_tensor))
# Create an index tensor and set the DOF state of the specified actors
actor_indices = torch.tensor([0, 17, 42], dtype=torch.int32, device="cuda:0")
gym.set_dof_state_tensor_indexed(sim, _dof_states, gymtorch.unwrap_tensor(actor_indices), 3)
```

The global index of a specific DOF in the tensor can be obtained in a number of ways:

1. `gym.get_actor_dof_index(env, actor_handle, i, gymapi.DOMAIN_SIM)` returns the global DOF index of the i-th DOF of the actor.
2. `gym.find_actor_dof_index(env, actor_handle, dof_name, gymapi.DOMAIN_SIM)` returns the global DOF index of the named DOF.

## 2.3 Rigid Body State Tensor

The rigid body tensor contains the state of all rigid bodies in the simulation. The shape of the rigid body state tensor is `(num_rigid_bodies, 13)`. The rigid body states are laid out sequentially. Common used functions:

```py
# Get the rigid body state tensor descriptor
_rb_tensor = gym.acquire_rigid_body_state_tensor(sim)
# Convert the descriptor to a PyTorch float32 tensor
rb_tensor = gymtorch.wrap_tensor(_rb_tensor)
# Refresh the rigid body state Pytorch tensor
gym.refresh_rigid_body_state_tensor(sim)
```

> The rigid body state tensor is read-only.

The global index of a specific rigid body in the tensor can be obtained in a number of ways:

1. `gym.get_actor_rigid_body_index(env, actor_handle, i, gymapi.DOMAIN_SIM)` returns the global rigid body index of the i-th rigid body of the actor.
2. `gym.find_actor_rigid_body_index(env, actor_handle, rb_name, gymapi.DOMAIN_SIM)` returns the global rigid body index of the named rigid body.

## 2.4 Jacobian and Mass Matrix Tensor

When acquiring a Jacobian or mass matrix tensor, we must specify the actor name. The tensor will then contain the matrices for all the actors with that name across all environments - assuming that each actor with that name is of the same type. Example code:

```py
# Acquire the jacobian and mass matrix tensors for all actors named "robot"
_jacobian = gym.acquire_jacobian_tensor(sim, "robot")
_massmatrix = gym.acquire_mass_matrix_tensor(sim, "robot")
# Wrap the tensor descriptors as PyTorch tensors
jacobian = gymtorch.wrap_tensor(_jacobian)
mm = gymtorch.wrap_tensor(_massmatrix)
# Refresh the jacobian and mass matrix tensors
gym.refresh_jacobian_tensors(sim)
gym.refresh_mass_matrix_tensors(sim)
```

> Mass matrix is available only in CPU pipeline.

The shape of the mass matrix tensor is `(num_envs, num_dofs, num_dofs)`.

The shape of the Jacobian tensor depends on the number of links, DOFs, and whether the base is fixed or not.

If the actor base is free to move (not fixed in place), the shape of the Jacobian tensor will be `(num_envs, num_links, 6, num_dofs + 6)`. The extra six DOFs (located at the beginning) correspond to the linear and angular degrees of freedom of the free root link.

An example to get the Jacobian for a specific link and DOF for a free-base actor:

```py
# Get link and DOF indices for the specified actor
link_dict = gym.get_asset_rigid_body_dict(robot_asset)
dof_dict = gym.get_asset_dof_dict(robot_asset)
link_index = link_dict["name_of_link"]
dof_index = dof_dict["name_of_dof"]
# Get the Jacobian for the specified link and DOF
jacobian[:, link_index, :, dof_index + 6]
```

If the actor base is fixed, the shape of the Jacobian tensor becomes `(num_envs, num_links - 1, 6, num_dofs)`.

An example to get the Jacobian for a specific link and DOF for a fixed-base actor:

```py
# Get link and DOF indices for the specified actor
link_dict = gym.get_asset_rigid_body_dict(robot_asset)
dof_dict = gym.get_asset_dof_dict(robot_asset)
link_index = link_dict["name_of_link"]
dof_index = dof_dict["name_of_dof"]
# Get the Jacobian for the specified link and DOF
jacobian[:, link_index - 1, :, dof_index]
```

## 2.5 Contact Tensor

The net contact force tensor contains the net contact forces experienced by each rigid body during the last simulation step, with the forces expressed as 3D vectors. It is a read-only tensor with shape `(num_rigid_bodies, 3)`.

```py
# Get the net contact force tensor descriptor
_net_cf = gym.acquire_net_contact_force_tensor(sim)
# Convert the descriptor to a PyTorch float32 tensor
net_cf = gymtorch.wrap_tensor(_net_cf)
# Refresh the net contact force tensor
gym.refresh_net_contact_force_tensor(sim)
```

> The values returned are affected by `SimParams.physx.contact_collection` and `SimParams.substeps`. Refer to the documentation.

## 2.6 Control Tensor

To manage actor behavior during simulation, we apply DOF forces or PD controls, or apply body forces directly.

### 2.6.1 DOF Control

Use `set_dof_actuation_force_tensor` to apply forces to the DOFs of all actors.

```py
# Create a force tensor
force_actions = 1.0 - 2.0 * torch.rand(num_dofs, dtype=torch.float32, device="cuda:0")
# Apply the forces
gym.set_dof_actuation_force_tensor(sim, gymtorch.unwrap_tensor(force_actions))
```

Use `set_dof_position_target_tensor` to set PD position targets for all DOFs in the simulation.

```py
pos_actions = 1.0 - 2.0 * torch.rand(num_dofs, dtype=torch.float32, device="cuda:0")
# Apply the position targets
gym.set_dof_position_target_tensor(sim, gymtorch.unwrap_tensor(pos_actions))
```

Use `set_dof_velocity_target_tensor` to set PD velocity targets for all DOFs in the simulation.

```py
vel_actions = 1.0 - 2.0 * torch.rand(num_dofs, dtype=torch.float32, device="cuda:0")
# Apply the velocity targets
gym.set_dof_velocity_target_tensor(sim, gymtorch.unwrap_tensor(vel_actions))
```

### 2.6.2 Body Forces

Use `apply_rigid_body_force_tensors` to apply forces and/or torques at the center-of-mass of all rigid bodies in the simulation.

```py
# Apply both forces and torques
gym.apply_rigid_body_force_tensors(sim, force_tensor, torque_tensor, gymapi.ENV_SPACE)
# Apply only forces
gym.apply_rigid_body_force_tensors(sim, force_tensor, None, gymapi.ENV_SPACE)
# Apply only torques
gym.apply_rigid_body_force_tensors(sim, None, torque_tensor, gymapi.ENV_SPACE)
```

Use `apply_rigid_body_force_at_pos_tensors` to apply forces and/or torques at a specific position.

```py
# Apply forces at given positions
gym.apply_rigid_body_force_at_pos_tensors(sim, force_tensor, pos_tensor, gymapi.ENV_SPACE)
```

# 3 Math Utilities

TODO

# 4 Sensors

TODO

## 4.1 Force Sensors

TODO

## 4.2 Camera Sensors

TODO

# 5 Terrains

TODO





<!-- CSS -->

