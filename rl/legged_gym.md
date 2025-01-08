Setup Environment for `legged_gym`
===

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

Install `rsl_rl` **v1.0.2**:

```shell
git clone https://github.com/leggedrobotics/rsl_rl
cd rsl_rl && git checkout v1.0.2 && pip install -e .
```

Install `legged_gym`:

```shell
git clone https://github.com/leggedrobotics/legged_gym
cd legged_gym && pip install -e .
```

## 0.1 Troubleshooting

1. ImportError: libpython3.8.so.1.0: cannot open shared object file: No such file or directory

Add path:

```shell
mkdir $CONDA_PREFIX/etc/conda/activate.d
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CONDA_PREFIX/lib" >> $CONDA_PREFIX/etc/conda/activate.d/env_vars.sh
```

2. AttributeError: module `numpy` has no attribute `float`

Open `<path-to-conda>/envs/legged-gym/lib/python3.8/site-packages/numpy/__init__.py`, line 135, change `np.float` to `float`.

3. AttributeError: module `distutils` has no attribute `version`

Install a newer version of torch.

4. Error relating to `libstdc++.so.6`.

```shell
conda install -c conda-forge libstdcxx-ng
```

# 1 Usage

# 2 Code Structure


Functions should be implemented in the class inherited form `BaseTask`:

**State**: root pose, DOF position, DOF velocity, contact force of each body.

**Observation** (input of actor network) `self.obs_buf`: 3-dim base velocity, 3-dim base angular velocity, 3-dim gravity, 3-dim input commands, 12-dim joint position, 12-dim joint velocity, 12-dim previous actions, 187-dim heights measurement. 235-dim in total. 

**Curriculum learning**: Start with easier tasks and gradually progress to more difficult ones.

**Height measurement**: Heights measurement are the height of the terrain.

**Actor creation**: When creating envs and actors, rigid shape properties, DOF properties and rigid body properties are processed.

Rigid shape properties: Randomizes the friction of each environment

DOF properties: Set position, velocity and torques limits form

Rigid body properties: Randomize base mass

The initial position of each actor is recorded in `self.env_origins`, which is defined in `_get_env_origins()` depending on terrain type.

Feet body indices, penalized contact body and termination contact indices are recorded during creation.



