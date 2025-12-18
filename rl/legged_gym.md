Setup Environment for `legged_gym`
===

# 0 Installation

Create a conda environment:

```bash
conda create -n legged-gym python==3.8
conda install pytorch torchvision torchaudio pytorch-cuda=12.4 -c pytorch -c nvidia
```

Install `isaac_gym`:

```bash
wget https://developer.nvidia.com/isaac-gym-preview-4
tar -xf IsaacGym_Preview_4_Package.tar.gz
cd isaacgym/python && pip install -e .
```

Install `rsl_rl` **v1.0.2**:

```bash
git clone https://github.com/leggedrobotics/rsl_rl
cd rsl_rl && git checkout v1.0.2 && pip install -e .
```

Install `legged_gym`:

```bash
git clone https://github.com/leggedrobotics/legged_gym
cd legged_gym && pip install -e .
```

## 0.1 Troubleshooting

1. ImportError: libpython3.8.so.1.0: cannot open shared object file: No such file or directory

Add path:

```bash
mkdir $CONDA_PREFIX/etc/conda/activate.d -p
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$CONDA_PREFIX/lib" >> $CONDA_PREFIX/etc/conda/activate.d/env_vars.sh
```

2. AttributeError: module `numpy` has no attribute `float`

Open `The file in which the error reported`, line 135, change `np.float` to `float`.

3. AttributeError: module `distutils` has no attribute `version`

Install a newer version of torch.

4. Error relating to `libstdc++.so.6`.

```bash
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

# 3 `rsl_rl` PPO

`VecEnv.step(actions) -> [observation, privileged_ovservation, reward, reset, extra_data]` Perform a step of simulation, in which the simulator is run for `decimation` steps.

In `rsl_rl`, the noise of action (stardard deviation) is `torch.Parameter` and is updated by the optimizer.

The learning rate is updated w.r.t. KL divergence. When KL divergence is too large, meaning the policy changes too fast, the learning rate is decreased. Otherwise, the learning rate is increased.

The total loss is the sum of three losses:

1. Surrogate loss. PPO clip loss.
2. Value function loss. This is a MSE loss.
3. Entropy bonus loss is used to balance exploration and exploitation.

Key functions in class `PPO`: `act()`, `process_env_step()`, `compute_returns()`, `update()`.

Key functions in class `RolloutStorate`: `add_trasitions()`, `compute_returns()`, `mini_batch_generator()`.

Key functions in class `OnPolicyRunner`: `learn()`, 

Key functions in class `ActorCritic`: `act()`, `get_actions_log_prob()`, `evaluate()`, `entropy()`.

Key functions in class `VecEnv`: `step()`.

# 4 Install Isaac Gym on 50-series GPUs

Need to manually build pytorch.

Clone the repository:

```bash
git clone https://github.com/pytorch/pytorch
cd pytorch
git checkout v2.3.1
git submodule update --init --recursive
```

Create a venv:

```bash
conda create -n isaacgym python=3.8
conda activate isaacgym
```

Change the following lines in `cmake/Modules_CUDA_fix/upstream/FindCUDA/select_compute_arch.cmake`, line 227:

```cmake
else()
  message(SEND_ERROR "Unknown CUDA Architecture Name ${arch_name} in CUDA_SELECT_NVCC_ARCH_FLAGS")
endif()
```

to:

```cmake
else()
  set(arch_bin 12.0)
  set(arch_ptx 12.0)
endif()
```

And `Dockerfile`, line 60:

```dockerfile
TORCH_CUDA_ARCH_LIST="3.5 5.2 6.0 6.1 7.0+PTX 8.0" TORCH_NVCC_FLAGS="-Xfatbin -compress-all" \
```

to

```dockerfile
TORCH_CUDA_ARCH_LIST="3.5 5.2 6.0 6.1 7.0+PTX 8.0 12.0" TORCH_NVCC_FLAGS="-Xfatbin -compress-all" \
```

Then run the following commands to build pytorch:


```bash
export USE_CUDA=1
export USE_NUMPY=1
export TORCH_CUDA_ARCH_LIST="8.0;8.6;8.9;9.0;12.0"
export MAX_JOBS=$(nproc)

python setup.py bdist_wheel
```

The built wheel file is located in `dist/`. Install it using pip:

```bash
pip install dist/torch-2.3.1-cp38-cp38-linux_x86_64.whl
```

Compatible versions of other packages:

```bash
pip install torchvision==0.18.1 torchaudio==2.3.1 numpy==1.23.5
```
