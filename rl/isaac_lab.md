Isaac Lab Note
===

# 1 Installation

> Refer to [Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html) for more information.

We install `isaac_lab` and `isaac_gym` on an Ubuntu system higher than 22.04. To install on Ubuntu 20.04, you should use binary installation.

## 1.1 Install Isaac Sim

First, create a conda environment:

```shell
conda create -n env_isaaclab python=3.10
conda activate env_isaaclab
```

Next, install `pytorch` with the correct CUDA version:

```shell
pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cu121
```

Update `pip`:

```shell
pip install --upgrade pip
```

Then, install Isaac Sim:

```shell
pip install 'isaacsim[all,extscache]==4.5.0' --extra-index-url https://pypi.nvidia.com
```

## 1.2 Install Isaac Gym

Clone the Isaac Lab repository. 

```shell
git clone git@github.com:isaac-sim/IsaacLab.git
```

Run the helper script to install Isaac Gym:

```shell
./isaaclab.sh --install
```

