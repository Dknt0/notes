Setup Environment for `legged_gym`
===

# 0 Installation

Create a conda environment:

```shell
conda create -n legged-gym python==3.8
pip install torch==2.4.1 torchaudio==2.4.1 torchvision==0.19.1
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
export LD_LIBRARY_PATH=<path-to-conda>/envs/legged-gym/lib:$LD_LIBRARY_PATH
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



