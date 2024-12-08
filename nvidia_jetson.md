Nvidia Jetson Note
===

> I'v gotten a Nvidia Jetson AGX Orin developer kit with its password forgotten, and try to deploy a dual-manipulator big model.
>
> Reference:
> 1. [Ubuntu for Jetson AGX manual](https://pages.ubuntu.com/rs/066-EOV-335/images/Ubuntu_22.04_for_NVIDIA_Jetson_Orin_Instructions.pdf?version=0&_gl=1*1ieoinb*_gcl_au*OTU3MjY1Mzk5LjE3MzAyODk1NDc.&_ga=2.180654244.205235902.1732520055-1792765167.1721727752)
> 2. [How to](https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/howto.html)
> 3. [Ubuntu for Jetson install](https://ubuntu.com/download/nvidia-jetson)
> 4. [Discussion in forum](https://forums.developer.nvidia.com/t/jetson-agx-orin-cant-turn-into-force-recovery-mode/231520/5)
> 5. [A blog in traditional Chinese](https://vmaker.tw/archives/60337)
> 6. [Install Pytorch on Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
> 
> Dknt 2024.11

# Hardware Description

> For Jetson AGX Orin kit only.

Put a Jetson AGX Orin on the table with the fan upward. There are 3 buttons on one side. The one closest to the LED is the Reset button, the middle one is the Force Recovery button and the left one is the Power button.

There is one DP port on one side of the kit, no HDMI port provided.

# Setup System with Jetson SDKManager

> This is the easiest way to setup a Jetson kit. There is no requirement to the installed system in the kit. Even if you have forgotten its password, you can use hardware method to enter in the recovery mode.

Install [Jetson SDKManager](https://developer.nvidia.com/sdk-manager).

Open the dev kit in recovery mode. The methods entering the recovery mode differ with every kit. First you should connect the host to the kit using the Type-C port next to the 40-pin header, then shutdown the kit. We used hardware recovery method. For Jetson AGX Orin, first press the Force Recovery button, hold, then press the Power button and release it while the Force Recovery button is still on-hold, release the Force Recovery button finally.

> Do not follow the instructions in the documentation. Use the **Power button**, instead of Reset button. See [this](https://forums.developer.nvidia.com/t/jetson-agx-orin-cant-turn-into-force-recovery-mode/231520/5)
> When recovery mode in entered, the fan will not spin and the monitor will not display anything. So just do the next step.

Open SDKManager, select Hardware Configuration as Target Hardware only, then choose your platform. Click next. Choose the packages you want to install, then choose `accept the terms`. Continue. Enter the password of **the host computer**.

In the next step, select the `Storage Device` as NVMe, since we have a 1T disk inserted in the kit. Click Flash, then wait for the processing finished.

Theoretically, this will install ubuntu desktop (22.04 arm for me) with NVIDIA driver, CUDA, etc.

> Jetson's GPU use DRAM as VRAM, like a integrated graphics. So when you type `nvidia-smi` in a terminal, you will get output with VRAM size equaling N/A (if you get nothing, your driver installation is fail).

# Deployment

> Use conda to create a virtual environment containing your packages.

To install **Pytorch** on jetson, you should follow the instructions on the [official site](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html), instead of using pip installation directly.

First, install `cusparselt`. Check your CUDA VERSION by entering in terminal `nvcc -V`. Then,

```shell
wget raw.githubusercontent.com/pytorch/pytorch/5c6af2b583709f6176898c017424dc9981023c28/.ci/docker/common/install_cusparselt.sh
export CUDA_VERSION=12.4
bash ./install_cusparselt.sh
```

Then we should specify the wheel URL. I have installed JetPack 6.1, corresponding to torch 2.5.0.

<span id="haha"></span>

```shell
export TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
python3 -m pip install --upgrade pip; python3 -m pip install numpy==1.26.1; python3 -m pip install --no-cache $TORCH_INSTALL
```

Try to run `python -c "import torch; print(torch.cuda.is_available())"`. If you get `True`, so far so good.

There is no pre-built `torchvision` for Jetson, so you should build it from source. Before that, check the torch and torchvision compatibility [here](https://pytorch.org/get-started/previous-versions/). Then you should clone the torchvision repository and switch back to a specific version.

```shell
git clone https://github.com/pytorch/vision
cd vision
git checkout v0.6.0
python setup install
```

Then try to run `python -c "import torchvision"`. You may get some error like missing `torchvision::nms`. You should add the directory where you install torchvision to `PYTHONPATH`.

```shell
export PYTHONPATH=<path-to-torchvision>:$PYTHONPATH
```

> You can add this in `~/.bashrc`.

This should solve the components missing problem.

Then comes another. You may find that your `torch.cuda.is_available()` failed after installing torchvision. This is be because another cpu-only torch is installed during installing torchvision. Do install torch again. See <a href="#haha">this instruction</a>.

`torch` and `torchvision` are both available now. (I hope so)

# Docker Support

> Do not contaminate your system environment.
> 
> Install user-built library inside your home directory (not in root directory). Try to use docker to solve dependence problem.

TODO

# Setup System in Terminal (Ignore this)

> Tried, succeeded with a lot of things to be done, such as dividing partition manually, installing and setting-up desktop environment and software... Then I give up and adopt Jetson SDKManager, which provides a much more friendly GUI to settle these stuffs.
> These notes are added here for reference.

Install boot firmware and Ubuntu image [here](https://ubuntu.com/download/nvidia-jetson) (Ubuntu Server 22.04).

Once the firmware and image are downloaded, run the following commands.

```shell
tar xf Jetson_Linux_R36.3.0_aarch64.tbz2 && cd Linux_for_Tegra/\
# Install the dependence
sudo ./tools/l4t_flash_prerequisites.sh
sudo apt install -y python3 cpp device-tree-compiler mkbootimg
```

Then we should put the dev kit into recovery mode. The methods entering the recovery mode differ with every kit. First we should connect to the kit using the Type-C port next to the 40-pin header, then shutdown the kit. One can enter the UEFI menu, check "Device Management/NVIDIA Configuratioin/Boot Configuration/Boot Into Recovery" option. Or some hardware recovery methods can be used. For Jetson AGX Orin, first press the Force Recovery button, hold, then press the Power button and release it while the Force Recovery button is still on-hold, release the Force Recovery button.

> Do not follow the instructions in the documentation. Use the **Power button**, instead of Reset button.
> When we enter the recovery mode, the fan will not spin and the monitor will not display anything.

After the recovery mode is entered, from the Linux_for_Tegra directory, enter the following command to program the latest QSPI boot firmware, it will then reboot the kit automatically upon success.

```shell
sudo ./flash.sh p3737-0000-p3701-0000-qspi internal
```

> This command is used for Jetson AGX Orin only.

Then we should program the Ubuntu image on an external boot media. Insert the media on the host, check name, then copy the image:

```shell
xzcat ubuntu-22.04-preinstalled-server-arm64+tegra-igx.img.xz | sudo dd of=<your-media-name> bs=16M status=progress
sudo sync
```

After that, insert the on the kit, then connect to the kit with keyboard, manually select the USB flash from BIOS.

> WTF: Following [this guider](https://pages.ubuntu.com/rs/066-EOV-335/images/Ubuntu_22.04_for_NVIDIA_Jetson_Orin_Instructions.pdf?version=0&_gl=1*1ieoinb*_gcl_au*OTU3MjY1Mzk5LjE3MzAyODk1NDc.&_ga=2.180654244.205235902.1732520055-1792765167.1721727752), you will have a system installed on a USB flash. Then you should `dd` the system to the disk, divide partitions through terminal, then connect to Internet and install all the dependence manually...
> So I give up, and turn to SDKManager.

**Check This** https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/IN/QuickStart.html.


<!-- CSS -->

<!-- Scripts -->
