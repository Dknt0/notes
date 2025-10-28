Docker Note
===

> To deploy applications developed in different environments.

# 0 Installation

> Reference:
> https://docs.docker.com/engine/install/ubuntu/

Install Docker Engine. Docker Desktop may not support X11 forwarding.

## 0.1 Docker Engine

Docker Engine is a CLI version of Docker containing all tools we need. Install Docker Engine.

```shell
# Remove old versions
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
```

Setup docker apt repository.

```shell
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

Install docker engine.

```shell
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Rootless mode.

```shell
sudo usermod -aG docker $USER
# Logout and check
```

## 0.2 Docker Desktop

> Docker Desktop may can not use X11 forwarding!

> Reference:
> https://docs.docker.com/desktop/install/linux/

Check and set KVM virtualization support. If the docker can not be run without sudo, one should check this.

Load `kvm` module if it is not loaded automatically.

```shell
modprobe kvm
modprobe kvm_intel
```

Then check it.

```shell
kvm-ok
lsmod | grep kvm
```

Check and set KVM device user permissions.

```shell
# Check ownership
ls -al /dev/kvm
# Set user to the kvm group
sudo usermod -aG kvm $USER
```

Set up Docker's repository.

```shell
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

Download the Docker deb package [here](https://desktop.docker.com/linux/main/amd64/docker-desktop-amd64.deb?utm_source=docker&utm_medium=webreferral&utm_campaign=docs-driven-download-linux-amd64&_gl=1*1h0pdfz*_gcl_au*MjEyMzY3NjQ4LjE3MzAzNDA2MTY.*_ga*MjIzMDI4OTAxLjE3MzAzNDA2MTY.*_ga_XJWPQMJYHQ*MTczMDM0MDYxNS4xLjEuMTczMDM0MTc3OC4xMy4wLjA.), then install it.

```shell
sudo apt-get update
sudo apt-get install ./docker-desktop-amd64.deb
```

To sign in to Docker, `pass` should be initialized before.

```shell
gpg --generate-key
```

My key is saved in `/home/dknt/.gnupg/openpgp-revocs.d`.

Then one can use the `id_public_key` to initialize `pass`:

```shell
pass init <your_generated_gpg-id_public_key>
```

# 1 Docker Basis

Docker provides the ability to package and run an application in a loosely isolated environment called a **container**. Containers are lightweight and contain everything needed to run the application.

Docker architecture is illustrated in the image below. It has a client-server architecture. The Docker client and daemon can run on the same system or separate.

<div class="center-image">
  <img src="image/docker-architecture.webp" width="800px" />
</div>

When we use Docker commands like `docker run`, the client sends the commands to the daemon `dockerd`.

Docker Desktop includes Docker daemon, Docker client, Docker Compose, etc.

Docker registry stores Docker images. Docker Hub is a public registry.

An **image** is a read-only template with instructions for creating a Docker container. Typically, one image is based on another image. To build an image, a Dockerfile should be created. Each instruction in a Dockerfile creates a layer in the image.

A **container** is a runnable instance of an image. User can control how isolated a container's network, storage, or other underlying subsystems are from other containers or from the host machine.

For instance, use the following command to run an `ubuntu` container, attach interactively (-i interactively) to the local command-line session (-t terminal), and run /bin/bash. (Make sure the daemon is running before run the command.)

```shell
docker run -i -t ubuntu /bin/bash
```

Commonly used commands:

```shell
# Pull a image from registry
docker pull image_name
# Create a new container
docker container create
```

# 2 Usage

## 2.1 ROS Noetic environment

```bash
docker pull ubuntu:20.04
docker run -it --name ros_noetic ubuntu:20.04 /bin/bash
```

Then install ROS Noetic following the official site.

To run the docker container with X11 forwarding, we need to allow access to the X server.

```shell
xhost +local:docker
```

Then use a shell with X11 forwarding:

```shell
docker run -it -p 22222:22 --env DISPLAY=:0 --privileged --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v /tmp/.X11-unix:/tmp/.X11-unix image_name
```

> Using `-p 22222:22` to export a port. Attention, the port mappings are specified at the time the container is created, and they cannot be modified afterwards.

> Ths `DISPLAY` may be set to `:0`, `:1` or `:0.0`

One can use nvidia graphics in a docker container. To do this, the [NVIDIA container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) should be installed.

Then, the image should be created with:

```shell
docker run --runtime=nvidia --gpus all -it -p 22222:22 --env DISPLAY=:0 --privileged --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v /tmp/.X11-unix:/tmp/.X11-unix image_name
```

One can then save the environment in an image.

```bash
# Save container to image
docker commit ros_noetic_container my_ros_noetic_image
# Save the image as a file
docker save -o image_name.tar image_name
# (Optional) Compress
pigz image_name.tar
# (Optional) Decompress
pigz -d -c image_name.tar.gz
# Load an image
docker load -i image_name.tar
```

To create a user with sudo permission:

```bash
sudo usermod -aG sudo user_name
```

The ssh may not be started by default, we should start the service manually.

```bash
sudo service ssh start
```

Useful command:

```bash
# Check available docker images
docker images
# Connect to an existing docker container
docker start <container_name_or_id>
docker exec -it <container_name_or_id> /bin/bash
```

### 2.1.1 Nvidia Support

Nvidia Support:


Then run the container with GPU support:

```bash
docker run --gpus all -it --runtime=nvidia -p 22222:22 --env DISPLAY=:0 --privileged --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v /tmp/.X11-unix:/tmp/.X11-unix --network host nvidia/cuda:11.8.0-cudnn8-runtime-ubuntu20.04 /bin/bash
```

Install ROS in a container.

```shell
apt update
apt install -y curl git vim wget build-essential unzip zip tmux net-tools iputils-ping lsb-release gnome-mines htop

# Setup environment variable
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# GL
sudo apt install nvidia-prime
sudo prime-select nvidia
```

There is a ROS Noetic container with everything we need, including ROS, gpu-based-opengl.

```bash
docker run -it --privileged --runtime=nvidia --network bridge -p 22222:22 --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev --name noetic_nvidia osrf/ros:noetic-desktop-full /bin/bash
```

**Run This One**

```bash
docker run -it --privileged --runtime=nvidia --network bridge -p 22222:22 --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev --name limx dknt/limx-image:latest /bin/bash
```

```bash
docker run -itd --network bridge \
--name=bitbot_gz \
--user ivan \
--privileged \
--gpus all \
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/etc/localtime:/etc/localtime:ro" \
-v /dev/bus/usb:/dev/bus/usb \
--device=/dev/dri \
--group-add video \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--env="DISPLAY=$DISPLAY" \
bitbot_gz_image:latest \
/bin/bash
```


```bash
sudo apt-get install '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev
```

If the network of a container with bridge mode is not working, you may need to enable the ipv4 forwarding.

```shell
sudo sysctl -w net.ipv4.ip_forward=1
# Check the value
sudo sysctl net.ipv4.ip_forward
```

Then restart the docker service.

```shell
sudo systemctl restart docker
```

### 2.1.2 Upload Image to Docker Hub

```shell
# Login
docker login
# Tag an image
docker tag limx-image:latest dknt/limx-image:first
# Push the image to Docker Hub
docker push dknt/limx-image:first
```

## 2.2 Nvidia Container Toolkit

It is recommended to use the NVIDIA Container Toolkit to access the GPU on the host machine. Installation:

```shell
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
```

Configuration for Docker:

```shell
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

## 2.3 Wayland Forwarding

Install dependencies on the host system:

```bash
sudo apt install libwayland-client0 libwayland-egl1 libwayland-server0
```

Install `xwayland` on the host.

```bash
sudo apt-get install xwayland
```

```bash
docker run -it --privileged --runtime=nvidia --network bridge -p 22222:22 --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all  -e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:/home/ivan/.Xauthority:ro -v /dev:/dev --device /dev/dri --group-add video --name limx dknt/limx-image:first

# Install 
sudo apt install xwayland
```

The following command still use X11...

```bash
docker run -it --privileged --runtime=nvidia --network bridge -p 22222:22 --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all  -e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:/home/ivan/.Xauthority:ro -v /dev:/dev --device /dev/dri --group-add video --name limx dknt/limx-image:first
```

On the host machine:

```shell
xhost +
```

## _ Start a docker project

```shell
docker compose watch
```

<!-- CSS class -->
<style>
  .center-image {
    display: flex;
    justify-content: center;
    align-items: center;
  }

  .center-image img {
    max-width: 100%;
    max-height: 100%;
    background-color: #FFFFFFAA;
    padding: 10px;
  }
</style>
