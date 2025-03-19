Some Tricks I Hope Never be Useful
===

# Change The Resolution in Ubuntu20

Open the file `/etc/default/grub`

Fine the line

```
#GRUB_GFXMODE=640x480
```

Then edit it and remove the #

```
GRUB_GFXMODE=1920x1200
```

Then update

```
sudo update-grub
```

Then reboot the computer.

# VNC server config

Download:

```shell
tigervnc-standalone-server
```

Then run the command:

```shell
vncserver -localhost no :2
```

The vncserver will be available on `current_ip:5902`

# RustDesk

It's better to use RustDesk for remote desktop control. [rustdesk](https://rustdesk.com/)

# Nvidia Software Setting

## Nvidia GPU driver

Use 535 driver on ubuntu 20.04

Edit `/etc/modprobe.d/blacklist.conf`, add these lines:

```shell
blacklist nouveau     
options nouveau modeset=0
```

Save, then run this command:

```shell
sudo update-initramfs -u
```

Reboot, enter the following command to check if nouveau has been closed.

```shell
lsmod | grep nouveau
```

Enter a tty, then type command:

```shell
sudo service gdm3 stop
```

Run the .run file to install the driver.

Restart gdm3 service.

```shell
sudo service gdm3 start
```

## CUDA

Use CUDA version 11.8

CUDA 11.8 is installed with driver 520 by default, we can not follow the instructions on the official site, since it remove 535 and install 520.

We use cuda_VERSION.run installer to install CUDA. Remember choose not to install driver 520.

After the installation, follow the output of installer to set up PATH and LD_LIBRARY_PATH

Type `nvcc -V` to check CUDA.

## Cudnn

Using tar file installation.

Unpacking the file, enter the directory, copy the head files and libraries by following commands.

```shell
sudo cp -a include/cudnn*.h /usr/local/cuda/include
sudo cp -a lib/libcudnn* /usr/local/cuda/lib64
sudo ldconfig
```

Remember to add `-a` in `cp` to copy the links.

Then we can type the command to test the installation, and the output should not be empty.

```shell
ldconfig -p | grep cudnn
```

## TensorRT

Using tar installation.

The same as cudnn, copy the head files and libraries into /usr/local/cuda, and run ldconfig.

# Local Monitor Remote Control

Use `ssh` to control a server connected with a monitor. We want enter a command in the remote `ssh`, which open a window on the monitor.

To do this, enter the following command on the server's local session, replace the REMOTE_USERNAME to your username:

```shell
xhost +SI:localuser:REMOTE_USERNAME
```

Then on the remote ssh session:

```shell
export DISPLAY=:1
```

The display number may change, you can check that on the server's local session.

# Firewall in Ubuntu

`ufw` command is used to manage a firewall. It's easy to be understand with the help.

# Remote Session Control

Using command line to unlock a remote session. For instance, to use the monitor connected to a server, we need to unlock the local session first.

```shell
loginctl  # To check sessions
loginctl unlock-session [session-number]  # Unlock a specific session
loginctl unlock-sessions  # Unlock all sessions
```

The local session is bind to tty2, usually.

# IsaacSim and IsaacLab

Install IsaacSim 4.1.0 on Ubuntu 20 through omniverse-launcher.

`glibc` version on Ubuntu 20 is incompatible for pip IsaacLab installation, and IsaacLab on github lacks packages to run a demo.

IsaacLab version: 1.2.0

It's better to use IsaacSim with Ubuntu 22 or higher.

* So what to do with this next?

# Hide the Icons on the Desktop

```shell
gsettings set org.gnome.shell.extensions.desktop-icons show-trash false
gsettings set org.gnome.shell.extensions.desktop-icons show-home false
```

# Unlock Nvidia Graphics Power Limit

The power limit is set to 80W by default on my laptop with RXT 4090. Type these commands to unlock that:

```shell
sudo cp /usr/share/doc/nvidia-driver-550/nvidia-dbus.conf /etc/dbus-1/system.d/
sudo cp /usr/share/doc/nvidia-kernel-common-550/nvidia-powerd.service /lib/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable nvidia-powerd.service
sudo systemctl start nvidia-powerd.service
```

Then switch to the performance mode.

# Can Not Open Settings After Checking Users Panel on Ubuntu20

It's a wierd bug. Install `pipewire` to solve that.

```shell
sudo apt install pipewire
```

# DDNS Server

With the help of God Z. I'v build my first DDNS server.

Firstly, buy a domain. For instance, I brought dknt123.cn from tencent cloud.

Go to [dnspod site](https://console.dnspod.cn/) to add a record. Choose whichever subdomain, like www, hahaha, etc. Then set whichever value, which will be changed automatically later.

Go to API Keys page in AccountCenter, then create a new DNSPod key. Write the ID and token somewhere since they will never show again.

Config the `ardnsopd.sh` like this:

```shell
#!/bin/sh
# Change this to your path
. /home/dknt/Library/dnspod/ardnspod
# Combine your token ID and token together as follows
arToken="ID,token"  # For instance "123456,hahaha123456hahaha123456"

arIp4QueryUrl="http://ipv4.rehi.org/ip"
arIp6QueryUrl="http://ipv6.rehi.org/ip"
arErrCodeUnchanged=0

arLastRecordFile=/tmp/ardnspod_last_record
# Change to your domain subdomain and name of the network interface
arDdnsCheck "your_address.cn" "subdomain" 4 "card_name"
```

Run `ardnsopd.sh`, the IP should be changed.

Use this command to check IP on the server:

```shell
nslookup subdomain.your_address.cn
```

Then enable cron to call `ardnsopd.sh` periodically:

```shell
sudo apt install cron
sudo systemctl daemon-reload
sudo systemctl start cron
sudo systemctl enable cron

sudo vim /etc/crontab
```

Add this line:

```shell
*/5 * * * * root path_to_ddnspod.sh
```

Then run this command:

```shell
sudo crontab /etc/crontab
sudo crontab -l
```

The task should be listed.

# Web Terminal

> It's better to use a web terminal in Jupyter...

I really want to "touch fish" on some "water lecture" with my Android tablet, and I will be happy if I can access a terminal through http. Then I found [Wetty](https://github.com/butlerx/wetty).

To build Wetty on Ubuntu20, one needs to install a higher version of Node.js (14.x or 16.x). To do so, type these commands in terminal:

```shell
curl -sL https://deb.nodesource.com/setup_16.x | sudo -E bash -
sudo apt install -y nodejs
# Check node and npm version
node -v
npm -v
```

Then clone the project. Until 2024.10.11, Wetty v2.7.0 is the latest release version, but it can not work on Ubuntu20. So we choose v2.6.0.

```shell
git clone https://github.com/butlerx/wetty.git /home/dknt/Software/wetty
cd wetty
git checkout v2.6.0
npm install
npm run build
npm -g i wetty
```

Then, run wetty (on port 3000 by default):

```shell
wetty
```

Open login page [http://your_ip:3000/wetty](http://your_ip:3000/wetty) in the browser

# System Schedule Service

One can use `crontab` for this. Another method to enable system schedule service is `systemctl`.

We need to create one `.service` file to run a service and one `.timer` file to start a service.

Create a `example.service` under `/etc/systemd/system/`.

```shell
# example.service
[Unit]
Description=example service.
Wants=network.target
After=network.target

[Service]
User=user_name
WorkingDirectory=executable_path
ExecStart=executable_path_args
# ExecStart=/usr/bin/bash -c 'executable_path_args'  # To run executable in a terminal
Restart=on-failure  # or `no`
SyslogIdentifier=example.service

[Install]
WantedBy=multi-user.target
```

Then create a `example.timer` under `/etc/systemd/system/`.

```shell
# example.timer
[Unit]
Description=example timer.

[Timer]
OnBootSec=10sec
OnUnitActiveSec=1min
Persistent=false

[Install]
WantedBy=timers.target
```

Then run the following command to start the service and timer:

```shell
sudo systemctl daemon-reload
sudo systemctl enable example.service
sudo systemctl start example.service
sudo systemctl enable example.timer
sudo systemctl start example.timer
```

# VSCode in a Browser: `code-server`

Download `code-server`:

```shell
curl -fsSL https://code-server.dev/install.sh | sh
```

Then run it:

```shell
code-server
```

Change the config file located at `~/.config/code-server/config.yaml`

```shell
bind-addr: 0.0.0.0:8080   # Exposes code-server to all IP addresses
auth: password            # Secures the session with a password
password: your_password   # EDIT HERE. Set your password here
cert: false               # Set this to true if you have SSL certificates
```

The extensions of `code-server` are located at `~/.local/share/code-server/extensions`, we can copy extensions from another path.

Attention, browser `code-server` is different from ssh `code-server` in the desktop VSCode, and their extensions are in different places.

# lftp Usage

Connect to a sftp server:

```shell
lftp sftp://user:passward@server-ip
```

Or connect later:

```shell
open ftp://example.com
```

Download multiple files:

```shell
mget <file1> <file2> ...
```

Download a whole directory:

```shell
mirror <remote-directory> <local-directory>
```

Upload files:

```shell
mput <file1> <file2> ...
```

Upload a whole directory:

```shell
mirror -R <local-directory> <remote-directory>
```

# Intermediary SSH

Run this command on the local computer.

```shell
autossh -fN -R :22222:localhost:22 <public-server-ip>
```

Change the ssh config file on the cloud server:

```shell
sudo vim /etc/ssh/sshd_config
```

Allow the gateway ports by ensuring the following line exists or is set: `GatewayPorts yes`. Then restart the SSH service:

```shell
sudo systemctl restart ssh
```

Check the firewall setting on the cloud server.

Then open the firewall setting on the cloud server firewall [console](https://console.cloud.tencent.com/lighthouse/firewalltemplate). Create a new template `ALL` and apply it to the cloud server.

Then the local computer can be connected using:

```shell
ssh username@<public-server-ip> -p 22222
```

# Terminal and Fonts

Install fonts for user:

```shell
mkdir ~/.fonts
mv <font-file> ~/.fonts
fc-cache -f -v
```

# `zsh` Setup


