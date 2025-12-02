# VNC

Install and setup the server:

```bash
# Install vncserver
sudo apt install x11vnc xvfb xterm

# Setup password
x11vnc -storepasswd
```

Attach to an existing display:

```bash
x11vnc -auth guess -forever -loop -noxdamage -repeat -rfbauth ~/.vnc/passwd -shared -rfbport 5900
```

Creating a virtual display (naive desktop):

```bash
x11vnc -display :20 -xrandr -create -env FD_PROG=/usr/bin/xterm -env X11VNC_FINDDISPLAY_ALWAYS_FAILS=1 -env X11VNC_CREATE_GEOM=1920x1080 -forever -rfbauth ~/.vnc/passwd -shared -rfbport 5900
```

Start with a desktop environment: 

```bash
sudo apt install xfce4-session
```

