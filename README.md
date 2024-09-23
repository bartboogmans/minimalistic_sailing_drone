# minimalistic_sailing_drone
Files for a minimalistic robotic sailingboat

Concept is:
- to have a simple 3d printed hull
- 1 sail with actuator (reel with sheet)
- 1 actuated rudder
- wifi + ros2 compatible controller
- Probably powered by replacable batteries (e.g. some 9v, or AA) with some power regulation hardware

With optionally:
- Heading/ inertial measurement units (on mast?)
- Wind direction sensing

## Control system setup
- Raspberry pi 3 with ubuntu 24 server
- Connect to wifi. Enable SSH
- (optionally) Disable USB and HDMI for power saving
- Install ROS2 Jammy
- Install RPI GPIO libraries
```shell
sudo apt-get install python3-rpi.gpio
```

