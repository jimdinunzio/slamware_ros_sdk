# SLAMWARE ROS SDK for SLAMTEC Mapper M1M1 (EOL)

## Versions
* SLAMWARE ROS SDK 2.8.3-rtm
* Firmware release-2.7-sdp-E1M1-20200622, sdp_version: 2.7.0-dev (Jun 17 2020)

Ported to ROS2 Humble on Ubuntu 22.04 x86_64 and tested with the now retired SLAMTEC Mapper M1M1 hardware.

## Build Environment
- On x86_64 use GCC 9
- On aarch64 (e.g. Raspberry PI) use GCC 9

### Build Steps
1. GCC 9 can be installed with 
```
   sudo apt install g++-9
```
4. Install other dependencies [aarch64]

`sudo apt install ros-humble-example-interfaces libc-ares-dev`

5. colcon build
