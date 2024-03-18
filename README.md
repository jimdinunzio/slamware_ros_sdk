# SLAMWARE ROS SDK for SLAMTEC Mapper M1M1 (EOL)

## Versions
* SLAMWARE ROS SDK 2.8.3-rtm
* Firmware release-2.7-sdp-E1M1-20200622, sdp_version: 2.7.0-dev (Jun 17 2020)

Ported to ROS2 Humble on Ubuntu 22.04 and tested with the now retired SLAMTEC Mapper M1M1 hardware.

## Build Requirements
GCC 7

## Build Steps
1. temporarily add this to your /etc/apt/sources.list
```
   # for gcc 
   deb [arch=amd64] http://archive.ubuntu.com/ubuntu focal main universe`
```
2. Run update

`sudo apt update`

3. Install g++-7

`sudo apt install g++-7`
