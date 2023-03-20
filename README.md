# Repository for use of delphi ESR & SSR Radars
This repository includes instructions and tools for the use of the Autonomoustuff Delphi ESR & SSR radars availible in the laboratory. 
The connection with the computer is done via the Kvaser Leaf Light v2 CAN to USB interface.

## Installation
The following instructions are currently tested only on Ubuntu 18.04 and ROS melodic.

### Radar drivers
To make use of the radar, the Autonomous Stuff drivers must be installed. The following instructions were extracted from the [Driver Pack Installation Guide](https://autonomoustuff.atlassian.net/wiki/spaces/RW/pages/17475947/Driver+Pack+Installation+or+Upgrade+Instructions) from the manufacturer.


```sh
sudo apt update && sudo apt install apt-transport-https

sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'

sudo apt update
```
Once you've updated apt, you can install any of the ROS binary drivers with:

```sh
sudo apt install ros-$ROS_DISTRO-<driver_name>
```
Where <driver_name> is replaced with the ROS package name of the driver (e.g. delphi-esr, delphi-ssr).

### Kvaser Interface 
Autonomous stuff also provides the [Kvaser ROS Interface](https://github.com/astuff/kvaser_interface) as a standardized way to access Kvaser CAN devices from ROS. First install the dependencies:
```sh
sudo apt-add-repository ppa:astuff/kvaser-linux
sudo apt update
sudo apt install kvaser-canlib-dev kvaser-drivers-dkms
```
Next, install the `kvaser_interface`:

```sh
sudo apt install apt-transport-https

sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'

sudo apt update

sudo apt install ros-$ROS_DISTRO-kvaser-interface
```

The `list_channels` tool can be useful to get the kvaser interface information for the use of the different launch files and scripts. To run it, use the following command:
```sh 
rosrun kvaser_interface list_channels 
```

If the kvaser leaf is connected and working correctly, the output should include something as the following:
```
Card 0:
  S/N: 53615
  UPC: 73-30130-00685-0
  Name: Kvaser Leaf Light v2
  Firmware rev: v4.4.391
  Driver: leaf v8.38.841
  Channel 0:
    Index: 0
    Max Bitrate: 0
```

Here, the number following `S/N` is the **circuid hardware id** and the number following `Index` is the **can circuit id**. Both this parameters must be correctly set in the launch file of the radar for it to work properly.

## Launch files
Currently, the launch file used is tested for the delphi ESR radar. Note that the `use_kvaser` parameter must be set to true or the launch file will not be able to process the radar data.   
``` 
roslaunch delphi_esr delphi_esr.launch use_kvaser:=true
```
> __Note__: In order to visualize the data in Rviz without errors, it may be neccessary to run a tf static transform, this can be done with: 
>`rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map esr_1 50`
## Scripts 
Currently, the only script availible is a python node that converts the data from `radar_msgs/RadarTrackArray` to a `sensor_msgs/PointCloud` format by subscribing to the corresponding topic and publishing in a new one.