## Sensing-Intelligent-System-Project 

install Apriltag Library (project directory)
```
cd apriltag-lib/apriltags-cpp

mkdir build && cd build

cmkae ..

sudo make install 

```


To make sis-project
```
source ev.sh

catkin_make

```
### USB-CAMERA

`sudo apt-get install ros-kinetic-usb-cam`

### Mavros Packages for px4 Autopilot.
```
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-msgs ros-kinetic-mavros-extras
cd /opt/ros/kinetic/lib/mavros
sudo ./install_geographiclib_datasets.sh 
```

### Virtual Reality Peripheral Network. 
Before connecting to MOCAP System (Optitrack) ,you have to install VRPN library. 
```
install vrpn...
sudo apt-get install ros-kinetic-vrpn

connect to vrpn server ip (e.g 192.168.1.99).

roslaunch vrpn_client_ros sample.launch server:=192.168.1.99 

```



