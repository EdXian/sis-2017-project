## Sensing-Intelligent-System-Project 

install Apriltag Library (project directory)
```
install cgal library ...
sudo apt-get install libcgal-dev libcgal-qt5-dev

cd apriltag-lib/apriltags-cpp

mkdir build && cd build

cmake ..

sudo make install 
```


To make sis-project
```
catkin_make

source ev.sh
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
Before connecting to MOCAP System (Optitrack) ,you have to install VRPN library first. 
```
install vrpn...
sudo apt-get install ros-kinetic-vrpn

connect to vrpn server ip (e.g 192.168.1.99).

roslaunch vrpn_client_ros sample.launch server:=192.168.1.99 

```
### Launch project
```
roslaunch usb_cam usb_cam-test.launch
roslaunch apriltags usb_cam_apriltags.launch 
rostopic echo /apriltags/detections
rosrun image_view image_view image:=/usb_cam/image_raw
```

## Hardware 
 1 Onboard Computer: UP Board
 2 Autopilot: PX4
 3 Web-cam : Logitech C310	
 3 Drone: DJI F450


