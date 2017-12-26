# 說明 
----------
多台無人機formation control 。 
裡面有兩個package 分別為offb2 和 offb_self。
offb2目前還沒做好，實驗用的是offb_self。

### 執行環境 
----------
Ubuntu : 16.04

ROS : kinetic 

PX4 : 1.6.5  px4fmu-v2_lpe 

###編譯
----------
```
cd ~/catkin_ws/src

git clone https://pinxian@bitbucket.org/nctuncrl/formation_control.git

cd ~/catkin_ws && catkin_make

rosrun offb_self formation_four
```
請使用`catkin_make`進行編譯，`rosmake`會出錯。

如果編譯時出現找不到VRPN 或MAVROS的錯誤，請先安裝。
```
sudo apt-get install ros-kinetic-vrpn-client-ros

sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```

### 注意 
----------

使用前先確定每台無人機的狀況。
