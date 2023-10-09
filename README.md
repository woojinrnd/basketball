# DEPENDENCIES
xsens mti driver  
dynamixel_sdk  

# INSTALL
## xsens mti driver  
#2-1. MT 소프트웨어 다운로드  
https://www.xsens.com/setup  

#2-2. 소프트웨어 설치  
좌측의 Xsens MTi  클릭 → 자동으로 MT_Software_Suite_linux_x64_2020.0.2 다운로드  
cd Downloads && sudo ./mtsdk_linux-x64_2020.0.2.sh  
sudo apt-get install sharutils  
default 경로 설정 : /usr/local/xsens  
cd /usr/local/xsens  

#2-3. catkin_ws 빌드 작업  
cd /usr/local/xsens  
sudo cp -r xsens_ros_mti_driver ~/catkin_ws/src  
cd ~/catkin_ws/src  
sudo chmod 777 -R xsens_ros_mti_driver  
cd  
pushd ~/catkin_ws/src/xsens_ros_mti_driver/lib/xspublic && make && popd  
catkin_make    
source devel/setup.bash  
roslaunch xsens_mti_driver display.launch  

#2-4. Permission Denied Error 해결  
ls -l /dev/ttyUSB0  
id  
sudo usermod -a -G dialout [username]  
* 컴퓨터를 재부팅해야 적용됨  

#3-2. ROS 노드 실행  
roslaunch xsens_mti_driver xsens_mti_node.launch  
rostopic echo /imu/data  

## dynamixel_sdk
sudo apt-get install ros-[version]-dynamixel-sdk


# BUILD
mkdir -p [what you want folder name]/[src]  
cd [what you want folder name]/[src]  
git clone https://github.com/woojinrnd/dynamixel_current_2port.git  
cd ..  
catkin_make  

# RUN
source devel/setup.bash  
split 3 terminal  
[terminal 1] roscore  
[terminal 2] rosrun dynamixel_current_2port Move_decision_node  
[terminal 3] rosrun dynamixel_current_2port dynamixel_current_2port  

# VSCODE
ctrl+shift+p --> Project Tree  
ctrl+shift+v --> README.md   

# ROS GRPAH
![0730(2)_rosgraph](https://github.com/woojinrnd/dynamixel_current_2port/assets/122770475/a4cfefc6-b8cb-490b-95da-91e7643174ce)

# Souce Tree
```
src
├── 0515_rosgraph.png
├── 0607_rosgraph2.png
├── 0607_rosgraph.png
├── 0621_rosgraph.png
├── 0721_rosgraph.png
├── 0730(2)_rosgraph.png
├── 0730_rosgraph.png
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── dynamixel_current_2port
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── callback.hpp
│   │   ├── dynamixel_controller.hpp
│   │   ├── dynamixel.hpp
│   │   ├── img_proc.hpp
│   │   ├── Move_decision.hpp
│   │   ├── sensor.hpp
│   │   └── Walkingpattern_generator.hpp
│   ├── launch
│   │   └── dynamixel_current_2port.launch
│   ├── package.xml
│   ├── src
│   │   ├── callback.cpp
│   │   ├── dynamixel_controller.cpp
│   │   ├── dynamixel.cpp
│   │   ├── fsr_ux_420_short.ino
│   │   ├── img_proc.cpp
│   │   ├── main_2.cpp
│   │   ├── main.cpp
│   │   ├── Move_decision.cpp
│   │   ├── Move_decision_node.cpp
│   │   ├── sensor.cpp
│   │   └── Walkingpattern_generator.cc
│   └── srv
│       ├── RL_NeckAngle.srv
│       ├── Select_Motion.srv
│       ├── Turn_Angle.srv
│       └── UD_NeckAngle.srv
└── README.md
```
