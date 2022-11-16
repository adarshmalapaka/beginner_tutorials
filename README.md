[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---

# ROS2 Publisher/Subscriber 

## Overview 

Inclusion of a ROS2 service server to modify the published message with the previously created ROS2 package containing the example publisher and subscriber scripts to publish a custom string message, whilst following the Google C++ Style Guide. The nodes are run using a YAML based launch file with appropriate logging. 

## Assumptions
* OS: Ubuntu Linux Focal (20.04) 64-bit
* ROS2 Distro: Humble Hawksbill
* ROS2 Workspace name: ros2_ws 
* ROS2 Installation Directory: ros2_humble

## ROS2 Dependencies
* ```ament_cmake```
* ```rclcpp```
* ```std_msgs```
* ```ros2launch```
* ```rosidl_default_generators```

## ROS2 Installation (source)

The following steps walkthrough the procedure to install the lastest LTS version of ROS2 (Humble) on an Ubuntu 20.04 machine, from source code. These steps can be found in [this link](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html).

If your system is running Ubuntu Linux Jammy (22.04) 64-bit, you may skip to the binary installation of ROS2 Humble using 
[this link.](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Step-by-step instructions for installing ROS2 Humble and setting up the workspace can also be found in the ```ros_pub_sub``` branch of this repository. 

NOTE: The above procedure can take about 2+ hours to run. For a system with Intel Core i5 11th generation, it took nearly 6 hours.

### Environment Setup
```
. <path-to-ROS2-installation>/ros2_humble/install/local_setup.bash
```

### Clang
```
sudo apt install clang
export CC=clang
export CXX=clang++
colcon build --cmake-force-configure
```

### ROS2 Workspace
Here, an overlay workspace on top of the underlay installation workspace shall be created to place the custom-defined ROS2 packages. 
```
. <path-to-ROS2-installation>/ros2_humble/install/local_setup.bash
mkdir -p <path-to-ROS2-workspace>/ros2_ws/src
cd <path-to-ROS2-workspace>/ros2_ws/src
```
Source the 'underlay' installation workspace followed by the 'overlay',
```
. <path-to-ROS2-installation>/ros2_humble/install/local_setup.bash
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
```

## Build Instructions
```
cd <path-to-ROS2-workspace>/ros2_ws/src
git clone https://github.com/adarshmalapaka/beginner_tutorials.git
cd ..  
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select beginner_tutorials
```

## Run Instructions

### Publisher-Subscriber

In a terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 launch beginner_tutorials pubsub_service_launch.yaml
```

### Setting Parameters

The available parameters to change are:
* pub_freq: double Frequency (in Hz) of the publisher node.
* queue_size: double Queue size of the nodes (publisher and subscriber).

Syntax to launch the nodes with parameters: 
```
ros2 launch beginner_tutorials pubsub_service_launch.yaml <param1>:=<value> <param2>:=<value>
```

In a new terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 launch beginner_tutorials pubsub_service_launch.yaml pub_freq:=10.0
```

### Service 

In a new terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files. Launch the above publisher and subscriber nodes to run the server (Service: ```/update_message```) using, 
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 launch beginner_tutorials pubsub_service_launch.yaml
```
In another terminal navigate to your ROS2 workspace (```ros2_ws```) and source the setup files and run the client to the service (Service: ```/update_message```),
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 service call /update_message beginner_tutorials/srv/UpdateMessage "{data: Updated message to publish}"
```
Enter ```Ctrl+c``` in each terminal to stop the nodes from spinning.

## Results

### rqt_console

To view the log messages in ```rqt_console```, open a terminal and run:
```
ros2 run rqt_console rqt_console
```

Two screenshots of this window are provided in the ```/results``` directory.

### cpplint 
Change to the root directory of the package, ```/beginner_tutorials```, and run:
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp ./include/beginner_tutorials/*.hpp > ./results/cpplint.txt
```
The results of running ```cpplint``` can be found in ```/results/cpplint.txt```.

### cppcheck
Change to the root directory of the package, ```/beginner_tutorials```, and run:
```
cppcheck --enable=all --std=c++17 ./src/*.cpp ./include/beginner_tutorials/*.hpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
The results of running ```cppcheck``` can be found in ```/results/cppcheck.txt```.
