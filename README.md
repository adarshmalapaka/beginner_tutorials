[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---

# ROS2 Package with Publisher-Subscriber, Services, and tf2 broadcaster.

## Overview 

Inclusion of a tf2 broadcaster and Level 2 integration gtests in addition to the previously created ROS2 service server to modify the published message and ROS2 package containing the example publisher and subscriber scripts to publish a custom string message, whilst following the Google C++ Style Guide.

## Assumptions
* OS: Ubuntu Linux Focal (20.04) 64-bit
* ROS2 Distro: Humble Hawksbill
* ROS2 Workspace name: ros2_ws 
* ROS2 Installation Directory: ros2_humble

## ROS2 Dependencies
* ```ament_cmake```
* ```rclcpp```
* ```std_msgs```
* ```geometry_msgs```
* ```tf2```
* ```tf2_ros```
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

Note: Whenever you are instructed to open a terminal in the steps below, ensure you source the setup files from your ROS2 Humble installation, if built from source.
 
### Wk11: tf2 Broadcaster with Verification

In a terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 run beginner_tutorials talker
```

In another terminal, source your ROS workspace as mentioned before and run:
ros2 run tf2_ros tf2_echo world talk

Alternatively, in a other terminal, you may run the folwogin and open the corresponding saved file ```frames.pdf```:
```
ros2 run tf2_tools view_frames -o frames
```

### Wk11: Level 2 GTests

To view the gtest results for the ROS2 publisher and tf2 broadcaster, run the following in your ```ros_ws``` workspace,
```
colcon test --event-handlers console_direct+ --packages-select beginner_tutorials
```

### Wk11: Rosbag

To launch ROS2 bag and record all topics, run the following in a terminal sourced with your ROS workspace (```ros2_ws```) and change to the ```/results/bag_files``` directory:
```
ros2 launch beginner_tutorials rosbag_launch.py record_all_topics:=true
```

where, ```record_all_topics``` is an argument that enables bag recording when set to ```true```. Default value is ```true```.
The bag file is saved with the filename ```all_topics_bag_0.db3``` in the ```/results/bag_files/all_topics_bag``` directory, if recording is enabled. 

To verify ```/chatter``` messages were recorded, change to the ```bag_files/all_topics_bag``` directory as mentioned above and run:
```
ros2 bag info all_topics_bag
```

Alternatively, in one terminal run the ```listener``` using:
```
ros2 run beginner_tutorials listener
```

and in another terminal, play the recorded bag files, from the ```bag_files/all_topics_bag``` directory:
```
ros2 bag play all_topics_bag
```


### Publisher-Subscriber

In a terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 launch beginner_tutorials pubsub_service_launch.yaml
```

### Setting Parameters

The available parameters to change are:
* ```pub_freq```: double Frequency (in Hz) of the publisher node.
* ```queue_size```: double Queue size of the nodes (publisher and subscriber).

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

### Wk10: Service 

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

### Log Output

Run the following to colorize the log outputs while using launching the nodes:
```
export RCUTILS_COLORIZED_OUTPUT=1
```

To display a WARN log message:
```
ros2 launch beginner_tutorials pubsub_service_launch.yaml

# In another terminal
ros2 service call /update_message beginner_tutorials/srv/UpdateMessage "{data:}"
```

To display an ERROR log message:
```
ros2 launch beginner_tutorials pubsub_service_launch.yaml

# Press Ctrl+C
```

To display a FATAL log message:
```
ros2 launch beginner_tutorials pubsub_service_launch.yaml pub_freq:=-5.0

```

Note: Running the above launch file should automatically display DEBUG and INFO messages. 

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
