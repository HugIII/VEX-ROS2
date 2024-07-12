# VEX-ROS2

ROS2 packages managing the communication of the VEX and the ROS system and ROS module with template for neuromorphic neural network. 
This package has been developped under ROS2 Galactic (https://docs.ros.org/en/galactic/index.html).

## Compilation
You first need to compile the vex_message package before compiling the vex_brain one. 

```
colcon build --packages-select vex_message
```

```
colcon build --packages-select vex_brain
```

```
colcon build --packages-select model
```
## Running Vex program 

To launch the VEX program, download the code from the VEX application and launch it.
(https://www.vexrobotics.com/vexcode/install/v5)

## Running Launch 
For run the file launch :
```
ros2 launch ./src/launch/vex_launch.launch.py
```

The file launches the entire ROS-VEX system. To change dynamic initialization you must change the file vex_config.

## Running nodes 
Each node can be launched in the following way :

```
ros2 run camera "node_name"
```

With node_name being either "talker" , "event" or "frame". Talker is the node retrieving data from the camera and publishing them to the corresponding topics, while event and frame are the nodes reading data from those topics and displaying them.