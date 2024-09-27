# VEX-ROS2

ROS2 packages managing the communication of the VEX and the ROS system and ROS module with template for neuromorphic neural network. 
This package has been developped under ROS2 Galactic (https://docs.ros.org/en/galactic/index.html).

## Usage 

We strongly recommend the use of docker by using an image containing an Ubuntu 20.04.6 with an installation of ROS2 galactic.

First, the container image must be pulled before being used.

```
docker pull osrf/ros:galactic-desktop
```

Then you can start the predefined launch sequence using
```
docker compose up -d
```
and close it using
```
docker compose down
```

You can experiment with ROS2 and launching modules using the command
```
docker compose run -rm ros2_galactic bash
```
to launch the container and put you in a bash shell inside the container.
To close the container just type
```
exit
```

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

## Running nodes from model 
Model node can be launch with the following command:

```
ros2 run model model
```
