# VEX-ROS2

ROS2 packages managing the communication of the VEX and the ROS system and ROS module with template for neuromorphic neural network. 
This package has been developped under [ROS2 Galactic](https://docs.ros.org/en/galactic/index.html).

## Prerequisite

The package either requires the installation of [Docker Desktop](https://docs.docker.com/desktop/) or [Docker Engine](https://docs.docker.com/engine/) and [Docker Compose](https://docs.docker.com/compose/) (recommended) or the installation of ROS2 galactic and a full C++ and Python3 development environments with pyserial (for more expert users). 
It also requires the installation of a developpent environment for the VEX v5 brain, either the official [VEX VS Code Extension](https://www.vexrobotics.com/vexcode/vscode-extension) (recommended) or the official IDE [VEXcode V5](https://www.vexrobotics.com/vexcode/install/v5) (unsupported).

The docker package, due to link serial over usb link, is develloped on Linux (ubuntu), Windows should work using WSL2 and MacOS is untested.

## Usage 

We strongly recommend the use of our docker image containing an Ubuntu 20.04.6 with an installation of ROS2 galactic (based on a [community-made ROS2 galactic image](https://docs.ros.org/en/galactic/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)).

First, a local image must be build using the `initialize.sh` script.
```
bash initialize.sh
```

Then you can start the predefined launch sequence using
```
docker compose run -rm ros2_galactic
```
and get in a bash shell inside the container after some setup.

Or you can experiment with ROS2 and compiling modules by skipping the launch sequence using the command
```
docker compose run -rm ros2_galactic bash
```
to launch the container without setup. The ROS2 code is found in the `/home/VEX-ROS` directory.

To close the container just type
```
exit
```

## Compilation
The compilation is part of the automatic launch sequence, this section is only useful if you choose to skip it while launching the container or not use the provided docker image. 

You first need to compile the vex_message and vex_brain packages. 
```
colcon build --packages-select vex_message
colcon build --packages-select vex_brain
```

Then, before compiling the model package, the previous packages must be sourced, resulting in the command
```
. install/setup.bash
colcon build --packages-select model
```

Then the entirety of the packages can be sourced using the same command as before.

```
. install/setup.bash
```
## Running Vex program 

To launch the VEX program, compile the cpp code contaided in the vex_code directory and flash it to the brain. The code in the vex_code directory is structured for the VEX VS code extension.

## Running Launch 
For run the file launch :
```
ros2 launch ./vex_brain/launch/vex_launch.launch.py
```

The file launches the entire ROS-VEX system. To change dynamic initialization you must change the file vex_config.

## Running nodes from model 
Model node can be launch with the following command:

```
ros2 run model model
```
