FROM christianfernandezlorden/vex-ros:galactic_pyserial 

RUN mkdir /home/VEX-ROS && cd /home/VEX-ROS && mkdir vex_brain && mkdir vex_message && mkdir model
