FROM christianfernandezlorden/vex-ros:galactic_pyserial 

RUN mkdir /home/VEX-ROS && cd /home/VEX-ROS && mkdir vex_brain && mkdir vex_message && mkdir model
COPY entrypoint.sh /home/VEX-ROS/entrypoint.sh
RUN chmod +x /home/VEX-ROS/entrypoint.sh

