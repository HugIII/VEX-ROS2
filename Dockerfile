#FROM christianfernandezlorden/vex-ros:galactic_pyserial
#FROM osrf/ros:galactic-desktop
FROM osrf/ros:humble-desktop
ENV ROS_DISTRO=humble

# Install pyserial from source (dont install pip) and then remove source
RUN cd / && mkdir /pyserial_install && cd pyserial_install \
 && curl -O https://files.pythonhosted.org/packages/1e/7d/ae3f0a63f41e4d2f6cb66a5b57197850f919f59e558159a4dd3a818f5082/pyserial-3.5.tar.gz \
 && tar -xvzf pyserial-3.5.tar.gz \
 && cd pyserial-3.5 && python3 setup.py install \
 && cd / && rm -rf /pyserial_install

# Change RMW to cyclonedds
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ADD username argument
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Install extra tools for development
RUN apt-get update && apt-get install -y --no-install-recommends gdb gdbserver nano

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${UID} --gid ${GID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} && mkdir /VEX-ROS

# Remove
RUN sed -i '/.*exec "\$@".*/d' /ros_entrypoint.sh
# Add the entrypoint script, make it executable and make it the starting script
COPY ./entrypoint.sh /VEX-ROS/entrypoint.sh
# Make file executable ans change ownerchip of home and VEX-ROS directories
RUN chmod +x /VEX-ROS/entrypoint.sh && chown -R ${UID}:${GID} /home/${USERNAME} && chown -R ${UID}:${GID} /VEX-ROS
# Change current user
USER ${USERNAME}
#RUN echo "source ./entrypoint.sh" >> /home/${USERNAME}/.bashrc
WORKDIR /VEX-ROS

ENTRYPOINT [ "./entrypoint.sh" ]
