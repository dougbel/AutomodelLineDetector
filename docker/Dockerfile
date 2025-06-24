FROM ros:noetic

# Set up workspace
RUN apt-get update && apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-vision-opencv \
    ros-noetic-rqt-image-view \
    python3-opencv \
    python3-catkin-tools \
    python3-pip \
    git \
    clang-format \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# sourcing ROS
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"

# Setup entrypoint
COPY docker/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

########################################       ADD NO ROOT USER
ARG USERNAME=docker
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME
# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME


RUN mkdir -p /tmp/runtime-docker
RUN chmod 700 /tmp/runtime-docker
RUN echo "export XDG_RUNTIME_DIR=/tmp/runtime-docker" >> ~/.bashrc


# Create workspace
ENV CATKIN_WS=/home/docker/catkin_ws

# sourcing user ROS packages on initiate commandline
RUN echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc 
WORKDIR $CATKIN_WS


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
