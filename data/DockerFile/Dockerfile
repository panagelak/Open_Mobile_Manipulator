FROM ros:kinetic-ros-base

RUN apt-get update && apt-get install -y \
    apt-utils \
    libfreenect-dev \
    ros-kinetic-freenect-* \
    && echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc \
    && rm -rf /var/lib/apt/lists/
RUN apt-get update -y && apt-get upgrade -y
CMD ["roslaunch","freenect_launch","freenect.launch", "depth_registration:=true", "publish_tf:=false"]

