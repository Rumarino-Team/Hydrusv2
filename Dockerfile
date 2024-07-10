# Use an official ROS image as a parent image
FROM ros:noetic-ros-core

# Install Python and detector node Dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    libgl1-mesa-glx \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

# Mission Node Dependencies
RUN apt update && apt install -y \
    ros-noetic-smach-ros \
    ros-noetic-executive-smach \
    ros-noetic-smach-viewer 

# ROS setup
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && \
    mkdir -p /home/catkin_ws/src && \
    cd /home/catkin_ws/ && \
    catkin_make && \
    source devel/setup.bash'
    
# Embedded Node Dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       gcc \
       curl \
       git  && \
    rm -rf /var/lib/apt/lists/*

# Install Arduino CLI and libraries
WORKDIR /usr/local/
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh && \
    arduino-cli core update-index && \
    arduino-cli core install arduino:avr
RUN arduino-cli lib install "Rosserial Arduino Library@0.7.9" && \
    sed -i '/#include "ros\/node_handle.h"/a #include "geometry_msgs/Vector3.h"' /root/Arduino/libraries/Rosserial_Arduino_Library/src/ros.h && \
    arduino-cli lib install "Servo@1.2.1" && \
    arduino-cli lib install "BlueRobotics MS5837 Library@1.1.1"

# Install additional Python packages using pip
RUN pip3 install opencv-python matplotlib

# Source ROS setup.bash script
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN /bin/bash -c "source /root/.bashrc"

# Install rosdep and update dependencies
RUN apt-get update && apt-get install -y python3-rosdep ros-noetic-rosserial-arduino && \
    rosdep init && rosdep update

# Install any ROS dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Copy embedded Arduino code
WORKDIR /root/Arduino/libraries
COPY ./embedded_arduino ./sensor_actuator_pkg

# Set the working directory
WORKDIR /home/catkin_ws
# Copy the rest of your application code
COPY . /home/catkin_ws/src

COPY ./ros-entrypoint.sh /home/catkin_ws/ros-entrypoint.sh
RUN chmod +x /home/catkin_ws/ros-entrypoint.sh

# # Build the catkin workspace
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Set the entrypoint
ENTRYPOINT ["./ros-entrypoint.sh"]
