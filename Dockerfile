# Use an official ROS image as a parent image
FROM ros:noetic-ros-core

# Install Python and other dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    libgl1-mesa-glx \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python packages using pip
RUN pip3 install opencv-python matplotlib

# Set the working directory
WORKDIR /home/catkin_ws

# Copy the rest of your application code
COPY . /home/catkin_ws

# Source ROS setup.bash script
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN /bin/bash -c "source /root/.bashrc"

# Install any ROS dependencies
RUN apt-get update && rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
