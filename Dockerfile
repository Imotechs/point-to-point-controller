FROM osrf/ros:jazzy-desktop-full

# Install necessary tools and Nav2 packages
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    ros-jazzy-xacro \
    ros-jazzy-urdf-tutorial \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -c "import pyexpat; print('pyexpat module found')" && \
    sed -i "s/xml.parsers.expat/expat/" /opt/ros/jazzy/lib/python3.12/site-packages/xacro/__init__.py && \
    sed -i "s/import expat/import pyexpat as expat/" /opt/ros/jazzy/lib/python3.12/site-packages/xacro/__init__.py

RUN sed -i "1s/^/import pyexpat as expat\n/" /opt/ros/jazzy/lib/python3.12/site-packages/xacro/__init__.py

# Set working directory inside container
WORKDIR /ros_ws

# Copy your project folder (adjust path as needed)
COPY Point-to-Point-Controller/src/ /ros_ws/src/

# Source ROS and build your workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Auto-source the workspace and ROS on container start
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /ros_ws/install/setup.bash' >> /root/.bashrc

# Default command to keep the container running with bash shell
CMD ["bash"]
