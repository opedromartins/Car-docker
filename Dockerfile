# Use the official ROS Noetic image as the base image
FROM osrf/ros:melodic-desktop-full

# Install required packages
RUN apt-get update && apt-get install -y \
    wget \
    git \
    nano \
    python3-catkin-tools \
    ros-noetic-socketcan-interface \
    ros-noetic-can-msgs \
    ros-noetic-teleop-twist-keyboard \
    python3-catkin-tools \
    can-utils \
    iproute2 && \
    rm -rf /var/lib/apt/lists/*

# Create a Catkin workspace and set it up
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN catkin init

# Clone the SD-VehicleInterface repository
RUN git clone https://github.com/opedromartins/SD-VehicleInterface.git /root/catkin_ws/src/SD-VehicleInterface

# Replace the ThreadedSocketCANInterfaceSharedPtr with the correct type for noetic
#RUN sed -i 's/can::ThreadedSocketCANInterfaceSharedPtr/std::shared_ptr<can::ThreadedInterface<can::SocketCANInterface>>/' /root/catkin_ws/src/SD-VehicleInterface/vehicle_interface/src/socketcan_bridge/socketcan_bridge_node.cpp

WORKDIR /root/catkin_ws/src
RUN catkin_create_pkg steering_control std_msgs rospy roscpp geometry_msgs
# Copy steering_control_node.cpp to src folder
COPY steering_control/steering_control_node.cpp /root/catkin_ws/src/steering_control/src
COPY steering_control/CMakeLists.txt /root/catkin_ws/src/steering_control

# Build the Catkin workspace
RUN catkin config --extend /opt/ros/noetic
RUN catkin build

# Source the ROS environment and the Catkin workspace in the entry point
# and create an alias for the keyboardlaunch script and the vehicle_interface launch file
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "alias vi_launch='roslaunch sd_vehicle_interface sd_vehicle_interface.launch'" >> /root/.bashrc && \
    echo "alias turn='rosrun steering_control steering_control_node'" >> /root/.bashrc

# Set the working directory to the root folder
WORKDIR /root