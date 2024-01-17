# Use the official ROS Noetic image as the base image
FROM osrf/ros:noetic-desktop-full

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

# Clone the SD-TwizyModel repository and the SD-VehicleInterface repository
RUN git clone https://github.com/alunos-pfc/TwizyModel-Noetic.git /root/catkin_ws/src/TwizyModel-Noetic
RUN git clone --single-branch -b melodic-devel https://github.com/streetdrone-home/SD-VehicleInterface.git /root/catkin_ws/src/SD-VehicleInterface

# Remove the msgs folder (duplicate of the can_msgs package)
RUN rm -r /root/catkin_ws/src/TwizyModel-Noetic/streetdrone_model/msgs/
# Replace the ThreadedSocketCANInterfaceSharedPtr with the correct type for noetic
RUN sed -i 's/can::ThreadedSocketCANInterfaceSharedPtr/std::shared_ptr<can::ThreadedInterface<can::SocketCANInterface>>/' /root/catkin_ws/src/SD-VehicleInterface/vehicle_interface/src/socketcan_bridge/socketcan_bridge_node.cpp

# Remove the TwizyModel-Noetic unnecessary packages
RUN rm -rf /root/catkin_ws/src/TwizyModel-Noetic/streetdrone_model/sd_description \
           /root/catkin_ws/src/TwizyModel-Noetic/streetdrone_model/sd_docs \
           /root/catkin_ws/src/TwizyModel-Noetic/streetdrone_model/sd_robot


# Build the Catkin workspace
RUN catkin config --extend /opt/ros/noetic
RUN catkin build

# Source the ROS environment and the Catkin workspace in the entry point
# and create an alias for the keyboardlaunch script and the vehicle_interface launch file
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "alias vi_launch='roslaunch sd_vehicle_interface sd_vehicle_interface.launch'" >> /root/.bashrc && \
    echo "alias control='rosrun sd_control sd_teleop_keyboard.py'" >> /root/.bashrc

# Change the default vehicle to twizy
RUN sed -i 's#default="env200"#default="twizy"#' /root/catkin_ws/src/SD-VehicleInterface/vehicle_interface/launch/sd_vehicle_interface.launch && \
    sed -i 's#<node pkg="sd_vehicle_interface" type="socketcan_bridge_node" name="socketcan_bridge_node" output = "screen"/>#<node pkg="sd_vehicle_interface" type="socketcan_bridge_node" name="socketcan_bridge_node" output = "screen"><param name="can_device" value="can0" /></node>#' \
    /root/catkin_ws/src/SD-VehicleInterface/vehicle_interface/launch/sd_vehicle_interface.launch

# Set the working directory to the root folder
WORKDIR /root