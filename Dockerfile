FROM ros:iron-ros-base AS dspace_ros_base
# Adds all dspace specific dependencies to a ros humble base image
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["tail", "-f", "/dev/null"]

ENV LD_LIBRARY_PATH=/opt/VESI/lib
ENV SIM_CLOCK_MODE=false
ENV ENABLE_LOG=false
ENV ROS_DISTRO=iron
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && \
    apt-get install -y --no-install-recommends openssh-server xauth build-essential libboost-all-dev python3-colcon-common-extensions git cmake zip g++ software-properties-common gdb wget python3-pip debconf python3 python3-setuptools ros-$ROS_DISTRO-rmw-cyclonedds-cpp

RUN rosdep update && \
    echo 'source /opt/ros/$ROS_DISTRO/local_setup.bash' >> /root/.bashrc

RUN mkdir -p /opt/VESI/lib 
COPY sut-te-bridge/ros2_bridge_ws/src/sut_te_bridge/include/V-ESI-API/lib/linux/libVESIAPI.so /opt/VESI/lib/

# Setting up CycloneDDS, using the CycloneDDS XML configuration file
RUN mkdir -p /etc/cyclonedds && \
    echo 'export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml' >> /root/.bashrc
COPY cyclonedds.xml /etc/cyclonedds/cyclonedds.xml

RUN ldconfig

FROM dspace_ros_base AS sut-te-bridge_base
# Build bridge message definitions to enable dev working environment
RUN mkdir -p /root/ros_ws_aux
COPY ros_ws_aux /root/ros_ws_aux

RUN rm -rf /root/ros_ws_aux/src/external/raptor_dbw_msgs/ct_interface/ && \
    rm -rf /root/ros_ws_aux/src/raptor_dbw_msgs && \
    source /opt/ros/$ROS_DISTRO/local_setup.bash && \
    rosdep install -i --from-path /root/ros_ws_aux/src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --base-paths /root/ros_ws_aux --build-base /root/ros_ws_aux/build --install-base /root/ros_ws_aux/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    echo 'source /root/ros_ws_aux/install/local_setup.bash' >> /root/.bashrc

FROM sut-te-bridge_base AS sut-te-bridge_dev
# Prepares image for an SUT to be mounted into the container during runtime
RUN mkdir -p /root/runtime_scripts && \
    mkdir -p /root/record_log
COPY sut-te-bridge/runtime_scripts /root/runtime_scripts

WORKDIR /root/runtime_scripts

ENTRYPOINT ["/bin/bash"]

FROM sut-te-bridge_base AS sut-te-bridge_simphera
# Build bridge node and enable autostart for headless execution
COPY sut-te-bridge/ros2_bridge_ws /root/ros2_bridge_ws
RUN mkdir -p /root/record_log && \
    source /opt/ros/$ROS_DISTRO/local_setup.bash && \
    source /root/ros_ws_aux/install/local_setup.bash && \
    rosdep install -i --from-path /root/ros2_bridge_ws/src --rosdistro $ROS_DISTRO -y && \
    colcon build --symlink-install --cmake-clean-first --base-paths /root/ros2_bridge_ws/ --build-base /root/ros2_bridge_ws/build --install-base /root/ros2_bridge_ws/install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    echo 'source /root/ros2_bridge_ws/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["sh", "-c", ". /opt/ros/$ROS_DISTRO/local_setup.sh && . /root/ros_ws_aux/install/local_setup.sh && . /root/ros2_bridge_ws/install/local_setup.sh && ros2 run sut_te_bridge SutTeBridgeNode --ros-args -p use_sim_time:=$SIM_CLOCK_MODE"]

FROM sut-te-bridge_base AS sut-te-bridge_foxglove
# Install Foxglove bridge 
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-$ROS_DISTRO-foxglove-bridge

ENTRYPOINT ["sh", "-c", ". /opt/ros/$ROS_DISTRO/local_setup.sh && . /root/ros_ws_aux/install/local_setup.sh && ros2 launch foxglove_bridge foxglove_bridge_launch.xml"]
