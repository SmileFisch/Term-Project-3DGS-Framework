FROM ros:humble-ros-base-jammy as builder

ENV ROS_WS=/root/ros_ws
#WORKDIR /home/orangepi/ros2_ws
WORKDIR $ROS_WS

COPY ./ src/mux

RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-pip && \
    pip install keyboard && \
    cd $ROS_WS && colcon build && \
    # Clean up unnecessary files
    apt-get remove --purge -y python3-pip && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# FROM ros:humble-ros-core-jammy
# ENV ROS_WS=/root/ros_ws
# WORKDIR ${ROS_WS}
# COPY --from=builder ${ROS_WS}/install ${ROS_WS}/install

CMD [ "/bin/bash" ,"-c", "source ${ROS_WS}/install/setup.bash && \
                          export ROS_DOMAIN_ID=30 && \
                          ros2 run mux mux_node" ]
