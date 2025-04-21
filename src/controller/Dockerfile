# Use a smaller base image if available
FROM ros:humble-ros-base-jammy as builder

ENV ROS_WS=/root/ros_ws
WORKDIR $ROS_WS

# Combine the COPY and RUN commands
COPY ./ src/controller
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-pip && \
    pip install keyboard && \
    cd $ROS_WS && colcon build && \
    # Clean up unnecessary files
    apt-get remove --purge -y python3-pip && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Start a new stage from a smaller base image if available
FROM ros:humble-ros-core-jammy
ENV ROS_WS=/root/ros_ws
WORKDIR $ROS_WS

# Install xboxdrv in the runtime image
RUN apt-get update && \
    apt-get install -y xboxdrv && \
    # Clean up after installation
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy only the necessary binaries and artifacts from the builder stage
COPY --from=builder $ROS_WS/install $ROS_WS/install

CMD [ "/bin/bash" ,"-c", "source ${ROS_WS}/install/setup.bash && \
                          export ROS_DOMAIN_ID=30 && \
                          ros2 run controller game_pad_node" ]
