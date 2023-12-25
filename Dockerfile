FROM osrf/ros:noetic-desktop-full

RUN apt update \
    && apt install -y \
    libpq-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/taufik_ws
COPY ./src /home/taufik_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /home/taufik_ws && catkin_make"
RUN echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
RUN echo 'source /home/taufik_ws/devel/setup.bash' >> ~/.bashrc
RUN echo 'cd /home/taufik_ws' >> ~/.bashrc

COPY ./tm_entrypoint.sh /
RUN chmod +x /tm_entrypoint.sh

ENTRYPOINT [ "/tm_entrypoint.sh" ]
CMD [ "roslaunch", "movement_controller", "start.launch" ]