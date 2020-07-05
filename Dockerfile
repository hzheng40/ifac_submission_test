FROM ros:melodic-robot-bionic

RUN apt-get update --fix-missing && \
    apt-get install -y \
    python-pip

RUN apt-get install -y vim \
                       ros-melodic-ackermann-msgs

RUN cp -r /usr/include/eigen3/Eigen /usr/include

RUN pip install numpy==1.16.0 \
                scipy==1.2.0 \
                pyyaml \
                llvmlite==0.31.0 \
                numba==0.47.0

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; mkdir -p catkin_ws/src"

COPY . /catkin_ws/src

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; cd catkin_ws; catkin_make"

CMD ["/catkin_ws/src/start.sh"]