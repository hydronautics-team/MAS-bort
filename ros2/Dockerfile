FROM ubuntu:jammy
FROM gcc:latest
FROM ros:iron

SHELL ["/bin/bash", "-l", "-c"]

# Install dependencies

RUN apt update && apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt install -y vim git tree curl libboost-system-dev \
                       qtbase5-dev libqt5serialport5-dev libwiringpi-dev

# Setup project

RUN git clone https://github.com/hydronautics-team/MAS-bort.git

WORKDIR /MAS-bort

RUN git submodule init && git submodule update

# Build QT part

RUN qmake . && make -j $(nproc)

# Build ROS2 part

WORKDIR /MAS-bort/ros2

RUN rosdep update

RUN source /opt/ros/iron/setup.bash && ./build.sh

RUN echo "source /opt/ros/iron/setup.bash" >> /root/.bashrc
RUN echo "source $(realpath .)/install/setup.bash" >> /root/.bashrc

WORKDIR /MAS-bort

# Run launch command

ENTRYPOINT ["tail", "-f", "/dev/null"]
