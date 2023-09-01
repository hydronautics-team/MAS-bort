FROM ubuntu:jammy
FROM gcc:latest
FROM ros:iron

SHELL ["/bin/bash", "-l", "-c"]

# Install dependencies

RUN apt update && apt install -y vim git tree curl libboost-all-dev clangd
RUN apt update && apt install -y qtbase5-dev libqt5serialport5-dev

# Setup project

RUN git clone https://github.com/dungineer/MAS-bort.git -b ros2-docker

WORKDIR MAS-bort

RUN git submodule init && git submodule update

# Build QT part

RUN qmake . && make

# Build ROS2 part

WORKDIR ros2

RUN rosdep update

RUN source /opt/ros/iron/setup.bash && ./build.sh

RUN echo "source /opt/ros/iron/setup.bash" >> /root/.bashrc
RUN echo "source $(realpath .)/install/setup.bash" >> /root/.bashrc

# Run launch command

ENTRYPOINT ["tail", "-f", "/dev/null"]

