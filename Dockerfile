#FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu20.04
FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04

ARG DISPLAY
ARG QT_X11_NO_MITSHM
ARG NVIDIA_DRIVER_CAPABILITIES

ENV DISPLAY=${DISPLAY}
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES}


# Minimal setup
RUN apt-get update && apt-get install -y locales lsb-release git 
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full \
 &&  apt-get update && apt-get install -y ros-noetic-rosbag 
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
SHELL ["/bin/bash", "-c"]
RUN source /root/.bashrc

# Install Autoware Mini
RUN mkdir -p /opt/catkin_ws/src
WORKDIR /opt/catkin_ws/src 

RUN mkdir -p /opt/catkin_ws/src/autoware_mini
COPY AUTOWARE_MINI /opt/catkin_ws/src/autoware_mini
# RUN git config --global http.postBuffer 10024288000
# RUN for i in {1..5}; do git clone https://github.com/UT-ADL/autoware_mini.git && break || sleep 5; done

RUN git clone https://github.com/UT-ADL/vehicle_platform.git
RUN git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git carla_ros_bridge
SHELL ["/bin/bash", "-c"]
RUN . /opt/ros/noetic/setup.bash
RUN apt-get install apt-utils -y 
RUN rosdep fix-permissions
RUN rosdep update -y 
ENV ROS_DISTRO=noetic
RUN apt install software-properties-common -y 
RUN add-apt-repository universe -y 
RUN apt update -y 
RUN apt install python2 -y 
RUN apt install python3-pip -y 

# Install scenario runner
RUN mkdir /opt/SCENARIO_RUNNER 
COPY SCENARIO_RUNNER /opt/SCENARIO_RUNNER 
RUN pip3 install -r /opt/SCENARIO_RUNNER/requirements.txt
RUN pip3 install pexpect

ENV SCENARIO_RUNNER_ROOT=/opt/SCENARIO_RUNNER 
RUN echo "export SCENARIO_RUNNER_ROOT=/opt/SCENARIO_RUNNER " >> /root/.bashrc
RUN echo "export PYTHONPATH=/opt/CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:/opt/CARLA_ROOT/PythonAPI/carla/agents:/opt/CARLA_ROOT/PythonAPI/carla" >> /root/.bashrc 

# Install autoware requirements
RUN ls /opt/catkin_ws/src/autoware_mini
RUN pip3 install -r /opt/catkin_ws/src/autoware_mini/requirements.txt

RUN apt-get update && rosdep update
RUN apt-get install -y python3-rosdep python3-rosinstall python3-vcstool python3-colcon-common-extensions
RUN rosdep install --from-paths . --ignore-src -r -y || echo

# Build the ROS workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
SHELL ["/bin/bash", "-c"]
RUN source /root/.bashrc
WORKDIR /opt/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

RUN echo "source /opt/catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN pip3 install --upgrade pip
RUN apt-get install -y wget 
WORKDIR /opt/
RUN wget https://files.pythonhosted.org/packages/ab/60/afc0c4486302e04195200079c6297b8075c5b7abc5b977d2db0399d0cb32/carla-0.9.13-cp38-cp38-manylinux_2_27_x86_64.whl
RUN pip3 install carla-0.9.13-cp38-cp38-manylinux_2_27_x86_64.whl 
RUN rm carla-0.9.13-cp38-cp38-manylinux_2_27_x86_64.whl
RUN mkdir CARLA_ROOT
COPY CARLA_ROOT /opt/CARLA_ROOT 
RUN echo "export CARLA_ROOT=/opt/CARLA_ROOT" >> /root/.bashrc 

# Install cuDNN for CUDA
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libcudnn8 libcudnn8-dev

# Install tmux for multi-terminal support
RUN apt install -y tmux 
WORKDIR /opt/catkin_ws
