FROM ubuntu
 
RUN apt-get update
RUN apt-get -y upgrade
RUN apt-get install -y git wget flex bison gperf cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
RUN apt-get install -y usbutils udev
 
RUN apt-get install -y python3 python3-pip python3-setuptools
RUN apt-get install -y lsb-core
 
# Install ROS2
# Set locale: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#set-locale
RUN apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
 
# Setup Sources: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#set-locale
RUN apt install -y software-properties-common
RUN add-apt-repository universe
RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
 
# Install ROS 2 packages
RUN apt-get update
RUN apt-get upgrade -y
# RUN run on startup
# RUN apt-get install -y ros-humble-desktop
# RUN apt-get install -y ros-dev-tools
# RUN apt install -y ~nros-humble-rqt*

RUN apt-get install nano -y

# Environment setup
RUN echo "export PROMPT_DIRTRIM=1" >> ~/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
RUN echo "export export LANG=en_US.UTF-8" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "alias nb='nano ~/.bashrc'" >> ~/.bashrc
RUN echo "alias sb='source ~/.bashrc'" >> ~/.bashrc