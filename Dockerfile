FROM px4io/px4-dev-ros-noetic AS base
ENV DEBIAN_FRONTEND=noninteractive
SHELL [ "/bin/bash", "-o", "pipefail", "-c" ]

ARG UNAME=sim
ARG USE_NVIDIA=1

# Dependencies
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
    python-is-python3 \
    apt-utils \
    libxext6 \
    libx11-6 \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libfuse-dev \
    fuse \
    git \
    libpulse-mainloop-glib0 \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \ 
    gstreamer1.0-gl \
    iputils-ping \
    nano \
    ros-noetic-rviz \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    python3-dev \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-pip \
    python3-matplotlib \
    python3-lxml \
    python3-pygame \
  && rm -rf /var/lib/apt/lists/*
  
# Python deps
RUN sudo pip install PyYAML MAVProxy

# User
RUN adduser --disabled-password --gecos '' $UNAME
RUN adduser $UNAME sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
ENV HOME=/home/$UNAME
USER $UNAME

# ROS vars
RUN echo "source /opt/ros/noetic/setup.bash --extend" >> ~/.bashrc && \
    echo "source /home/sim/ardupilot_docker/catkin_ws/devel/setup.bash --extend" >> ~/.bashrc && \
    echo "source /home/sim/ardupilot/Tools/completion/completion.bash --extend" >> ~/.bashrc && \
    echo "source /usr/share/gazebo-11/setup.bash --extend" >> ~/.bashrc
    
# Nvidia GPU vars
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
RUN if [[ -z "${USE_NVIDIA}" ]] ;\
    then printf "export QT_GRAPHICSSYSTEsudo docker imagesM=native" >> /home/${UNAME}/.bashrc ;\
    else echo "Native rendering support disabled" ;\
    fi

WORKDIR $HOME
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod a+x install_geographiclib_datasets.sh
RUN sudo ./install_geographiclib_datasets.sh

RUN sudo usermod -a -G dialout $UNAME

WORKDIR $HOME
RUN git clone https://github.com/ArduPilot/ardupilot.git
RUN echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

# Add terminal commands
RUN echo "alias copy_realsense_plugin='sudo cp /home/sim/ardupilot_docker/gazebo-realsense/gzrs/RealSensePlugin.hh /usr/include/gazebo-11/gazebo/plugins/'" >> ~/.bashrc && \
    echo "alias run='roslaunch drone_sim launch_world.launch'" >> ~/.bashrc && \
    echo "alias rec='roslaunch drone_sim record.launch'" >> ~/.bashrc