# 0 - Docker manual

### 1. Clone repository
```bash
git clone https://github.com/SerValera/ardupilot_docker.git
cd ardupilot_docker
```

### 2. Allow docker xhost for gui interfaces
``` bash
xhost +local:docker
```

### 3. Create an updated image (if Dockerfile have updates)

```bash
sudo docker-compose build
```

### 4. Create an instance of a container from image (!!! in 'ardupilot_docker' folder !!!):
``` bash
sudo docker run -it --privileged --ipc=host --net=host \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v ~/.Xauthority:/home/sim/.Xauthority \
-v ./:/home/sim/ardupilot_docker:rw \
-e DISPLAY=$DISPLAY -p 14570:14570/udp --name=ardupilot ardupilot_docker_drone_sim :latest bash
```


### 4. OR Create an instance of a container from image (in 'ardupilot_docker' folder):
``` bash
sudo docker run -it --privileged --ipc=host --net=host \
--gpus=all \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
--env="QT_X11_NO_MITSHM=1" \
-v ~/.Xauthority:/home/sim/.Xauthority \
-v ./:/home/sim/ardupilot_docker:rw \
--runtime=nvidia --gpus all \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-e DISPLAY=$DISPLAY -p 14570:14570/udp --name=px4 ardupilot_docker-drone_sim:latest bash
```

#### 5. To enter the docker use:
``` bash
docker exec -it ardupilot bash
```

### 6. In docker, run ONCE for ardupilot.
``` bash
cd ardupilot_docker
git clone https://github.com/intel/gazebo-realsense
cd 
copy_realsense_plugin
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
git submodule update --init --recursive
```

``` bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```


### 7. In docker. Plugin installation:
``` bash
cd
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

``` bash
sudo apt install libcanberra-gtk-module libcanberra-gtk3-module
sudo apt install dbus-x11
```

Open ~/.bashrc

``` bash
cd 
sudo nano ~/.bashrc
```

and paste following lines in the end of file

``` bash
export PATH="/usr/bin:$PATH"
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11/worlds
```

``` bash
source ~/.bashrc
```

[Next: Go to Start Simulation](1_sessions.md)