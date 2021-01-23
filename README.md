* [install NVIDIA DRIVE™ OS 5.2.0 and DriveWorks 3.5 (Linux)](https://github.com/nvidia/dw-ros#install-nvidia-drive-os-520-and-driveworks-35-linux)
* [cross compile ROS](https://github.com/nvidia/dw-ros#cross-compile-ros)
* [cross compile nv_sensors](https://github.com/nvidia/dw-ros#cross-compile-nv_sensors)
* [run on the target system](https://github.com/nvidia/dw-ros#run-on-the-target-system)

## install NVIDIA DRIVE™ OS 5.2.0 and DriveWorks 3.5 (Linux)
follow the download [page](https://developer.nvidia.com/drive/downloads) to install NVIDIA DRIVE™ OS 5.2.0 and DriveWorks 3.5 (Linux). 

## cross compile ROS 
* prepare a cross-compilation sysroot

installing arm64 Debian dependencies with qemu-chroot

```
SYSROOT=~/nvidia/nvidia_sdk/DRIVE_OS_5.2.0_SDK_Linux_OS_DDPX/DRIVEOS/drive-t186ref-linux/targetfs
cd $SYSROOT

sudo apt install qemu-user-static
sudo cp /usr/bin/qemu-aarch64-static usr/bin
sudo cp -b /etc/resolv.conf etc
sudo mount -o bind /dev dev
sudo mount -o bind /proc proc
sudo mount -o bind /sys sys

sudo LC_ALL=C chroot .
# apt update
# apt install libboost-all-dev libtinyxml-dev libtinyxml2-dev liblz4-dev libbz2-dev libapr1 libaprutil1 libconsole-bridge-dev libpoco-dev libgpgme-dev python-defusedxml python-rospkg python-catkin-pkg python-netifaces liblog4cxx-dev
# exit

sudo umount sys proc dev 
sudo rm usr/bin/qemu-aarch64-static
sudo mv etc/resolv.conf~ etc/resolv.conf
sudo rm -rf var/lib/apt/lists/*
sudo rm -rf dev/*
sudo rm -rf var/log/*
sudo rm -rf var/tmp/*
sudo rm -rf var/cache/apt/archives/*.deb
sudo rm -rf tmp/*
```

* fix broken symlinks

The broken symlinks can be fixed temporarily with overlays, using commands similar to the following:
```
sudo mkdir /lib/aarch64-linux-gnu
sudo mkdir /tmp/ros-cc-overlayfs
sudo mount -t overlay -o lowerdir=$SYSROOT/lib/aarch64-linux-gnu,upperdir=/lib/aarch64-linux-gnu,workdir=/tmp/ros-cc-overlayfs overlay /lib/aarch64-linux-gnu
```

* follow this [page](http://wiki.ros.org/melodic/Installation/Source) to install all prerequisites and then run below commands to download source of ROS Melodic Morenia (Ubuntu 18.04 is the target root file system of DRIVE OS Linux 5.2.0)

```
mkdir -p ~/ros_catkin_ws/src && cd ~/ros_catkin_ws
rosinstall_generator ros_comm sensor_msgs --rosdistro melodic --deps --tar > melodic-ros_comm.rosinstall 
vcs import src < melodic-ros_comm.rosinstall
```

* invoke the cross compiler

create Toolchain-V5L.cmake in ~/Downloads with below content for cross compilation

```
set(CMAKE_SYSTEM_NAME Linux) 
# Specify the cross compiler 
set(TOOLCHAIN "$ENV{HOME}/nvidia/nvidia_sdk/DRIVE_OS_5.2.0_SDK_Linux_OS_DDPX/DRIVEOS/toolchains/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu") 
set(CMAKE_CXX_COMPILER "${TOOLCHAIN}/bin/aarch64-linux-gnu-g++") 
set(CMAKE_C_COMPILER "${TOOLCHAIN}/bin/aarch64-linux-gnu-gcc") 
# Targetfs path 
set(ROS_SYSROOT "$ENV{HOME}/nvidia/nvidia_sdk/DRIVE_OS_5.2.0_SDK_Linux_OS_DDPX/DRIVEOS/drive-t186ref-linux/targetfs") 
# Library paths 
set(LD_PATH "${ROS_SYSROOT}/usr/lib/aarch64-linux-gnu") 
set(LD_PATH_EXTRA "${ROS_SYSROOT}/lib/aarch64-linux-gnu") 
# setup compiler for cross-compilation 
set(CMAKE_CXX_FLAGS           "-fPIC"               CACHE STRING "c++ flags") 
set(CMAKE_C_FLAGS             "-fPIC"               CACHE STRING "c flags") 
set(CMAKE_SHARED_LINKER_FLAGS ""                    CACHE STRING "shared linker flags") 
set(CMAKE_MODULE_LINKER_FLAGS ""                    CACHE STRING "module linker flags") 
set(CMAKE_EXE_LINKER_FLAGS    ""                    CACHE STRING "executable linker flags") 
set(CMAKE_FIND_ROOT_PATH ${ROS_SYSROOT}) 
# Set compiler flags 
set(CMAKE_SHARED_LINKER_FLAGS   "--sysroot=${CMAKE_FIND_ROOT_PATH} -L${LD_PATH} -L${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} -Wl,-rpath,${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} -Wl,-rpath,${LD_PATH_EXTRA} ${CMAKE_SHARED_LINKER_FLAGS}") 
set(CMAKE_MODULE_LINKER_FLAGS   "--sysroot=${CMAKE_FIND_ROOT_PATH} -L${LD_PATH} -L${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} -Wl,-rpath,${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} -Wl,-rpath,${LD_PATH_EXTRA} ${CMAKE_SHARED_LINKER_FLAGS}") 
set(CMAKE_EXE_LINKER_FLAGS      "--sysroot=${CMAKE_FIND_ROOT_PATH} -L${LD_PATH} -L${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH} -Wl,-rpath,${LD_PATH_EXTRA} -Wl,-rpath,${LD_PATH_EXTRA} ${CMAKE_EXE_LINKER_FLAGS}") 
set(CMAKE_C_FLAGS "-fPIC --sysroot=${CMAKE_FIND_ROOT_PATH} -fpermissive -g" CACHE INTERNAL "" FORCE) 
set(CMAKE_CXX_FLAGS "-fPIC --sysroot=${CMAKE_FIND_ROOT_PATH} -fpermissive -g" CACHE INTERNAL "" FORCE) 
# Search for programs only in the build host directories 
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER) 
# Search for libraries and headers only in the target directories 
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY) 
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY) 

# set system default include dir
include_directories(BEFORE SYSTEM ${ROS_SYSROOT}/../include)
```
cross compile ROS

```
src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=~/Downloads/Toolchain-V5L.cmake 

```
## cross compile nv_sensors
fetch nv_sensors source
```
git clone https://github.com/NVIDIA/dw-ros.git
cd dw-ros
```
set up ROS environment
```
source ~/ros_catkin_ws/install_isolated/setup.bash
```
cross compile nv_sensors and install it to ROS directory copied from target
```
SYSROOT=~/nvidia/nvidia_sdk/DRIVE_OS_5.2.0_SDK_Linux_OS_DDPX/DRIVEOS/drive-t186ref-linux/targetfs
catkin_make_isolated -DCMAKE_TOOLCHAIN_FILE=$HOME/Downloads/Toolchain-V5L.cmake -DCMAKE_EXE_LINKER_FLAGS="${CMAKE_EXE_LINKER_FLAGS} -L/usr/local/driveworks/targets/aarch64-Linux/lib -Wl,-rpath,/usr/local/driveworks/targets/aarch64-Linux/lib -L$SYSROOT/usr/local/cuda-10.2/targets/aarch64-linux/lib -Wl,-rpath,$SYSROOT/usr/local/cuda-10.2/targets/aarch64-linux/lib" --install
```
replace with the binary installation directory so can run any binary installed packaes on the target. 
```
sed -i "s#$HOME/ros_catkin_ws/install_isolated#/opt/ros/melodic#g" install_isolated/_setup_util.py
```
copy built out binaries to your target system
```
scp -r install_isolated nvidia@<target>:~
```
## run on the target system
follow http://wiki.ros.org/melodic/Installation/Ubuntu to install ROS necessary apt packages via below command
```
sudo apt install ros-melodic-ros-base ros-melodic-image-view
```
install some packages
```
sudo apt install libboost-all-dev libtinyxml-dev libtinyxml2-dev liblz4-dev libbz2-dev libapr1 libaprutil1 libconsole-bridge-dev libpoco-dev libgpgme-dev python-defusedxml python-rospkg python-catkin-pkg python-netifaces liblog4cxx-dev
```
remove old version library (the detailed reason discussed in the forum [post](https://forums.developer.nvidia.com/t/libgdal-so-has-undefined-symbol/110239/5))
```
rm /usr/lib/libxerces-c*.

```
set up ros environment
```
source ~/install_isolated/setup.bash
```
run pre-requisite ros nodes and program 
```
roscore &
```
run camera sensor node advertising camera_start service
```
nv_sensors_producer
```
In anoter shell (also set up ros environment), enable live camera
```
rosservice call camera_start camera.gmsl "camera-name=SF3324,interface=csi-a,link=0,output-format=processed"
```
or replay recorded files
```
rosservice call camera_start camera.virtual "video=/usr/local/driveworks/data/samples/recordings/highway0/video_first.h264"
```
start X server
```
sudo -b X -ac -noreset -nolisten tcp
```
then verify on image receiving side by subscribing the topic
```
rosrun image_view image_view image:=/cameraData _autosize:=True
```
