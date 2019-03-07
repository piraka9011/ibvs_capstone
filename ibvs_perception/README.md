Steps to get running on Ubuntu 16.04.
You do not need to follow the Cuda instructions if running on CPU.

## Cuda
1. Download [Cuda 8.0](https://developer.nvidia.com/cuda-80-ga2-download-archive) using the runfile.
2. Use defaults except for installing the Nvidia drivers. You should do this from the settings panel instead under Software & Updates > Additional Drivers
3. Add the following to your `.bashrc`
```
export PATH=/usr/local/cuda-8.0/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-8.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
``` 
4. Download [cuDNN](https://developer.nvidia.com/rdp/form/cudnn-download-survey) and install it according to provided instructions (Your cuda path should be in `/usr/local/cuda-8.0`).

## Caffe
1. Install the following:
```
sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler libatlas-base-dev
sudo apt-get install --no-install-recommends libboost-all-dev
```
2. Install caffe using the [CMake Build](http://caffe.berkeleyvision.org/installation.html) instructions.
```
cp Makefile.config.example Makefile.config
```
Uncomment/change the following lines as needed:
```
# Line 5, only if not using CPU
USE_CUDNN := 1
# Line 8, only if using CPU
CPU_ONLY := 1
# Line 23
OPENCV_VERSION := 3
```

## GPD
1. Navigate to your ROS workspace and clone the grasp_pose_generator:
```
git clone https://github.com/atenpas/gpg.git
```
2. Build and install the grasp_pose_generator:
```
cd gpg
mkdir build && cd build
cmake ..
make
sudo make install
```
3. Navigate to your ROS workspace and clone GPD
```
git clone https://github.com/atenpas/gpd.git
```
4. Build your workspace
5. run ibvs_capstone/perception/setup.sh to transfer gpd launch files

## Realsense
1. Install Ros Realsense package
```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```
2. Navigate to your ROS workspace and clone/build the realsense package
```
git clone https://github.com/intel-ros/realsense.git
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
source ~/.bashrc
```
3. Run the bridge (roslaunch realsense2_camera rs_rgbd.launch )
4. Run gpd (roslauch gpd realsense.launch)

Perception Programs:

bestGrips.py - Takes the best gpd grip and publishes it

publishYolo.py - Grabs camera image and allows user to create a bounding box to simulate a Yolo match

publishCube.py - Takes a YoloObject message and creates a bounding cube message for it

Steps for Kinectv2
1. Install libfreenect2
2. Install iai_kinect2 bridge
