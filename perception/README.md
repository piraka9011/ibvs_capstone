TODO: Add fancy links and formating.

Steps to get running.

1. Intall GPD per instructions
2. Clone ibvs_capstone
3. add to rosdep
4. catkin_make
5. run ibvs_capstone/perception/setup.sh to transfer gpd launch files

Steps for Kinectv2
1. Install libfreenect2
2. Install iai_kinect2 bridge
3. Run the bridge (roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=1)
4. Run gpd (roslaunch gpd kinect2.launch)

Steps for Realsense
1. Install Ros Realsense package
2. Run the bridge (roslaunch realsense2_camera rs_rgbd.launch )
3. Run gpd (roslauch gpd realsense.launch)

Perception Programs:

bestGrips.py - Takes the best gpd grip and publishes it

publishYolo.py - Grabs camera image and allows user to create a bounding box to simulate a Yolo match

publishCube.py - Takes a YoloObject message and creates a bounding cube message for it
