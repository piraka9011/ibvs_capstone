#!/usr/bin/env python
# Select a grasp for the robot to execute.
import rospy
from gpd.msg import GraspConfigList
from gpd.msg import GraspConfig

grasps = [] # global variable to store grasps

def callback(msg):
    global grasps
    grasps = msg.grasps
    if len(grasps) > 0:
        grasp = grasps[0] # grasps are sorted in descending order by score
	print 'Selected grasp with score:', grasp.score
	rospy.loginfo(grasp)
	pub.publish(grasp)

pub = rospy.Publisher('detect_grasps/best_grasp', GraspConfig)
rospy.init_node('bestGrip', anonymous=True)

# Subscribe to the ROS topic that contains the grasps.
grasps_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)

# Wait for grasps to arrive.
rate = rospy.Rate(1)

while not rospy.is_shutdown():    
    rate.sleep()
