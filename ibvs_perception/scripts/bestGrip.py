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

if __name__ == '__main__':
	rospy.init_node('bestGrip', anonymous=True)
	pub_topic = rospy.get_param('/ibvs/grasp_topic', '/ibvs/best_grasp')
	pub = rospy.Publisher(pub_topic, GraspConfig)

	# Subscribe to the ROS topic that contains the grasps.
	grasps_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)

	# Wait for grasps to arrive.
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():    
	    rate.sleep()
