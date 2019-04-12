#!/usr/bin/env python
"""Republish the best grasp with properly oriented transform with respect to the robot
base.

See atenpas GPD package for more info on the GraspConfig msg.
"""

# ROS
import rospy
from geometry_msgs.msg import TransformStamped
from gpd.msg import GraspConfigList
from gpd.msg import GraspConfig
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros

# Sys
from math import pi
import numpy as np
import signal
import sys
import utils
from utils import R, axis_angle, signal_handler


class BestGrasps:
    def __init__(self):
        self.grasp_br = tf2_ros.TransformBroadcaster()
        self.grasp_br_trans = tf2_ros.TransformBroadcaster()

        self.camera_frame = rospy.get_param('/ibvs_perception/camera_frame',
                                            'camera_link')
        new_grasp_topic = rospy.get_param('/ibvs_perception/grasp_topic',
                                          '/ibvs_perception/best_grasp')
        self.grasp_pub = rospy.Publisher(new_grasp_topic, GraspConfig, queue_size=10)

        grasp_topic = '/detect_grasps/clustered_grasps'
        grasp_sub = rospy.Subscriber(grasp_topic, GraspConfigList, self._grasp_callback)

        rospy.loginfo("Parameters:\nGrasp Topic: {}\nCamera Frame: {}".format(
            grasp_topic, self.camera_frame))

        # Wait for grasps to arrive.
        rate = rospy.Rate(0.1)
        while grasp_sub.get_num_connections() == 0:
            rospy.logwarn_throttle(5, "Waiting for grasps from {}".format(grasp_topic))
            rate.sleep()

    def _grasp_callback(self, msg):
        grasps = msg.grasps
        if len(grasps) > 0:
            grasp = grasps[0]  # grasps are sorted in descending order by score
            self._grasp_to_tf(grasp)
            self.grasp_pub.publish(grasp)

    def _grasp_to_tf(self, grasp):
        tf_stamped = TransformStamped()
        tf_stamped.child_frame_id = 'grasp'
        tf_stamped.header.frame_id = self.camera_frame

        # Translation
        sample = grasp.sample
        tf_stamped.transform.translation.x = sample.x
        tf_stamped.transform.translation.y = sample.y
        tf_stamped.transform.translation.z = sample.z

        # Rotation
        x_grasp = np.array([grasp.axis.x, grasp.axis.y, grasp.axis.z])
        y_grasp = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
        z_grasp = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
        x_angle = axis_angle(-utils.Z_AXIS, x_grasp)
        y_angle = axis_angle(utils.Y_AXIS, y_grasp)
        z_angle = axis_angle(utils.X_AXIS, z_grasp)
        Qg = quaternion_from_euler(x_angle, y_angle, z_angle)
        Qg = quaternion_multiply(Qg, R('x', pi/2))
        tf_stamped.transform.rotation.x = Qg[0]
        tf_stamped.transform.rotation.y = Qg[1]
        tf_stamped.transform.rotation.z = Qg[2]
        tf_stamped.transform.rotation.w = Qg[3]

        # Stamp
        tf_stamped.header.stamp = rospy.Time.now()
        self.grasp_br.sendTransform(tf_stamped)

        return tf_stamped

    def start_sim(self):
        while not rospy.is_shutdown():
            try:
                rospy.spin()
            except KeyboardInterrupt, rospy.ROSInterruptException:
                sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    # rospy.init_node('bestGrip', log_level=rospy.DEBUG)
    rospy.init_node('bestGrip')
    bg = BestGrasps()
    bg.start_sim()
