#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import TransformStamped
from gpd.msg import GraspConfigList
from gpd.msg import GraspConfig
from gpd.srv import SetParameters
from ibvs_perception.cfg import GPDwsConfig
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros
from sensor_msgs import point_cloud2

from math import pi
import numpy as np
from numpy.linalg import norm
import signal
import sys


# Simple Quaternion Rotation mat. from euler angles
def R(axis, angle):
    if axis == "x":
        return quaternion_from_euler(angle, 0, 0)
    elif axis == "y":
        return quaternion_from_euler(0, angle, 0)
    elif axis == "z":
        return quaternion_from_euler(0, 0, angle)


class BestGrasps:
    def __init__(self):
        self.grasps = []
        self.gpd_srv = rospy.ServiceProxy('/gpd/set_params', SetParameters)
        self.gpd_srv_param = SetParameters()

        self.grasp_br = tf2_ros.TransformBroadcaster()
        self.grasp_br_trans = tf2_ros.TransformBroadcaster()

        self.cfg_server = Server(GPDwsConfig, self.cfg_callback)

        self.x_min, self.x_max = 0.03, 0.15
        self.y_min, self.y_max = -0.21, -0.09
        self.z_min, self.z_max = 0.0, 2.0

        self.camera_frame = rospy.get_param('/ibvs_best_grip/camera_frame',
                                            'camera_link')
        grasp_topic = rospy.get_param('/ibvs_best_grip/grasp_topic',
                                      '/ibvs_perception/best_grasp')
        self.grasp_pub = rospy.Publisher(grasp_topic, GraspConfig, queue_size=10)
        
        grasp_sub = rospy.Subscriber('/detect_grasps/clustered_grasps',
                                     GraspConfigList, self.grasp_callback)

        cloud_topic = rospy.get_param('/ibvs_perception/cloud_topic',
                                      '/camera/depth_registered/points')
        pc_sub = rospy.Subscriber(cloud_topic, PointCloud2, self.pc_callback)

        filter_cloud_topic = rospy.get_param('/ibvs_perception/filter_cloud_topic',
                                             '/ibvs_perception/filtered_points')
        self.pc_pub = rospy.Publisher(filter_cloud_topic, PointCloud2, queue_size=100)

        rospy.loginfo("Parameters:\nGrasp Topic: {}\nCloud Topic: {}\nFiltered "
                      "Cloud Topic: {}\nCamera Frame: {}".format(grasp_topic,
                                                                 cloud_topic,
                                                                 filter_cloud_topic,
                                                                 self.camera_frame))
        # Wait for grasps to arrive.
        rate = rospy.Rate(0.1)
        while grasp_sub.get_num_connections() == 0:
            rospy.logwarn_throttle(5, "Waiting for grasps...")
            rate.sleep()

    def cfg_callback(self, config, level):
        rospy.loginfo("Got a reconfigure")
        set_ws = config.set_ws
        set_grasp_ws = config.set_grasp_ws
        set_cam_pos = config.set_cam_pos
        self.x_min, self.x_max = config.ws_min_x, config.ws_max_x,
        self.y_min, self.y_max = config.ws_min_y, config.ws_max_y,
        self.z_min, self.z_max = config.ws_min_z, config.ws_max_z
        ws = [config.ws_min_x, config.ws_max_x,
              config.ws_min_y, config.ws_max_y,
              config.ws_min_z, config.ws_max_z]
        grasp_ws = [config.grasp_ws_min_x, config.grasp_ws_max_x,
                    config.grasp_ws_min_y, config.grasp_ws_max_y,
                    config.grasp_ws_min_z, config.grasp_ws_max_z]
        cam_pos = [config.cam_pos_x, config.cam_pos_y, config.cam_pos_z]
        self.set_message(set_ws, ws, set_grasp_ws, grasp_ws, set_cam_pos, cam_pos)
        return config

    def grasp_callback(self, msg):
        grasps = msg.grasps
        if len(grasps) > 0:
            grasp = grasps[0]  # grasps are sorted in descending order by score
            self._grasp_to_tf(grasp)
            rospy.logdebug('Selected grasp with score: {}'.format(grasp.score.data))
            rospy.logdebug(grasp)
            self.grasp_pub.publish(grasp)

    def pc_callback(self, msg):
        frame_id = msg.header.frame_id
        seq = msg.header.seq
        gen = point_cloud2.read_points(msg, skip_nans=True)
        new_cloud_list = []
        for p in gen:
            x, y, z, _ = p
            if ((x <= self.x_max and x >= self.x_min) and
                (y <= self.y_max and y >= self.y_min) and
                (z <= self.z_max and z >= self.z_min)):
                new_cloud_list.append((x, y, z))
        pc2_header = Header(seq=seq, frame_id=frame_id, stamp=rospy.Time.now())
        pc2_msg = point_cloud2.create_cloud_xyz32(pc2_header, new_cloud_list)
        self.pc_pub.publish(pc2_msg)

    def _unit_vector(self, v):
        return v / norm(v)

    def _axis_angle(self, axis, v):
        axis_norm = self._unit_vector(axis)
        v_norm = self._unit_vector(v)
        result = np.arccos(np.clip(np.dot(axis_norm, v_norm), -1.0, -1.0))
        if not np.isnan(result):
            return result
        else:
            return 0.0

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
        x_axis = np.array([1, 0, 0])
        y_axis = np.array([0, 1, 0])
        z_axis = np.array([0, 0, 1])
        x_grasp = np.array([grasp.axis.x, grasp.axis.y, grasp.axis.z])
        y_grasp = np.array([grasp.binormal.x, grasp.binormal.y, grasp.binormal.z])
        z_grasp = np.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
        x_angle = self._axis_angle(-z_axis, x_grasp)
        y_angle = self._axis_angle(y_axis, y_grasp)
        z_angle = self._axis_angle(x_axis, z_grasp)
        Qg = quaternion_from_euler(x_angle, y_angle, z_angle)
        tf_stamped.transform.rotation.x = Qg[0]
        tf_stamped.transform.rotation.y = Qg[1]
        tf_stamped.transform.rotation.z = Qg[2]
        tf_stamped.transform.rotation.w = Qg[3]

        # Stamp
        tf_stamped.header.stamp = rospy.Time.now()
        self.grasp_br.sendTransform(tf_stamped)

        # Create one for translation
        Tr = tf_stamped
        Qt = quaternion_multiply(Qg, R('x', pi))
        # Qt = quaternion_multiply(Qt, R('z', pi))
        Tr.transform.rotation.x = Qt[0]
        Tr.transform.rotation.y = Qt[1]
        Tr.transform.rotation.z = Qt[2]
        Tr.transform.rotation.w = Qt[3]
        Tr.header.stamp = rospy.Time.now()
        Tr.child_frame_id = 'grasp_trans'
        self.grasp_br_trans.sendTransform(Tr)
        return tf_stamped

    def set_message(self, set_ws=True, ws=None, set_ws_grasps=False, ws_grasps=None,
                    set_cam=False, cam_pos=None):
        zero_ws = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        zero_pos = [0.0, 0.0, 0.0]
        self.gpd_srv_param.set_workspace = set_ws
        self.gpd_srv_param.workspace = zero_ws if ws is None else ws
        self.gpd_srv_param.set_workspace_grasps = set_ws_grasps
        self.gpd_srv_param.workspace_grasps = zero_ws if ws is None else ws_grasps
        self.gpd_srv_param.set_camera_position = set_cam
        self.gpd_srv_param.camera_position = zero_pos if cam_pos is None else cam_pos

    def start_sim(self):
        while not rospy.is_shutdown():
            try:
                rospy.spin()
            except KeyboardInterrupt, rospy.ROSInterruptException:
                sys.exit(0)


def signal_handler(sig, frame):
    print("ctrl-c captured")
    sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    # rospy.init_node('bestGrip', log_level=rospy.DEBUG)
    rospy.init_node('bestGrip')
    bg = BestGrasps()
    bg.start_sim()
