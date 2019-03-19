#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from gpd.msg import GraspConfigList
from gpd.msg import GraspConfig
from gpd.srv import SetParameters
from ibvs_perception.cfg import GPDwsConfig
from visualization_msgs.msg import Marker, MarkerArray

import sys


class BestGrasps:
    def __init__(self):
        self.grasps = []
        self.gpd_srv = rospy.ServiceProxy('/gpd/set_params', SetParameters)
        self.gpd_srv_param = SetParameters()

        self.cfg_server = Server(GPDwsConfig, self.cfg_callback)

        x_min, x_max, y_min, y_max, z_min, z_max = 0.0, 0.5, 0.0, 0.5, 0.0, 0.5
        self.ws_marker = self.create_ws_marker(x_min, x_max, y_min, y_max, z_min, z_max)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=5)

        pub_topic = rospy.get_param('/ibvs_best_grip/grasp_topic',
                                    '/ibvs_perception/best_grasp')
        self.pub = rospy.Publisher(pub_topic, GraspConfig, queue_size=10)
        grasp_sub = rospy.Subscriber('/detect_grasps/clustered_grasps',
                                     GraspConfigList, self.grasp_callback)

        # Wait for grasps to arrive.
        rate = rospy.Rate(0.1)
        while grasp_sub.get_num_connections() == 0:
            rospy.logwarn_throttle(5, "Waiting for grasps...")
            rate.sleep()

        while self.marker_pub.get_num_connections() == 0:
            rate.sleep()

    def cfg_callback(self, config, level):
        rospy.loginfo("Got a reconfigure")
        set_ws = config.set_ws
        set_grasp_ws = config.set_grasp_ws
        set_cam_pos = config.set_cam_pos
        ws = [config.ws_min_x, config.ws_max_x,
              config.ws_min_y, config.ws_max_y,
              config.ws_min_z, config.ws_max_z]
        grasp_ws = [config.grasp_ws_min_x, config.grasp_ws_max_x,
                    config.grasp_ws_min_y, config.grasp_ws_max_y,
                    config.grasp_ws_min_z, config.grasp_ws_max_z]
        cam_pos = [config.cam_pos_x, config.cam_pos_y, config.cam_pos_z]
        self.set_message(set_ws, ws, set_grasp_ws, grasp_ws, set_cam_pos, cam_pos)
        self.ws_marker = self.create_ws_marker(config.ws_min_x, config.ws_max_x,
                                               config.ws_min_y, config.ws_max_y,
                                               config.ws_min_z, config.ws_max_z)
        return config

    def grasp_callback(self, msg):
        self.ws_marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(self.ws_marker)

        grasps = msg.grasps
        if len(grasps) > 0:
            grasp = grasps[0]  # grasps are sorted in descending order by score
            rospy.logdebug('Selected grasp with score: {}'.format(grasp.score.data))
            rospy.logdebug(grasp)
            self.pub.publish(grasp)

    def create_ws_marker(self, x_min, x_max, y_min, y_max, z_min, z_max):
        dx = x_max - x_min
        dy = y_max - y_min
        dz = z_max - z_min
        m1 = Marker()
        m1.ns = "ibvs_marker"
        m1.id = 0
        m1.type = m1.CUBE
        m1.action = m1.ADD
        m1.pose.position.x = (x_max - dx / 2)
        m1.pose.position.y = (y_max - dy / 2)
        m1.pose.position.z = (z_max - dz / 2)
        m1.pose.orientation.x = 0.0
        m1.pose.orientation.y = 0.0
        m1.pose.orientation.z = 0.0
        m1.pose.orientation.w = 1.0
        m1.scale.x = dx
        m1.scale.y = dy
        m1.scale.z = dz
        m1.color.r = 1.0
        m1.color.g = 0.0
        m1.color.b = 0.0
        m1.color.a = 0.5
        m1.lifetime = rospy.Duration()
        # m1.header.frame_id = "/kinect2_link"
        m1.header.frame_id = "/base"
        m1.header.stamp = rospy.Time.now()
        return m1

    def set_message(self,
                    set_ws=True, ws=None,
                    set_ws_grasps=False, ws_grasps=None,
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
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                rospy.spin()
            except KeyboardInterrupt, rospy.ROSInterruptException:
                sys.exit(0)


if __name__ == '__main__':
    # rospy.init_node('bestGrip', log_level=rospy.DEBUG)
    rospy.init_node('bestGrip')
    bg = BestGrasps()
    bg.start_sim()
