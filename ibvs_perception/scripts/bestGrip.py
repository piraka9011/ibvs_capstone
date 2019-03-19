#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from gpd.msg import GraspConfigList
from gpd.msg import GraspConfig
from gpd.srv import SetParameters
from ibvs_perception.cfg import GPDwsConfig
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs import point_cloud2

import signal
import sys


class BestGrasps:
    def __init__(self):
        self.grasps = []
        self.gpd_srv = rospy.ServiceProxy('/gpd/set_params', SetParameters)
        self.gpd_srv_param = SetParameters()

        self.cfg_server = Server(GPDwsConfig, self.cfg_callback)

        self.x_min, self.x_max = -0.36, -0.06
        self.y_min, self.y_max = -0.18, 0.15
        self.z_min, self.z_max = 0.0, 2.0
        self.ws_marker = self.create_ws_marker(self.x_min, self.x_max,
                                               self.y_min, self.y_max,
                                               self.z_min, self.z_max)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=5)

        grasp_topic = rospy.get_param('/ibvs_best_grip/grasp_topic',
                                      '/ibvs_perception/best_grasp')
        self.grasp_pub = rospy.Publisher(grasp_topic, GraspConfig, queue_size=10)
        grasp_sub = rospy.Subscriber('/detect_grasps/clustered_grasps',
                                     GraspConfigList, self.grasp_callback)

        cloud_topic = '/kinect2/qhd/points'
        pc_sub = rospy.Subscriber(cloud_topic, PointCloud2, self.pc_callback)
        filter_cloud_topic = rospy.get_param('/ibvs_perception/filter_cloud_topic',
                                             '/ibvs_perception/filtered_points')
        self.pc_pub = rospy.Publisher(filter_cloud_topic, PointCloud2, queue_size=100)

        rospy.loginfo("Parameters:\nGrasp Topic: {}\nCloud Topic: {}\nFiltered "
                      "Cloud Topic: {}".format(grasp_topic, cloud_topic, filter_cloud_topic))
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


def signal_handler(sig, frame):
    print("ctrl-c captured")
    sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    # rospy.init_node('bestGrip', log_level=rospy.DEBUG)
    rospy.init_node('bestGrip')
    bg = BestGrasps()
    bg.start_sim()
