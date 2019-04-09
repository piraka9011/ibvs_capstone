#!/usr/bin/env python

# ROS
import rospy
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from dialogflow_ros.msg import DialogflowResult
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header

# Sys
import numpy as np
import signal
import sys
import threading


IMG_SCALE_X = 640
IMG_SCALE_Y = 480
WS_X_MAX = 0.8  # 0.93
WS_X_MIN = 0.2  # 0.16
WS_Y_MIN = -0.16  # -0.4
WS_Y_MAX = 0.3  # 0.31
WS_Z_MIN = 0.00
WS_Z_MAX = 0.65


class YoloBounder:
    def __init__(self):
        # Params
        self._offset = 0.07     # Shape of cube from center
        self.desired_object = 'teddy bear'  # Desired object from Dialogflow

        # Bounding Box (BB) params
        self.xctr = 0.0     # 2D X
        self.yctr = 0.0     # 2D Y
        self.center_coordinate = np.array([0.0, 0.0, 0.0])  # 3D X,Y,Z
        self.bb_called = False  # Check if subscriber called

        # OpenCV Bridge
        self.bridge = CvBridge()

        # Dialogflow Setup
        rospy.Subscriber('/dialogflow_client/results', DialogflowResult, self._df_cb)

        # Depth Image for depth info
        self.depth_img = None
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self._img_cb)
        # Depth Camera Intrinsics for converstion
        depth_info = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)
        self.fx = depth_info.K[0]
        self.fy = depth_info.K[4]
        self.cx = depth_info.K[2]
        self.cy = depth_info.K[5]
        # Safe BB update
        self._bb_lock = threading.Lock()

        # PC Setup
        cloud_topic = rospy.get_param('/ibvs_perception/cloud_topic',
                                      '/camera/depth_registered/points')
        pc_sub = rospy.Subscriber(cloud_topic, PointCloud2, self._pc_callback)
        filter_cloud_topic = rospy.get_param('/ibvs_perception/filter_cloud_topic',
                                             '/ibvs_perception/filtered_points')
        self.pc_pub = rospy.Publisher(filter_cloud_topic, PointCloud2, queue_size=10)

        # Darknet
        bb_topic = '/darknet_ros/bounding_boxes'
        self._bb_sub = rospy.Subscriber(bb_topic, BoundingBoxes, self._bb_callback)

        rospy.loginfo("Parameters:\n"
                      "Cloud Topic: {}\n"
                      "Filtered Cloud Topic: {}".format(cloud_topic, filter_cloud_topic))

        # Wait for images to arrive.
        rate = rospy.Rate(0.1)
        while self._bb_sub.get_num_connections() == 0:
            rospy.logwarn_throttle(5, "Waiting for images...")
            rate.sleep()

    def project_pixel_to_3d(self, u, v):
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        norm = np.sqrt(x * x + y * y + 1)
        x /= norm
        y /= norm
        z = 1.0 / norm
        return np.array([x, y, z])

    def compute_center(self, xmin, xmax, ymin, ymax):
        xctr = (xmin + xmax) / 2
        yctr = (ymin + ymax) / 2
        if xctr > IMG_SCALE_X:
            xctr = IMG_SCALE_X
        if yctr > IMG_SCALE_Y:
            yctr = IMG_SCALE_Y

        return xctr, yctr

    def _df_cb(self, msg):
        if msg.action == 'grab_object':
            parameters = msg.parameters
            for parameter in parameters:
                if parameter.param_name == 'Capstone_Entity':
                    self.desired_object = parameter.value[0]
                    rospy.loginfo("Updated object to: {}".format(self.desired_object))

    def _img_cb(self, msg):
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            print(e)

        if self.bb_called:
            # Get Depth of 2D position
            depth_pt = self.depth_img[self.yctr][self.xctr]
            # Convert center to 3D normal vector
            ctr_pt = self.project_pixel_to_3d(self.xctr, self.yctr)
            # Change magnitude based on depth
            self.center_coordinate = ctr_pt * depth_pt
            # Scale down
            if depth_pt > 0.01:
                self.center_coordinate = self.center_coordinate / 1000

    def _pc_callback(self, msg):
        frame_id = msg.header.frame_id
        seq = msg.header.seq
        gen = point_cloud2.read_points(msg, skip_nans=True)
        new_cloud_list = []
        pcx_max = self.center_coordinate[0] + self._offset
        pcx_min = self.center_coordinate[0] - self._offset
        pcy_max = self.center_coordinate[1] + self._offset
        pcy_min = self.center_coordinate[1] - self._offset
        # Filter
        for p in gen:
            x, y, z, _ = p
            if ((x <= pcx_max and x >= pcx_min) and
                (y <= pcy_max and y >= pcy_min) and
                (z <= WS_Z_MAX and z >= WS_Z_MIN)):
                new_cloud_list.append((x, y, z))
        # Republish
        pc2_header = Header(seq=seq, frame_id=frame_id, stamp=rospy.Time.now())
        pc2_msg = point_cloud2.create_cloud_xyz32(pc2_header, new_cloud_list)
        self.pc_pub.publish(pc2_msg)

    def _bb_callback(self, msg):
        self.bb_called = True
        for box in msg.bounding_boxes:
            box_obj = box.Class
            if box_obj == self.desired_object:
                self._bb_lock.acquire()
                self.xctr, self.yctr = self.compute_center(box.xmin, box.xmax, box.ymin,
                                                           box.ymax)
                self._bb_lock.release()

    def start_sim(self):
        while not rospy.is_shutdown():
            try:
                rospy.spin()
            except KeyboardInterrupt, rospy.ROSInterruptException:
                sys.exit(0)


def signal_handler(sig, frame):
    print("Ctrl-c captured")
    sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    # rospy.init_node('bestGrip', log_level=rospy.DEBUG)
    rospy.init_node('yolo_bounder')
    yb = YoloBounder()
    yb.start_sim()
