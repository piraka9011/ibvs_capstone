#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from gpd.srv import SetParameters
import signal
import sys

IMG_SCALE = 255
WS_X_MAX = 0.8  # 0.93
WS_X_MIN = 0.2  # 0.16
WS_Y_MIN = -0.16  # -0.4
WS_Y_MAX = 0.3  # 0.31
WS_Z_MIN = -0.2
WS_Z_MAX = 0.45


class YoloBounder:
    def __init__(self):
        self._offset = 0.05
        self.desired_object = 'bottle'
        bb_topic = '/darknet_ros/bounding_boxes'
        self._bb_sub = rospy.Subscriber(bb_topic, BoundingBoxes, self._bb_cb)

        self.gpd_srv = rospy.ServiceProxy('/gpd/set_params', SetParameters)
        self.gpd_srv_param = SetParameters()

    def set_message(self, set_ws=True, ws=None):
        zero_ws = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gpd_srv_param.set_workspace = set_ws
        self.gpd_srv_param.workspace = zero_ws if ws is None else ws

    def _bb_cb(self, msg):
        for box in msg.bounding_boxes:
            box_obj = box.Class
            if box_obj == self.desired_object:
                x_min = (box.xmin / IMG_SCALE) * WS_X_MIN - self._offset
                x_max = (box.xmax / IMG_SCALE) * WS_X_MAX + self._offset
                y_min = (box.ymin / IMG_SCALE) * WS_Y_MIN - self._offset
                y_max = (box.ymax / IMG_SCALE) * WS_Y_MAX + self._offset
                ws = [x_min, x_max, y_min, y_max, WS_Z_MIN, WS_Z_MAX]
                rospy.loginfo_throttle(3, "Xmin: {}, Xmax: {}, Ymin: {}, "
                                          "Ymax: {}".format(x_min, x_max, y_min, y_max))
                self.set_message(ws=ws)

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
