#!/usr/bin/env python

import cv2
import sys
import os
import signal
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ibvs_perception.msg import YoloObject, YoloObjectList
from std_msgs.msg import String

# Import detect_object function from training package
sys.path.append(os.path.abspath(os.path.join(__file__, '..', '..', '..')))
from training.detect import YOLODetector

#TODO: Rename variables here so they are more accurate


class YoloMock:
    def __init__(self):
        # /kinect2/sd/image_color_rect
        camera_topic = rospy.get_param('/ibvs_yolo_kinect/camera_topic',
                                       '/camera/color/image_raw')
        target_topic = rospy.get_param('/ibvs/yolo_target_topic',
                                       '/ibvs/perception/yolo_target')
        rospy.loginfo("YoloMock Parameters:\nCamera Topic: {}\nTarget Topic: {}".format(
            camera_topic, target_topic))

        self.pub = rospy.Publisher(target_topic, YoloObjectList, queue_size=10)
        rospy.Subscriber(camera_topic, Image, self.callback)

        self.bridge = CvBridge()
        self.yolo = YoloObject()
        self.detector = YOLODetector()
        rospy.loginfo("YoloMock Spinning...")
        rospy.spin()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

        # format for YOLO result: [(tag, confidence, (x, y, w, h))]
        # TODO: fallback for 1) more than 1 desired objects found 2) no desired object found?
        r = self.detector.run(cv_image)
        rospy.loginfo("Done Running")
        obj_list = YoloObjectList()
        for obj in r:
            yolo = YoloObject()
            yolo.tag, yolo.score, xywh = r[0]
            yolo.baseX, yolo.baseY, yolo.width, yolo.height = xywh
            obj_list.objects.append(yolo)

        rospy.loginfo("Object List: {}".format(obj_list))
        rospy.sleep(1)
        self.pub.publish(obj_list)


def signal_handler(sig, frame):
    print("Ctrl-c captured")
    sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('YoloMock')

    try:
        ym = YoloMock()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        sys.exit()


# This part if for testing package import
# Will remove after YOLO is installed on our hardware
# d = YOLODetector()
# img = cv2.imread('../../training/bunny_plushie_1.jpg')
# print d.run(img)
