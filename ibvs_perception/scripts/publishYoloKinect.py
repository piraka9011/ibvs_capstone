#!/usr/bin/env python

import cv2
import sys
import os
import threading
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ibvs_perception.msg import YoloObject, YoloObjectList
from std_msgs.msg import String

# Import detect_object function from training package
sys.path.append(os.path.abspath(os.path.join(__file__, '..', '..', '..')))
from training.detect import YOLODetector

global cv_image

#TODO: Rename variables here so they are more accurate


class YoloMock():
    def __init__(self):
        # /kinect2/sd/image_color_rect
        camera_topic = rospy.get_param('/ibvs/camera_topic',
                                       '/camera/color/image_rect_color')
        target_topic = rospy.get_param('/ibvs/yolo_target_topic',
                                       '/ibvs/perception/yolo_target')

        self.pub = rospy.Publisher(target_topic, YoloObjectList)
        self.bridge = CvBridge()
        self.yolo = YoloObject()
        self.d = YOLODetector()
        self.sub = rospy.Subscriber(camera_topic, Image, self.callback)
        rospy.spin()

    def callback(self, data):
        self.sub.unregister()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # format for YOLO result: [(tag, confidence, (x, y, w, h))]
            # TODO: fallback for 1) more than 1 desired objects found 2) no desired object found?
            r = self.d.run(cv_image)
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
        except CvBridgeError as e:
            print(e)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if data.data == "start":
        ic = YoloMock()
        r = rospy.Rate(1)
        try:
            while not rospy.is_shutdown():
                r.sleep()
        except KeyboardInterrupt:
            print("Shutting down")

        cv2.destroyAllWindows()


def main(args):
    rospy.init_node('YoloMock', anonymous=True)
    rospy.Subscriber("/nlp", String, callback)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

# This part if for testing package import
# Will remove after YOLO is installed on our hardware
# d = YOLODetector()
# img = cv2.imread('../../training/bunny_plushie_1.jpg')
# print d.run(img)
