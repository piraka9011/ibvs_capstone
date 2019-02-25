#!/usr/bin/env python

import cv2
import sys
import os
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from perception.msg import YoloObject，YoloObjectList

# Import detect_object function from training package
sys.path.append(os.path.abspath(os.path.join(__file__, '..', '..', '..')))
from training.detect import detect_object

global cv_image

class YoloMock():
  def __init__(self):
    self.pub = rospy.Publisher('/ibvs/perception/yolo_target', YoloObjectList)
    self.bridge = CvBridge()
    self.sub = rospy.Subscriber('/kinect2/sd/image_color_rect',Image,self.callback)
    self.yolo = YoloObject()

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.imshow("Window", cv_image)
      cv2.waitKey(10)
      # r = cv2.selectROI("Window",cv_image, False)
      # format for YOLO result: [(tag, confidence, (x, y, w, h))]
      # TODO: fallback for 1) more than 1 desired objects found 2) no desired object found?
      r = detect_object(cv_image)
      obj_list = YoloObjectList()
      for obj in r:
        yolo = YoloObject()
        yolo.tag, yolo.score, xywh = r[0]
        yolo.baseX, yolo.baseY, yolo.width, yolo.height = xywh
        obj_list.append(yolo)

      rospy.loginfo("Object List: {}".format(obj_list))
      self.pub.publish(obj_list)

    except CvBridgeError as e:
      print(e)



def main(args):
  ic = YoloMock()
  rospy.init_node('YoloMock', anonymous=True)
  r = rospy.Rate(1)
  try:
    while not rospy.is_shutdown():
      r.sleep()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

# This part if for testing package import
# Will remove after YOLO is installed on our hardware
# img = cv2.imread('../../training/IMG_0314.jpg')
# print detect_object(img)

