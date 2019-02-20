#!/usr/bin/env python

import cv2
import sys
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from perception.msg import YoloObject

global cv_image

class YoloMock():
  def __init__(self):
    self.pub = rospy.Publisher('/ibvs/perception/yolo_target',YoloObject)
    self.bridge = CvBridge()
    self.sub = rospy.Subscriber('/camera/color/image_rect_color',Image,self.callback)
    self.yolo = YoloObject()

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.imshow("Window", cv_image)
      cv2.waitKey(10)
      r = cv2.selectROI("Window",cv_image, False)
      print(r)
      self.yolo.tag = 'MockImage'
      self.yolo.score = 78.4325
      self.yolo.baseX = r[0]
      self.yolo.baseY = r[1]
      self.yolo.width = r[2]
      self.yolo.height = r[3]
      rospy.loginfo(self.yolo)
      self.pub.publish(self.yolo)

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

