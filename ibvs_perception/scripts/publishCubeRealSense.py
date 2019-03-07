#!/usr/bin/env python
import cv2
import sys
import rospy
from perception.msg import YoloObject
from gpd.srv import SetParameters


global cv_image


class GPD_talker():
  def __init__(self):
    self.pub = rospy.ServiceProxy('/gpd/set_params', SetParameters)
    self.sub = rospy.Subscriber('/jake',YoloObject,self.callback)
    self.service = SetParameters()
    self.setMessage()

  def setMessage(self):
    self.service.set_workspace = False
    self.service.workspace = [0.0,0.0,0.0,0.0,0.0,0.0]
    self.service.set_workspace_grasps = False
    self.service.workspace_grasps = [0.55, 1.0, -0.41, 0.03, -0.29, 1.0]
    self.service.set_camera_position = False
    self.service.camera_position = [0.0,0.0,0.0]
    

  def callback(self,data):
    self.service.set_workspace = True
    SCALE = .13
    XOFFSET = -.0
    XSCALE = 320
    YOFFSET = -.025
    YSCALE = 240
    ws = [0.0,0.0,0.0,0.0,0.0,0.0]
    ws[0] = (-1 + data.baseX/XSCALE) * SCALE + XOFFSET
    ws[1] = (-1 + (data.baseX + data.width) / XSCALE) * SCALE + XOFFSET
    ws[2] = (-1 + (data.baseY/YSCALE))* SCALE + YOFFSET
    ws[3] = (-1 + (data.baseY + data.height) / YSCALE)* SCALE + YOFFSET
    ws[4] = 0.5 #minZ  
    ws[5] = 1.5#maxZ
    self.service.workspace = ws
    rospy.loginfo(self.service.workspace)
    result = self.pub(set_workspace=True, workspace=ws)
    print(result)

def main(args):
  ic = GPD_talker()
  rospy.init_node('GPD_talker', anonymous=True)
  r = rospy.Rate(2)
  try:
    while not rospy.is_shutdown():
      r.sleep()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

