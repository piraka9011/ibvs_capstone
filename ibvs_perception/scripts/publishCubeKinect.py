#!/usr/bin/env python
import cv2
import sys
import rospy
from ibvs_perception.msg import YoloObject
from gpd.srv import SetParameters

global cv_image


class GPDTalker:
    def __init__(self):
        self.pub = rospy.ServiceProxy('/gpd/set_params', SetParameters)
        self.sub = rospy.Subscriber('/ibvs/perception/yolo_target', YoloObject,
                                    self.callback)
        self.service = SetParameters()
        self.setMessage()

    def setMessage(self):
        self.service.set_workspace = False
        self.service.workspace = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.service.set_workspace_grasps = False
        self.service.workspace_grasps = [0.55, 1.0, -0.41, 0.03, -0.29, 1.0]
        self.service.set_camera_position = False
        self.service.camera_position = [0.0, 0.0, 0.0]

    def callback(self, data):
        self.service.set_workspace = True
        XOFFSET = -.05
        XSCALE = 255
        YOFFSET = -.0
        YSCALE = 212
        ws = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ws[0] = -1 + (data.baseX / XSCALE) + XOFFSET
        ws[1] = -1 + (data.baseX + data.width) / XSCALE + XOFFSET
        ws[2] = -1 + (data.baseY / YSCALE) + YOFFSET
        ws[3] = -1 + (data.baseY + data.height) / YSCALE + YOFFSET
        ws[4] = 0.5  # minZ
        ws[5] = 1.75  # maxZ
        self.service.workspace = ws
        rospy.loginfo(self.service.workspace)
        result = self.pub(set_workspace=True, workspace=ws)
        print(result)


def main(args):
    ic = GPDTalker()
    rospy.init_node('GPD_talker', anonymous=True)
    r = rospy.Rate(2)
    try:
        while not rospy.is_shutdown():
            r.sleep()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
