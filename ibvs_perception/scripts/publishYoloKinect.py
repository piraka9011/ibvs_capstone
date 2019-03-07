#!/usr/bin/env python

import cv2
import sys
import os
import threading
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from perception.msg import YoloObject, YoloObjectList
from std_msgs.msg import String

# Import detect_object function from training package
sys.path.append(os.path.abspath(os.path.join(__file__,'..', '..', '..')))
from training.detect import YOLODetector

global cv_image

#TODO: Rename variables here so they are more accurate

class YoloMock():
    def __init__(self):
        # /kinect2/sd/image_color_rect
        camera_topic = rospy.get_param('/ibvs/camera_topic',
                                       '/camera/color/image_rect_color')
        target_topic = rospy.get_param('/ibvs/yolo_target_topic', '/ibvs/perception/yolo_target')
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
            #self.pub.publish(obj_list)
            self.pub = rospy.Publisher('/amber', YoloObjectList)
            rospy.sleep(1)
            self.pub.publish(obj_list)


        except CvBridgeError as e:
            print(e)


# class WaitForMsg():
#     def __init__(self, topic):
#         self.topic = topic
#         self.timeout = None
#         self.mutex = threading.Lock()
#         self.subscriber = rospy.Subscriber(topic, String, self._callback, queue_size=1)
#         self.msg = None
#         self.execute()

#     def _callback(self, msg):
#         self.mutex.acquire()
#         self.msg = msg
#         self.mutex.release()

#     def wait_for_msg(self):
#         rospy.loginfo('waiting for message...')
#         if self.timeout is not None:
#             timeout_time = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
#         while self.timeout is None or rospy.Time.now() < timeout_time:
#             self.mutex.acquire()
#             if self.msg is not None:
#                 rospy.loginfo('got message.')
#                 message = self.msg
#                 if not self.latch:
#                     self.msg = None

#                 self.mutex.release()
#                 return message
#             self.mutex.release()
#             rospy.sleep(.1) # TODO: InterruptException? Discuss with Anas

#         rospy.loginfo(self.state_name + ' experienced a Timeout on waiting for message.')
#         return None

#     def execute(self):
#         msg = self.wait_for_msg()
#         if msg is not None:
#             return msg

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if data.data == "start":
        ic = YoloMock()
        #rospy.init_node('YoloMock', anonymous=True)
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
