#!/usr/bin/env python
import sys
import roslib
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from keyboard.msg import Key
from cv_bridge import CvBridge, CvBridgeError
from os import listdir
from os.path import isfile, join, exists, abspath, dirname
import numpy as np
import std_msgs.msg
import datetime

from face_recognition_ros.srv import *


class photo_manager(object):
    def __init__(self, wait=0.0):
        image_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
        rospy.Subscriber(image_topic, Image, self.image_callback)
        s = rospy.Service('/take_photo_srv', take_photo, self.Isrunnging)
        self.bridge = CvBridge()
        self.num_photo = 0
        self.takepicture = False
        self.cur_time=datetime.datetime.now()

    def Isrunnging(self, req):
        # print(self.detected_msg.data)
        self.takepicture = True
        self.cur_time=datetime.datetime.now()
        cv2.imwrite("photo" + str(self.cur_time) + ".jpg", self.cv2_img)
        self.savedname = "photo" + str(self.cur_time) + ".jpg"
        print('save picture')
        self.num_photo = self.num_photo + 1
        return self.savedname

    def listener(self):
        rospy.spin()

    def image_callback(self, msg):
        # print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # self.cv2_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")

        except CvBridgeError, e:
            print("Image callaback error")
            print(e)

        # rospy.spin()


if __name__ == '__main__':
    rospy.init_node('take_photo_service')
    print("Initialize node")
    Photo_saver = photo_manager()
    print("Take photo service created")
    Photo_saver.listener()
