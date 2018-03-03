#!/usr/bin/env python
import face_recognition
import sys
import roslib
import rospy
import cv2
from hsrb_interface import Robot
from hsrb_interface import geometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from keyboard.msg import Key 
from cv_bridge import CvBridge, CvBridgeError
from os import listdir
from os.path import isfile, join, exists
import numpy as np
import std_msgs.msg
import select, termios, tty
import tensorflow as tf
from rude_carnie.model import select_model, get_checkpoint
from rude_carnie import paths
from rude_carnie.detect import face_detection_model
from face_recognition_ros.srv import *

moveBindings = {
		'i':(4,3,0,0),
		'o':(3,2,0,-1),
		'j':(20,0,0,1),
}

class FaceDectectors(object):
	def __init__(self, wait=0.0):
		self.settings = termios.tcgetattr(sys.stdin)
		image_topic = "/hsrb/head_rgbd_sensor/rgb/image_rect_color"
		rospy.Subscriber(image_topic, Image, self.image_callback)
		s = rospy.Service('/face_recognition_run', face_recognition_srv, self.Isrunnging)
		self.string_pub = rospy.Publisher('/face_detected_name', String, queue_size=10)
		self.bridge = CvBridge()
		self.path =exists("known_faces")
		print self.path
		self.takepicture = False
		self.threshold = 0.6
		self.known_folder = "/home/mk/robocup_home_ws/src/villa_perception/face_recognition//scripts/known_faces"
		self.known_imgs = [f for f in listdir(self.known_folder) if isfile(join(self.known_folder, f))]
		print self.known_imgs
		self.known_img_names = []
		self.known_encodings = []
		self.count=0
		self.unknowncount=0
		self.Isrecognized =False
		self.detected_name ="unknown"

		for img_name in self.known_imgs:
			img = face_recognition.load_image_file(self.known_folder +'/'+img_name)
			print img_name
			self.known_encodings.append(face_recognition.face_encodings(img)[0])
			dot_idx = img_name.rfind('.')
			name = img_name[0:dot_idx]
			self.known_img_names.append(name)

	def Isrunnging(self,req):
		# print(self.detected_msg.data)
		return self.detected_name

	def listener(self):
		rospy.spin()
	

	def key_callback(self,msg):		#"p"
		self.code=msg.code
		if self.code==112:
		   self.takepicture = True
		   print('take picture')
		print msg.code

	def crop_face(self,image_data):
		image_batch = image_data
		files = []
		face_detect = face_detection_model('/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')
		face_files = face_detect.run(image_data)
		#print(face_files)
		return face_files
		
	def image_callback(self,msg):
		# print("Received an image!")
		self.detected_msg =std_msgs.msg.String()
		self.unknowncount=self.unknowncount+1
		if(self.unknowncount>10):
			self.detected_name="unknown"
			self.unknowncount=0
		try:
			# Convert your ROS Image message to OpenCV2
			cv2_img   = self.bridge.imgmsg_to_cv2(msg,"bgr8")
			facelists =self.crop_face(cv2_img)
			
			for face in facelists:
				unknown_encoding = face_recognition.face_encodings(face)[0]
				distances = face_recognition.face_distance(self.known_encodings, unknown_encoding)
				if(min(distances)<self.threshold):
					match_idx = np.argmin(distances)
					print('matched: '+self.known_img_names[match_idx])
					self.detected_msg.data =self.known_img_names[match_idx]
					self.count=self.count+1
					self.Isrecognized =True
					if(self.count>10):
						self.detected_name=self.detected_msg.data
						self.string_pub.publish(self.detected_msg)
						self.count=0
					self.unknowncount=0
					# self.tts.say('matched')
					
				else:
					self.Isrecognized =False
					self.count=0

		except CvBridgeError, e:
			print(e)

		if self.takepicture:
			cv2.imwrite('new_image.jpeg', cv2_img)
			print('save picture')
			# self.detected_msg.data="Take a photo"
			# self.string_pub.publish(self.detected_msg)
			self.takepicture=False
		# else:
			# Save your OpenCV2 image as a jpeg 
			# cv2.imwrite('minkyu_image.jpeg', cv2_img)


	# for img_name in unknown_imgs:
	# 	img = face_recognition.load_image_file(unknown_folder+'/'+img_name)
	# 	unknown_encoding = face_recognition.face_encodings(img)[0]
	# 	distances = face_recognition.face_distance(known_encodings, unknown_encoding)
	# 	if(min(distances)<threshold):
	# 		match_idx = np.argmin(distances)
	# 		print(img_name+': '+known_img_names[match_idx])
	# 	else:
	# 		print(img_name+': unknown')

	# rospy.spin()
if __name__ == '__main__':
	 print("Pre Initialize node")
	rospy.init_node('face_recognition_service')
	print("Initialize node")
	Face_manager = FaceDectectors(sys.argv[1] if len(sys.argv) > 1 else 0.0)
	Face_manager.listener()
	# s=rospy.Service('face_recognition_srv',face_recognition_srv,Face_manager.listener)
	print("Service created")	