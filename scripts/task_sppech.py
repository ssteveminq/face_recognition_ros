#!/usr/bin/env python

"""
ask 3 as specified in 5.3.3 of the Rulebook
"""

import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from tmc_msgs.msg import Voice
from hark_msgs.msg import HarkSource
import time
import math
import functools
import random
from hsrb_interface import Robot
from villa_task import tts
import re
from hsrb_interface import geometry

speech_time = 0
speech_transcript = ''
listen = None

speech = None
unanswered = False
speak_time = 0

 

def update_face(s):
    global speech
    global stop_speech
    global stop_mov
    global remember
    sentence = " q "
    print s.data
    stop_speech = True
    sentence = "Hello  " + s.data
    if sentence != remember or stop_mov == False:
        stop_mov = True
        speech.say(sentence)
        remember = sentence
    stop_speech = False
    print "wow"


def move_robot(omni_base, azimuth):

    print("Begin moving robot")


def act(robot):
    global listen
    global speech
    global unanswered
    global speech_time
    global question_count
    global turning
    global speak_time
    global stop_speech
    global remember
    remember = " "

    Turned = False
    stop_mov = False
    stop_speech = False
    whole_body = robot.get('whole_body')

    speech = tts.TextToSpeech()
    
    whole_body.move_to_neutral()
    stop_head = rospy.ServiceProxy("/viewpoint_controller/stop", Empty)
    stop_head()
    speech_time = rospy.get_time()
    

    speech_time = rospy.get_time()
    face_detection = rospy.Subscriber('/face_detected_name', String, update_face)
    rospy.spin()


print("Process starts")

with Robot() as robot:
    act(robot)

