#!/usr/bin/env python

import rospy
import baxter_interface
import baxter_dataflow
import roslib
import cv
import cv2
import cv_bridge
import pygame
import os
import traceback
import threading
import Queue
import rospkg
import std_msgs
import numpy as np
import math
import tf
from sensor_msgs.msg import Image, JointState
from baxter_core_msgs.srv import ListCameras, SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from baxter_core_msgs.msg import AnalogIOStates, EndEffectorState
from baxter_interface import CHECK_VERSION

rospy.init_node('test3', anonymous = True)

def BothArms():
	def threadl():

		def movel(limb, angle):
			limb.set_joint_position_speed(1)
			limb.move_to_joint_positions(angle)
			# Max Joint Range (s0, s1, e0, e1, w0, w1, w2)
			#     ( 1.701,  1.047,  3.054,  2.618,  3.059,  2.094,  3.059)
			# Min Joint Range (e0, e1, s0, s1, w0, w1, w2)
			#     (-1.701, -2.147, -3.054, -0.050, -3.059, -1.571, -3.059)
			#				[s0, s1, e0, e1, w0, w1, w2]	
		left = baxter_interface.Limb('left')
		joint_movesl = 	([0.6707, 0.0682, -1.494, 1.7176, -0.032, 1.5270, 1.8093],
						 [0.525, 0.0690, -1.494, 1.7176, -0.033, 1.5282, 1.8085],
						)

		for move in joint_movesl:
			print move
			tl = threading.Thread(target = movel, args = (left, dict(zip(left.joint_names(), move))))
			tl.daemon = True		
			tl.start()
			tl.join()


	def threadr():

		def mover(limb, angle):
			limb.move_to_joint_positions(angle)
			limb.set_joint_position_speed(1)
			# Max Joint Range (s0, s1, e0, e1, w0, w1, w2)
			#     ( 1.701,  1.047,  3.054,  2.618,  3.059,  2.094,  3.059)
			# Min Joint Range (e0, e1, s0, s1, w0, w1, w2)
			#     (-1.701, -2.147, -3.054, -0.050, -3.059, -1.571, -3.059)
			#				[s0, s1, e0, e1, w0, w1, w2]	
		right = baxter_interface.Limb('right')
		joint_movesr = 	([-1.651, 0.9939, 1.0156, 2.1318, -2.468, 0.5224, 0.0260],
						 [-1.548, 0.8260, 1.2241, 2.2691, -2.215, 0.2791, -0.196],
						)
		for move in joint_movesr:
			print move
			tr = threading.Thread(target = mover, args = (right, dict(zip(right.joint_names(), move))))
			tr.daemon = True
			tr.start()
			tr.join()

	rospy.sleep(10)
	threadl = threading.Thread(target = threadl)
	threadr = threading.Thread(target = threadr)
	threadl.start()
	threadr.start()

BothArms()
