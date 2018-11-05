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

'''
Basic Baxter functions
Here you'll find the following functions:
message_matrix_to_pose: Converts a Matrix into Euler angles
move_baxter: Moves an arm to the desired position given as position in space (x,y,z) and angles (x,y,z)
move_head: Lets you move Baxter's head
send_image: Shows an image on Baxter's head screen
get_angles: Prints every joint angle for the desired arm
camera_video: Captures a real time video using Baxter's arm camera
QtoE: Converts and prints Quaternion angles to Euler angles
BothArms: Lets you move both arms at the same time

Other considerations:
X is positive to the front of the robot
Y is positive to the left of the robot (from his perspective)
Z is positive up
'''
picture = None

class move():

	def __init__(self, arm):
		
		self.limb = arm
		self.limb_interface = baxter_interface.Limb(self.limb) #Declares the limb
		self.head = baxter_interface.Head()	#Declares the head
		self.gripper = baxter_interface.Gripper(self.limb)	#Declares the gripper
		self.camera = baxter_interface.CameraController('right_hand_camera')
		self.camera.open()
		self.camera.resolution          = self.camera.MODES[0]

		self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size = 10) #Publisher for changing the screen display
	
		#Declares the other arm (The opposite arm to the one passed as argument. If you pass left, declares right as the other arm and vice versa)

		if arm == "left":
			self.other_limb = "right"
		else:
			self.other_limb = "left"

		self.other_limb_interface = baxter_interface.Limb(self.other_limb)

		self.limb_interface.set_joint_position_speed(0.5) #Sets the speed for the args arm. {0.0, 1.0}
		self.other_limb_interface.set_joint_position_speed(0.5) #Sets the speed for the other arm {0.0, 1.0}

		self.angles = self.limb_interface.joint_angles()	#Stores all the joint angles	
	
	def message_matrix_to_pose(self, T, frame):
		t = PoseStamped()
		t.header.frame_id = frame
		t.header.stamp = rospy.Time.now()
		translation = tf.transformations.translation_from_matrix(T) #Transforms a matrix to x, y, z positions
		orientation = tf.transformations.quaternion_from_matrix(T)	#Transforms a matrix to x, y, z, w Quaternion angles
		t.pose.position.x = translation[0]
		t.pose.position.y = translation[1]
		t.pose.position.z = translation[2]
		t.pose.orientation.x = orientation[0]
		t.pose.orientation.y = orientation[1]
		t.pose.orientation.z = orientation[2]
		t.pose.orientation.w = orientation[3]
		return t

	def move_baxter(self, source_frame, trans, rot):
		service_name = '/ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
		ik_service = rospy.ServiceProxy(service_name, SolvePositionIK)
		frame = source_frame

		self.limb_interface.set_joint_position_speed(0.5)
	
		matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans),
			tf.transformations.euler_matrix(rot[0], rot[1], rot[2]))
		
		rospy.wait_for_service(service_name, 10)
		ik_message = SolvePositionIKRequest()
		ik_message.pose_stamp.append(self.message_matrix_to_pose(matrix, frame))
		
		try:
			response = ik_service(ik_message)
		except:
			print "Movement couldn't be executed"

		print response.isValid[0]
		
		if response.isValid[0] == True:
			movimiento = dict(zip(response.joints[0].name, response.joints[0].position))
			self.limb_interface.move_to_joint_positions(movimiento)
		else:
			print "Movement couldn't be executed"

		print response.joints[0].position
		print response.joints[0].name
	
	def move_head(self, nod, pan): #Moves Baxter's head
		if nod == True:
			self.head.command_nod() #Nods
		self.head.set_pan(pan)   #Moves head sideways, {-1.5, 1.5}. Negative means movement to the right and positive to the left from Baxter's perspective, if you are in front of baxter the perspective is reversed, hence, negative is left and positive right
			
	def send_image(self, image):	#Shows an image on Baxter's screen
		img = cv2.imread(image)	#Reads an image
		msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8") #Makes the opencv-ros bridge, converts an image to msg
		self.pub.publish(msg) #Shows the image on Baxter's head screen
		rospy.sleep(0.1)

	def get_angles(self):	#Shows all the joint angles
		print self.angles

	def camera_video(self):

		def callback(msg):
			# Transforma el mensaje a imagen
			global picture
			picture = cv_bridge.CvBridge().imgmsg_to_cv2(msg) #, "bgr8") #bgr8

		rospy.Subscriber('/cameras/right_hand_camera/image', Image , callback)

		while not rospy.is_shutdown():
		#Capturar un frame
			while np.all(picture) == None:
				print "hola"
				continue 

			frame = picture
				
			#Mostrar la imagen
			cv2.imshow('Imagen', frame)

			#Salir con 'ESC'
			k = cv2.waitKey(5) & 0xFF
			if k == 27:
				break
		 
		cv2.destroyAllWindows()

	def QtoE(self): #Quaternion to Euler. Converts Quaternion angles(x, y, z, w) into Euler angles (x, y ,z) and prints them
		euler = tf.transformations.euler_from_quaternion(self.limb_interface.endpoint_pose()['orientation'])
		print ("Arm positions and Quaternion angles")
		print (self.limb_interface.endpoint_pose())
		print ("Arm Euler angles: ", euler)

	def BothArms(self):	#Lets you move both arms at the same time

		def threadl():	#Thread for the left arm

			def movel(limb, angle):	#Movement for the left arm
				limb.set_joint_position_speed(1)	#Sets the movement speed of the arm {0.0, 1.0}
				limb.move_to_joint_positions(angle)	#Moves to the provided angles
				# Max Joint Range (s0, s1, e0, e1, w0, w1, w2)
				#     ( 1.701,  1.047,  3.054,  2.618,  3.059,  2.094,  3.059)
				# Min Joint Range (s0, s1, e0, e1, w0, w1, w2)
				#     (-1.701, -2.147, -3.054, -0.050, -3.059, -1.571, -3.059)
				#				[s0, s1, e0, e1, w0, w1, w2] movement matrix format. Example: [0.67, 0.06, -1.49, 1.71, -0.03, 1.52, 1.80]
			left = baxter_interface.Limb('left')	#Declares the arm
			joint_movesl = 	([0.6707, 0.0682, -1.494, 1.7176, -0.032, 1.5270, 1.8093],	#Set of desired movements for the arm
							 [0.525, 0.0690, -1.494, 1.7176, -0.033, 1.5282, 1.8085],
							)

			for move in joint_movesl:
				print move
				tl = threading.Thread(target = movel, args = (left, dict(zip(left.joint_names(), move))))	#Creates the thread
				tl.daemon = True		
				tl.start()
				tl.join()


		def threadr():	#Thread for the right arm

			def mover(limb, angle):#Movement for the right arm
				limb.set_joint_position_speed(1)	#Sets the movement speed of the arm {0.0, 1.0}
				limb.move_to_joint_positions(angle)	#Moves to the provided angles
				# Max Joint Range (s0, s1, e0, e1, w0, w1, w2)
				#     ( 1.701,  1.047,  3.054,  2.618,  3.059,  2.094,  3.059)
				# Min Joint Range (s0, s1, e0, e1, w0, w1, w2)
				#     (-1.701, -2.147, -3.054, -0.050, -3.059, -1.571, -3.059)
				#				[s0, s1, e0, e1, w0, w1, w2] movement matrix format. Example: [-1.61, 0.99, 1.0, 2.1, -2.4, 0.52, 0.02]	
			right = baxter_interface.Limb('right')
			joint_movesr = 	([-1.651, 0.9939, 1.0156, 2.1318, -2.468, 0.5224, 0.0260],
							 [-1.548, 0.8260, 1.2241, 2.2691, -2.215, 0.2791, -0.196],
							)
			for move in joint_movesr:
				print move
				tr = threading.Thread(target = mover, args = (right, dict(zip(right.joint_names(), move))))	#Creates the thread
				tr.daemon = True
				tr.start()
				tr.join()

		rospy.sleep(1)
		threadl = threading.Thread(target = threadl)	#Creates the thread
		threadr = threading.Thread(target = threadr)	#Creates the thread
		threadl.start()
		threadr.start()

def main():
	rospy.init_node('move', anonymous = True) #Initializes a ros node
	mov = move('right')
	#Uncomment the function you want to test
	#mov.send_image('/home/julio/ros_ws/src/baxter_examples/share/images/researchsdk.png')
	#mov.move_head(True, -0.5)
	#mov.move_baxter('base',[0.4, -0.2, -0.1],[-3.09,-0.095,-3.08])
	#mov.get_angles()
	#mov.camera_video()
	#mov.QtoE()
	#mov.BothArms()
	
	#################################################################
	

if __name__ == "__main__":
	main()
