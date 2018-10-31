#!/usr/bin/env python
# -*- encoding: utf-8 -*-

#####################################################################################
#                                                                                   #
# Copyright (c) 2014, Active Robots Ltd.                                            #
# All rights reserved.                                                              #
#                                                                                   #
# Redistribution and use in source and binary forms, with or without                #
# modification, are permitted provided that the following conditions are met:       #
#                                                                                   #
# 1. Redistributions of source code must retain the above copyright notice,         #
#    this list of conditions and the following disclaimer.                          #
# 2. Redistributions in binary form must reproduce the above copyright              #
#    notice, this list of conditions and the following disclaimer in the            #
#    documentation and/or other materials provided with the distribution.           #
# 3. Neither the name of the Active Robots nor the names of its contributors        #
#    may be used to endorse or promote products derived from this software          #
#    without specific prior written permission.                                     #
#                                                                                   #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"       #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE         #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE        #
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE          #
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR               #
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF              #
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS          #
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN           #
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)           #
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE        #
# POSSIBILITY OF SUCH DAMAGE.                                                       #
#                                                                                   #
#####################################################################################

from board import *
from engine import *

import rospy
import roslib
import pdb
import cv;
import cv2;
import cv_bridge

import numpy
import math
import os
import sys
import string
import time
import random
import tf
from sensor_msgs.msg import Image
import baxter_interface
from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import matplotlib.pyplot as plt
from operator import itemgetter, attrgetter

# load the package manifest
roslib.load_manifest("activerobots")

# initialise ros node
rospy.init_node("pick_and_place", anonymous = True)

# directory used to save analysis images
image_directory = os.getenv("HOME") + "/Visual_Servoing/"

# locate class
class locate():
    def __init__(self, arm, distance):
	print "1 __init__"
	#called by 33#
        global image_directory
        self.limb           = arm
        self.limb_interface = baxter_interface.Limb(self.limb)

        if arm == "left":
            self.other_limb = "right"
        else:
            self.other_limb = "left"

        self.other_limb_interface = baxter_interface.Limb(self.other_limb)

        # gripper ("left" or "right")
        self.gripper = baxter_interface.Gripper(arm)

        # image directory
        self.image_dir = image_directory

        # flag to control saving of analysis images
        self.save_images = True

        # required position accuracy in metres
        self.piece_tolerance = 0.01
        self.board_tolerance = 0.05

        # number of pieces found
        self.pieces_found = 0

        # start positions
        self.chess_board_x = 0.610                       # x     = front back 0.50 #0.627 #0.537
        self.chess_board_y = -0.295                      # y     = left right 0.00 #-0.025 #-0.035
        self.chess_board_z = 0.220                       # z     = up down 0.05 #0.297
        self.chess_piece_x = 0.50                        # x     = front back 0.50
        self.chess_piece_y = -0.10                       # y     = left right -0.10
        self.chess_piece_z = 0.05                        # z     = up down 0.05
        self.roll        = -1.0 * math.pi              # roll  = horizontal -1.0 #-1.0
        self.pitch       = 0.0 * math.pi               # pitch = vertical 0.0 #0.0
        self.yaw         = 0.0 * math.pi               # yaw   = rotation 0.0 #1.0

        self.pose = [self.chess_piece_x, self.chess_piece_y, self.chess_piece_z,     \
                     self.roll, self.pitch, self.yaw]

        # camera parametecrs (NB. other parameters in open_camera)
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = 0.0                      # camera gripper offset #0.012
        self.cam_y_offset = 0.0			#0.021
        self.width        = 960 #640 960                       # Camera resolution
        self.height       = 600 #400 600

        # Hough circle accumulator threshold and minimum radius.
        self.hough_accumulator = 35
        self.hough_min_radius  = 10
        self.hough_max_radius  = 35

        # canny image, imagen negra
        self.canny = cv.CreateImage((self.width, self.height), 8, 1)
        # Canny transform parameters
        self.canny_low  = 45
        self.canny_high = 150

        # minimum chess board area
        self.min_area = 15000

        # callback image
        self.cv_image = cv.CreateImage((self.width, self.height), 8, 3)

        # colours
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)
        
	# circle counter
	self.count64 = 0
	self.pieces_coord_ini =	[(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),
				(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),
				(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),
				(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),
				(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),
				(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),
				(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),
				(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)]

	self.pieces_coord =	[]

	self.s2 = 		[(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)]

        # chess board corners
        self.chess_board_corner = [(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0)]

        # chess board places
        self.chess_board_place = [(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0)]
	
	# chess board places in pixels
        self.chess_board_place_pixels = [(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0)]
	
        # Enable the actuators
        baxter_interface.RobotEnable().enable()

        # set speed as a ratio of maximum speed
        self.limb_interface.set_joint_position_speed(0.5)
        self.other_limb_interface.set_joint_position_speed(0.5)

        # create image publisher to head monitor
        self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)

        # calibrate the gripper
        self.gripper.calibrate()

        # display the start splash screen
        #self.splash_screen("Visual Servoing", "Pick and Place") #EDITADO

        # reset cameras
        #self.reset_cameras()

        # close all cameras
        #self.close_camera("left")
        #self.close_camera("right")
        #self.close_camera("head")

        # open required camera, "go to 3"
        self.open_camera(self.limb, self.width, self.height)
	#//back from 3

        # subscribe to required camera, "go to 14"
        self.subscribe_to_camera(self.limb)
	#//back from 14

        # distance of arm to table and chess board
        self.distance      = distance
        self.board_distance = distance #distance - 0.075 original
	
        # move other arm out of harms way, "go to 20"
        if arm == "left":
            self.baxter_ik_move("right", (0.25, -0.50, 0.2, math.pi, 0.0, math.pi))
        else:
            self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))
		
	#//back from 20	

    def move_out(self):
	
	# move other arm out of harms way, "go to 20"
        if self.limb == "left":
            self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))

    # reset all cameras (incase cameras fail to be recognised on boot)
    def reset_cameras(self):
	print "2 reset_cameras"
        reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
        rospy.wait_for_service('cameras/reset', timeout=10)
        reset_srv()

    # open a camera and set camera parameters
    def open_camera(self, camera, x_res, y_res):
	print "3 open_camera"
	#called by 1#
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("ERROR - open_camera - Invalid camera")

        # close camera
        #cam.close()

        # set camera parameters
        cam.resolution          = (int(x_res), int(y_res))
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # open camera
        cam.open()
	#'back to 1'

    # close a camera
    def close_camera(self, camera):
	print "4 close_camera"
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("ERROR - close_camera - Invalid camera")

        # set camera parameters to automatic
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # close camera
        cam.close()

    # convert Baxter point to image pixel
    def baxter_to_pixel(self, pt, dist):
	print "5 baxter_to_pixel"
	dist = 0.37 #0.525
        x = (self.width / 2)                                                         \
          + int((pt[1] - (self.pose[1] + self.cam_y_offset)) / (self.cam_calib * dist))
        y = (self.height / 2)                                                        \
          + int((pt[0] - (self.pose[0] + self.cam_x_offset)) / (self.cam_calib * dist))

        return (x, y)

    # convert image pixel to Baxter point
    def pixel_to_baxter(self, px, dist):
	#print "6 pixel_to_baxter"
	dist = 0.37 #0.525
        x = ((px[1] - (self.height / 2)) * self.cam_calib * dist)                \
          + self.pose[0] + self.cam_x_offset
        y = ((px[0] - (self.width / 2)) * self.cam_calib * dist)                 \
          + self.pose[1] + self.cam_y_offset

        return (x, y)

    # Not a tree walk due to python recursion limit
    def tree_walk(self, image, x_in, y_in):
	#print "7 tree_walk"
	#called by 8#
        almost_black = (1, 1, 1)

        pixel_list = [(x_in, y_in)]                   # first pixel is black save position
        cv.Set2D(image, y_in, x_in, almost_black)     # set pixel to almost black
        to_do = [(x_in, y_in - 1)]                    # add neighbours to to do list
        to_do.append([x_in, y_in + 1])
        to_do.append([x_in - 1, y_in])
        to_do.append([x_in + 1, y_in])

        while len(to_do) > 0:
            x, y = to_do.pop()                             # get next pixel to test
            if cv.Get2D(image, y, x)[0] == self.black[0]:  # if black pixel found
                pixel_list.append([x, y])                  # save pixel position
                cv.Set2D(image, y, x, almost_black)        # set pixel to almost black
                to_do.append([x, y - 1])                   # add neighbours to to do list
                to_do.append([x, y + 1])
                to_do.append([x - 1, y])
                to_do.append([x + 1, y])

        return pixel_list
	#'back to 8'

    # Remove artifacts and find largest object
    def look_for_chess_board(self, canny):
	print "8 look_for_chess_board"
	#called by 24#        
	width, height = cv.GetSize(canny)

        centre   = (0, 0)
        max_area = 0

        # for all but edge pixels
        for x in range(1, width - 2):
            for y in range(1, height - 2):
                if cv.Get2D(canny, y, x)[0] == self.black[0]:       # black pixel found
                    pixel_list = self.tree_walk(canny, x, y)        # tree walk pixel "go to 7"
                    if len(pixel_list) < self.min_area:             # if object too small //back from 7
                        for l in pixel_list:
                            cv.Set2D(canny, l[1], l[0], self.white) # set pixel to white
                    else:                                           # if object found
                        n = len(pixel_list)
                        if n > max_area:                            # if largest object found
                            sum_x  = 0                              # find centre of object
                            sum_y  = 0
                            for p in pixel_list:
                                sum_x  = sum_x + p[0]
                                sum_y  = sum_y + p[1]

                            centre = sum_x / n, sum_y / n           # save centre of object
                            max_area = n                            # save area of object

        if max_area > 0:                                            # in board found
            cv.Circle(canny, (centre), 9, (250, 250, 250), -1)      # mark board centre

        # display the modified canny
        cv.ShowImage("Modified Canny", canny)
	print "7 tree_walk"
        # 3ms wait
        cv.WaitKey(3)
	
	#print "centre: ", centre

        return centre                                        # return centre of object
	#'back to 24'

    # flood fill edge of image to leave objects
    def flood_fill_edge(self, canny):
	print "9 flood_fill_edge"
	#called by 24#
        width, height = cv.GetSize(canny)

        # set boarder pixels to white
        for x in range(width):
            cv.Set2D(canny, 0, x, self.white)
            cv.Set2D(canny, height - 1, x, self.white)

        for y in range(height):
            cv.Set2D(canny, y, 0, self.white)
            cv.Set2D(canny, y, width - 1, self.white)

        # prime to do list
        to_do = [(2, 2)]
        to_do.append([2, height - 3])
        to_do.append([width - 3, height - 3])
        to_do.append([width - 3, 2])

        while len(to_do) > 0:
            x, y = to_do.pop()                               # get next pixel to test
            if cv.Get2D(canny, y, x)[0] == self.black[0]:    # if black pixel found
                cv.Set2D(canny, y, x, self.white)            # set pixel to white
                to_do.append([x, y - 1])                     # add neighbours to to do list
                to_do.append([x, y + 1])
                to_do.append([x - 1, y])
                to_do.append([x + 1, y])
	#back to 24'

    # camera call back function
    def camera_callback(self, data, camera_name):
	#print "10 camera_callback"
        # Convert image from a ROS image message to a CV image
        try:
            self.cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8") #cv_image = lo que muestra la camara
	    #self.cv_image = cv2.flip(self.cv_image, -1)
        except cv_bridge.CvBridgeError, e:
            print e

        # 3ms wait
        cv.WaitKey(3)

    # left camera call back function
    def left_camera_callback(self, data):
	#print "11 left_camera_callback"
        self.camera_callback(data, "Left Hand Camera")

    # right camera call back function
    def right_camera_callback(self, data):
	#print "12 right_camera_callback"
        self.camera_callback(data, "Right Hand Camera")

    # head camera call back function
    def head_camera_callback(self, data):
	#print "13 head_camera_callback"
        self.camera_callback(data, "Head Camera")

    # create subscriber to the required camera
    def subscribe_to_camera(self, camera):
	#print "14 subscribe_to_camera"
	#called by 1#
        if camera == "left":
            callback = self.left_camera_callback
            camera_str = "/cameras/left_hand_camera/image"
        elif camera == "right":
            callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        elif camera == "head":
            callback = self.head_camera_callback
            camera_str = "/cameras/head_camera/image"
        else:
            sys.exit("ERROR - subscribe_to_camera - Invalid camera")

        camera_sub = rospy.Subscriber(camera_str, Image, callback)
	#'back to 1'
	
    # Convert cv image to a numpy array
    def cv2array(self, im):
	#print "15 cv2array"
        depth2dtype = {cv.IPL_DEPTH_8U: 'uint8',
                       cv.IPL_DEPTH_8S: 'int8',
                       cv.IPL_DEPTH_16U: 'uint16',
                       cv.IPL_DEPTH_16S: 'int16',
                       cv.IPL_DEPTH_32S: 'int32',
                       cv.IPL_DEPTH_32F: 'float32',
                       cv.IPL_DEPTH_64F: 'float64'}
  
        arrdtype=im.depth
        a = numpy.fromstring(im.tostring(),
                             dtype = depth2dtype[im.depth],
                             count = im.width * im.height * im.nChannels)
        a.shape = (im.height, im.width, im.nChannels)

        return a

    # find next object of interest
    def find_next_chess_piece(self, piece_data, iteration):
	#print "16 find_next_chess_piece"
        # if only one object then object found
        if len(piece_data) == 1:
            return piece_data[0]

        # sort objects right to left
        od = []
        for i in range(len(piece_data)):
            od.append(piece_data[i])

        od.sort()

        # if one piece is significantly to the right of the others
        if od[1][0] - od[0][0] > 30:       # if piece significantly to right of the others
            return od[0]                   # return right most piece
        elif od[1][1] < od[0][1]:          # if right most piece below second piece
            return od[0]                   # return lower piece
        else:                              # if second piece below right most piece
            return od[1]                   # return lower piece
    
    # IMPORTANTE
    # find gripper angle to avoid nearest neighbour
    def find_gripper_angle(self, next_piece, piece_data):
	#print "17 find_gripper_angle"
        # if only one piece any angle will do
        if len(piece_data) == 1:
            return self.yaw

        # find nearest neighbour
        neighbour = (0, 0)
        min_d2    = float(self.width * self.width + self.height * self.height)

        for i in range(len(piece_data)):
            if piece_data[i][0] != next_piece[0] or piece_data[i][1] != next_piece[1]:
                dx = float(piece_data[i][0]) - float(next_piece[0])   # NB x and y are ushort
                dy = float(piece_data[i][1]) - float(next_piece[1])   # float avoids error
                d2 = (dx * dx) + (dy * dy)
                if d2 < min_d2:
                    neighbour = piece_data[i]
                    min_d2    = d2

        # find best angle to avoid hitting neighbour
        dx = float(next_piece[0]) - float(neighbour[0])
        dy = float(next_piece[1]) - float(neighbour[1])
        if abs(dx) < 1.0:
            angle = - (math.pi / 2.0)             # avoid divide by zero
        else:
            angle = math.atan(dy / dx)            # angle in radians between -pi and pi
        angle = angle + (math.pi / 2.0)           # rotate pi / 2 radians
        if angle > math.pi / 2.0:                 # ensure angle between -pi and pi
            angle = angle - math.pi

        return - angle                            # return best angle to grip chess piece

    # Use Hough circles to find piece centres (Only works with round objects)
    def hough_it(self, n_piece, iteration):
	#print "19 hough_it"
        # create gray scale image of pieces
        gray_image = cv.CreateImage((self.width, self.height), 8, 1)
        cv.CvtColor(cv.fromarray(self.cv_image), gray_image, cv.CV_BGR2GRAY)

        # create gray scale array of pieces
        gray_array = self.cv2array(gray_image)

        # find Hough circles, dist = 40
        circles = cv2.HoughCircles(gray_array, cv.CV_HOUGH_GRADIENT, 1, 25, param1=50,  \
                  param2=self.hough_accumulator, minRadius=self.hough_min_radius,       \
                  maxRadius=self.hough_max_radius)

        # Check for at least one piece found
        if circles is None:
            # display no pieces found message on head display
            self.splash_screen("no pieces", "found")
            # no point in continuing so exit with error message
            sys.exit("ERROR - hough_it - No piece found")

        circles = numpy.uint16(numpy.around(circles))

        piece_data = {}
        n_pieces   = 0

        circle_array = numpy.asarray(self.cv_image)
	font     = cv.InitFont(cv.CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 1)
        # check if chess piece is in chess board
	while n_pieces != 64:
		for i in circles[0,:]:
		    # convert to baxter coordinates
		    piece = self.pixel_to_baxter((i[0], i[1]), self.board_distance)
		    piece_data[n_pieces]  = (i[0], i[1], i[2])
		    n_pieces            += 1
		    k = 0
		    if n_pieces == 64:
		        self.count64 += 1
			for j in circles[0,:]:
				if k < 64:
					self.pieces_coord_ini[k] = (j[0], j[1])
					k += 1 
			s = sorted(self.pieces_coord_ini, key=itemgetter(1,0))
			self.pieces_coord_ini = sorted(self.pieces_coord_ini, key=itemgetter(1,0))
			self.pieces_coord = []
			m = 0
			d = 0
			while m < 64:
				self.s2[d] = s[m]
				if (m+1) % 8 == 0:
					self.s2.sort()
					self.pieces_coord.append(self.s2)
					self.s2 = [(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)]
					d = -1
				m += 1
				d += 1
			for l in range(8):
				for q in range(8):
					# draw the outer circle in blue
					cv2.circle(circle_array, (self.pieces_coord[l][q][0], self.pieces_coord[l][q][1]), 20, (255, 0, 0), 2)
					# draw the center of the circle in blue
					cv2.circle(circle_array, (self.pieces_coord[l][q][0], self.pieces_coord[l][q][1]), 2, (255, 0, 0), 3)
					cv.PutText(cv.fromarray(self.cv_image), str(l*8 + q), (self.pieces_coord[l][q][0], self.pieces_coord[l][q][1]), font, (0,255,0))

			# passes new coords to chess_board_place_pixels
			for l in range(8):
				for q in range(8):
					if l < 2:	
						self.chess_board_place_pixels[l*8+q] = (self.pieces_coord[l][q][0], self.pieces_coord[l][q][1])
					elif l >1:
						self.chess_board_place_pixels[l*8+q] = (self.pieces_coord[l][q][0], self.pieces_coord[l][q][1]) #32+l*8+q

			#draw chess_board_place coordinates to verify
			for aux1 in range(64): #12
				cv2.circle(circle_array, (self.chess_board_place_pixels[aux1][0], self.chess_board_place_pixels[aux1][1]),2, (0, 0, 255), 2)
				cv.PutText(cv.fromarray(self.cv_image), str(aux1), (self.chess_board_place_pixels[aux1][0], self.chess_board_place_pixels[aux1][1]), font, self.black)
	
		    #endwhile

		#print circles
		print "n_pieces = ", n_pieces
		print "count64 = ", self.count64

		#print "6 pixel_to_baxter"
		circle_image = cv.fromarray(circle_array)
		#cv.ShowImage("Hough Circle", circle_image)
		#3ms wait
		cv.WaitKey(3)

		# display image on head monitor
		font     = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 1)
		position = (30, 60)
		s = "Buscando Piezas"
		cv.PutText(circle_image, s, position, font, self.white)
		msg = cv_bridge.CvBridge().cv2_to_imgmsg(circle_array, encoding="bgr8")
		self.pub.publish(msg)
		
		if self.save_images:
		    # save image of Hough circles on raw image
		    file_name = self.image_dir                                                 \
		              + "hough_circle_" + str(n_piece) + "_" + str(iteration) + ".jpg"
		    cv.SaveImage(file_name, circle_image)

		# Check for at least one piece found
		if n_pieces == 0:                    # no pieces found
		    # display no pieces found message on head display
		    self.splash_screen("no pieces", "found")
		    sys.exit("ERROR - hough_it - No se encontraron piezas")

		# select next piece and find it's position
		next_piece = self.find_next_chess_piece(piece_data, iteration)

		# find best gripper angle to avoid touching neighbouring piece
		angle = self.find_gripper_angle(next_piece, piece_data)
		
		#converts chess board place pixels to chess board place
		for aux in range(64):
			self.chess_board_place[aux] = self.pixel_to_baxter(self.chess_board_place_pixels[aux], self.board_distance)
		
		# return next chess piece position and pickup angle
		return next_piece, angle

    # move a limb
    def baxter_ik_move(self, limb, rpy_pose):
	print "20 baxter_ik_move"
	#called by 1#
	#called by 33#
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if self.limb == limb:
                self.limb_interface.move_to_joint_positions(limb_joints)
            else:
                self.other_limb_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display, "go to 31"
            self.splash_screen("Invalid", "move")
	    #//back from 31
            # little point in continuing so exit with error message
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

        if self.limb == limb:               # if working arm
            quaternion_pose = self.limb_interface.endpoint_pose()
            position        = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved
            self.pose = [position[0], position[1],                                \
                         self.pose[2], self.pose[3], self.pose[4], self.pose[5]]
	#'back to 1'
	#'back to 33'

    # find distance of limb from nearest line of sight object
    def get_distance(self, limb):
	print "21 get_distance"
        if limb == "left":
            #dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
	    dist = 0.62 #0.55
        elif limb == "right":
            #dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
	    dist = 0.62 #0.55
        else:
            sys.exit("ERROR - get_distance - Invalid limb")

        # convert mm to m and return distance
        return float(dist / 1000.0)

    # randomly adjust a pose to dither arm position
    # used to prevent stalemate when looking for chess board
    def dither(self):
	print "23x dither"
        x = self.chess_board_x
        y = self.chess_board_y + (random.random() / 10.0) #y = self.chess_board_y + (random.random() / 10.0)
        pose = (x, y, self.chess_board_z, self.roll, self.pitch, self.yaw)

        return pose

    # find the chess board
    def canny_it(self, iteration):
	print "24 canny_it"
	#called by 27#
	#called by 24 (iterative)#
	#called by 23#
        if self.save_images:
            # save raw image of chess board
            file_name = self.image_dir + "chess_board_" + str(iteration) + ".jpg"
	    self.cv_image = self.cv_image
            cv.SaveImage(file_name, cv.fromarray(self.cv_image))

        # create an empty image variable, the same dimensions as our camera feed.
        gray = cv.CreateImage((cv.GetSize(cv.fromarray(self.cv_image))), 8, 1)

        # convert the image to a grayscale image
        cv.CvtColor(cv.fromarray(self.cv_image), gray, cv.CV_BGR2GRAY)

        # display image on head monitor
        font     = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 1)
        position = (30, 60)
        cv.PutText(cv.fromarray(self.cv_image), "Buscando Tablero", position, font, self.white)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
        self.pub.publish(msg)

        # create a canny edge detection map of the greyscale 
        cv.Canny(gray, self.canny, self.canny_low, self.canny_high, 3)

        # display the canny transformation
        cv.ShowImage("Canny Edge Detection", self.canny)

        if self.save_images:
            # save Canny image of chess board
            file_name = self.image_dir + "canny_board_" + str(iteration) + ".jpg"
            cv.SaveImage(file_name, self.canny)

        # flood fill edge of image to leave only objects "go to 9"
        self.flood_fill_edge(self.canny)
	#//back from 9, "go to 8"
        chess_board_centre = self.look_for_chess_board(self.canny)
	#//back from 8

        # 3ms wait
        cv.WaitKey(3)

        while chess_board_centre[0] == 0:
            if random.random() > 0.6:
                self.baxter_ik_move(self.limb, self.dither())
	    #"go to 24" (iterate)
            chess_board_centre = self.canny_it(iteration)
	    #//back from 24

        return chess_board_centre

    def chess_piece_iterate(self, n_ball, iteration, ball_data):
	#print "28 chess_piece_iterate"
        # print iteration number
        print "PIECE", n_ball, "ITERATION ", iteration

        # find displacement of ball from centre of image
        pixel_dx    = (self.width / 2) - ball_data[0]
        pixel_dy    = (self.height / 2) - ball_data[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.board_distance)

        x_offset = - pixel_dy * self.cam_calib * self.board_distance
        y_offset = - pixel_dx * self.cam_calib * self.board_distance

		#"go to 19"
        ball_data, angle = self.hough_it(n_ball, iteration)

        # find displacement of ball from centre of image
        pixel_dx    = (self.width / 2) - ball_data[0]
        pixel_dy    = (self.height / 2) - ball_data[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.board_distance)

        return ball_data, angle, error

    # find all the chess pieces and place them in the chess board
    def pick_and_place(self):
	print "30 pick_and_place"

        n_piece = 0
        while True and n_piece < 1:              
            n_piece          += 1
            iteration        = 0
            angle            = 0.0

            # use Hough circles to find pieces and select one piece
			# "go to 19"
            next_piece, angle = self.hough_it(n_piece, iteration)

            error     = 2 * self.piece_tolerance

            print
            print "Square number ", n_piece
            print "==============="

            # iterate to find next chess piece
            # if hunting to and fro accept error in position
            while error > self.piece_tolerance and iteration < 100:
                iteration               += 1
                next_piece, angle, error  = self.chess_piece_iterate(n_piece, iteration, next_piece)
            
            font     = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 1)
            position = (30, 60)
            s        = "Tomando Piezas de ajedrez"
            cv.PutText(cv.fromarray(self.cv_image), s, position, font, self.white)
            msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
            self.pub.publish(msg)
            
    # display message on head display
    def splash_screen(self, s1, s2):
	print "31 splash_screen"
        splash_array = numpy.zeros((self.height, self.width, 3), numpy.uint8)
        font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 3.0, 3.0, 9)

        ((text_x, text_y), baseline) = cv.GetTextSize(s1, font)
        org = ((self.width - text_x) / 2, (self.height / 3) + (text_y / 2))
        cv2.putText(splash_array, s1, org, cv.CV_FONT_HERSHEY_SIMPLEX, 3.0,          \
                    self.white, thickness = 7)

        ((text_x, text_y), baseline) = cv.GetTextSize(s2, font)
        org = ((self.width - text_x) / 2, ((2 * self.height) / 3) + (text_y / 2))
        cv2.putText(splash_array, s2, org, cv.CV_FONT_HERSHEY_SIMPLEX, 3.0,          \
                    self.white, thickness = 7)

        splash_image = cv.fromarray(splash_array)

        # 3ms wait
        cv2.waitKey(3)

        msg = cv_bridge.CvBridge().cv2_to_imgmsg(splash_array, encoding="bgr8")
        self.pub.publish(msg)
    
    def get_tabp(self):
	#print "34 get_tabp"
	return self.chess_board_place

# read the setup parameters from setup.dat
def get_setup():
    print "32 get_setup"
    global image_directory
    file_name = image_directory + "setup.dat"

    try:
        f = open(file_name, "r")
    except ValueError:
        sys.exit("ERROR: chess_setup must be run before chess")

    # find limb
    s = string.split(f.readline())
    if len(s) >= 3:
        if s[2] == "left" or s[2] == "right":
            limb = s[2]
        else:
            sys.exit("ERROR: invalid limb in %s" % file_name)
    else:
        sys.exit("ERROR: missing limb in %s" % file_name)

    # find distance to table
    s = string.split(f.readline())
    if len(s) >= 3:
        try:
            distance = float(s[2])
        except ValueError:
            sys.exit("ERROR: invalid distance in %s" % file_name)
    else:
        sys.exit("ERROR: missing distance in %s" % file_name)

    return limb, distance

def main():
    tab = []
    print "33 main"
    # get setup parameters, "go to 32"
    limb, distance = get_setup()
    #//back from 32
    print "limb     = ", limb
    print "distance = ", distance

    # create locate class instance, "go to 1"
    locator = locate(limb, distance)
    #//back from 1
    raw_input("Press Enter to start: ")

    # open the gripper
    locator.gripper.open()

    # move close to the chess board
    locator.pose = (locator.chess_board_x,
                    locator.chess_board_y,
                    locator.chess_board_z,
                    locator.roll,
                    locator.pitch,
                    locator.yaw)
    #"go to 20"
    locator.baxter_ik_move(locator.limb, locator.pose)
    #//back from 20

    #"go to 30"
    locator.pick_and_place()
    #//back from 30
    
    locator.move_out()

    for i in range(64):
        tab.append(locator.get_tabp()[i])

    b=Board()
    e=Engine(tab)

    while(True):

    	b.render()

  	c = raw_input('>>> ') #Comment for baxter against himself
	#c = 'go' #Uncomment for baxter against himself

	if(c=='quit' or c=='exit'):
        	exit(0)

    	elif(c=='undomove'):
        	e.undomove(b)

    	elif('setboard' in c):
	       	e.setboard(b,c) 

    	elif(c=='getboard'):
		e.getboard(b)

	elif(c=='go'):
	        e.search(b)

	elif(c=='new'):
	        e.newgame(b)

	elif(c=='bench'):
	        e.bench(b)        

	elif('sd ' in c):
	        e.setDepth(c)

	elif('perft ' in c):
	        e.perft(c,b)        

	elif(c=='legalmoves'):
	        e.legalmoves(b)
        
	else:
	        e.usermove(b,c)

if __name__ == "__main__":
    main()

