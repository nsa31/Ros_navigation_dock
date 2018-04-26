#!/usr/bin/env python

import rospy, cv2, cv_bridge
import actionlib
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
import time
import math
import thread
import imutils
PI = 3.1415926535897

class detect_ar(object):
	def __init__(self):
		self.bridge=cv_bridge.CvBridge()
		self.cam_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.info_cb)
		self.img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.img_cb)
		self.marker = rospy.Subscriber('visualization_marker', Marker, self.process)
		self.axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1],[0,0,0]]).reshape(-1,3) #3D axis len
		self.rvecs = None
		self.tvecs = None
		self.D = None
		self.K = None
		self.img = None
		self.logo_found = False
		self.errx = 0
		
	def info_cb(self, msg): #save D and K matrix
		self.K = np.array(msg.K).reshape(3,3)
		self.D = np.array(msg.D)
	
	def img_cb(self, msg):
		self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		
	def process(self, msg):
		if (msg.id >=0): #change formarkers. For markers 9 to 17, detect 13 marker
			px = msg.pose.position.x
			py = msg.pose.position.y
			pz = msg.pose.position.z
			ox = msg.pose.orientation.x
			oy = msg.pose.orientation.y
			oz = msg.pose.orientation.z
			ow = msg.pose.orientation.w			
			
			theta = 2 * math.acos(ow)
			x = ox / (math.sin(theta/2))
			y = oy / (math.sin(theta/2))
			z = oz / (math.sin(theta/2))
			
			#rotate axis
			x = x * theta
			y = y * theta
			z = z * theta	
			
			rvecs = np.array([x, y, z])			
			tvecs = np.array([px, py, pz])

			
			imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.K, self.D) 	
			#AR CENTER (from Demo6 part2.py)
			cv2.circle(self.img,(imgpts[3][0][0].astype(int), imgpts[3][0][1].astype(int)), 5, (255,255,0), -1)
			#WEBCAM CENTER 
			h, w = self.img.shape[:2]
			wx = w/2
			cv2.circle(self.img, (wx, imgpts[3][0][1].astype(int)),5, (0,0,157), -1)
			
			#error
			self.errx = imgpts[3][0][0] - wx
			#print "In CLASS AR:", self.errx						
			
			self.logo_found = True
			self.return_ar_val()
		else:
			self.logo_found = False
			self.return_ar_val()
		
		#cv2.imshow('ARimg',self.img)
		#cv2.waitKey(3)
		
	def return_ar_errx(self):
		return self.errx
	
	def return_ar_val(self):
		return self.logo_found			
		

class Comp4(object):
	def __init__(self):
		self.cmd_vel_pub = rospy.Publisher('cmd_vel/velocityramp', Twist, queue_size=1)
		#self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
		rospy.Subscriber('/scan', LaserScan, self.scan_cb)
		self.twist = Twist()
		self.ua_logo_flag = False
		self.ar_logo_flag = False
		self.ar_logo_errx = 0
		self.dock_counter = 0
		self.range_ahead = 999
		self.travel_init_wp()
		

	def scan_cb(self, msg):
		depth = []
		for dist in msg.ranges:
			if not np.isnan(dist):
				depth.append(dist)
		if len(depth) == 0:
			self.range_ahead = 999
		else:
			self.range_ahead = min(depth)


	def move(self, speed, distance, isForward):  
		if(isForward):
				self.twist.linear.x = abs(speed)
		else:
				self.twist.linear.x = -abs(speed)
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = 0
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
		while(current_distance < distance):
				self.cmd_vel_pub.publish(self.twist)
				t1=rospy.Time.now().to_sec()
				current_distance= speed*(t1-t0)
		self.twist.linear.x = 0
		self.cmd_vel_pub.publish(self.twist)
			
	def travel_init_wp(self):
		print "TRAVELED TO INIT WP"		
		counter = 0
		counter_true = 0 
		while (counter<5):
			self.ar_obj = detect_ar()
			rospy.sleep(2)
			self.ar_logo_flag = self.ar_obj.return_ar_val()
			print self.ar_logo_flag
			if (self.ar_logo_flag == True):
				self.ar_logo_errx = self.ar_obj.return_ar_errx()
				counter_true = counter_true + 1
				print self.ar_logo_errx, "counter_true while", counter_true
			elif (self.ar_logo_flag == False):
				print "NO AR LOGO"
			counter = counter+ 1
			
		if counter_true >=2:
			print "go to docking"
			self.docking() #TURN OFF TO GET ERRX ONLY
		else:
			print "GO CHECK UA"
		
				
	def docking(self):
		print "DOCKINGGGGGGGGGGGGGGGG"
		# TEST 3 CASES HERE EACH WITH DIFF MOVE & ROTATE (-150 extreme)
		#Case 1: camera extreme right of tgt (face tgt)
		current_angle = 0
		rate = rospy.Rate(0.5)
		if (self.ar_logo_errx < -80 and self.range_ahead >= 0.8 and self.range_ahead != 999):
		 	print "CASE 1"
			#rotate 90 deg & move forward
			while (current_angle < 2):
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1)
				print "CURR ANGLE:", current_angle
				self.twist.angular.z = 16.0 #4.8
				current_angle = current_angle + 1
				rate.sleep()
				
			self.move(0.1, 1.0, 1)
			
			current_angle = 0
			while (current_angle < 4):
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1)
				print "CURR ANGLE3:", current_angle
				self.twist.linear.x = 0
				self.twist.angular.z = -20.0
				current_angle = current_angle + 1
				rate.sleep()
				
			self.travel_init_wp()
			
		#Case 2: camera mid (face tgt)
		elif (self.ar_logo_errx >= -80 and self.ar_logo_errx <= 110 and self.range_ahead >= 0.8 and self.range_ahead != 999):
			print "CASE 2"
			self.move(0.1, 0.7, 1) #(speed, distance, forward)				
			self.travel_init_wp()
			
		#Case 3: camera extreme left of tgt (face tgt)
		elif (self.ar_logo_errx > 110 and self.range_ahead >= 0.8 and self.range_ahead != 999):
			print "CASE 3"
			#rotate 90 deg & move forward
			while (current_angle < 2):
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1)
				print "CURR ANGLE:", current_angle
				self.twist.angular.z = -16.0
				current_angle = current_angle + 1
				rate.sleep()
			
			self.move(0.1, 1.0, 1)
			
			current_angle = 0
			while (current_angle < 4):
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1)
				print "CURR ANGLE3:", current_angle
				self.twist.linear.x = 0
				self.twist.angular.z = 20.0
				current_angle = current_angle + 1
				rate.sleep()
				
			self.travel_init_wp()
			
		elif (self.range_ahead < 0.8 or self.range_ahead == 999):
			print "STOP"
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
		
			
	
if __name__ == "__main__":
	rospy.init_node('comp4')
	comp4 = Comp4()
	rospy.spin()
	
		

