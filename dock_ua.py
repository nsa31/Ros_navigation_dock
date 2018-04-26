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

class detect_logo(object):
	def __init__(self):
		rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.caminfo_cb)
		rospy.Subscriber('camera/rgb/image_raw', Image, self.process)    
		self.bridge = cv_bridge.CvBridge() 
		self.template = cv2.imread('ua_small.png') 
		self.template = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)
		self.template = cv2.Canny(self.template, 50, 60) 
		self.th, self.tw = self.template.shape[:2]
		self.threshold = 0.20 
		self.logo_found = False 
		self.K = None
		self.D = None
		self.errx = 0
		
	def caminfo_cb(self, msg):
		self.K = np.array(msg.K).reshape(3,3)
		self.D = np.array(msg.D)			
	
	def process(self, msg):	
		img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') 
		img = imutils.resize(img, width = int(img.shape[1] * 0.75))
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		found = None
		for scale in np.linspace(0.1, 0.75, 25)[::-1]:
				resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
				r = gray.shape[1] / float(resized.shape[1])
				if resized.shape[0] < self.th or resized.shape[1] < self.tw:
					  break
				edged = cv2.Canny(resized, 50, 60)
				result = cv2.matchTemplate(edged, self.template, cv2.TM_CCOEFF_NORMED)
				(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
				if found is None or maxVal > found[0]:
					  found = (maxVal, maxLoc, r)

		(maxVal, maxLoc, r) = found
		(startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
		(endX, endY) = (int((maxLoc[0] + self.tw) * r), int((maxLoc[1] + self.th) * r))
		
		if maxVal > self.threshold:
			cv2.rectangle(img, (startX, startY), (endX, endY), (0, 0, 255), 2)
			cv2.circle(img, (startX, startY), 5, (255,255,0), -1)
			h, w = img.shape[:2]
			wx = w/2
			cv2.circle(img, (wx, endY/2),5, (0,0,157), -1)
			
			#error
			self.errx = (endX/2) - wx
			
			self.logo_found = True
			self.return_ua_val()
		else:
			#print self.logo_found
			self.logo_found = False
			self.return_ua_val()
			
		#cv2.imshow("UAimg", img)
		#cv2.waitKey(3)		
		
	
	def return_ua_errx(self):
		return self.errx
		
	def return_ua_val(self):
		return self.logo_found
		

class Comp4(object):
	def __init__(self):
		self.cmd_vel_pub = rospy.Publisher('cmd_vel/velocityramp', Twist, queue_size=1)
		rospy.Subscriber('/scan', LaserScan, self.scan_cb)
		self.twist = Twist()
		self.ua_logo_flag = False
		self.ua_logo_errx = 0
		self.ar_logo_flag = False
		self.ar_logo_errx = 0
		self.dock_counter = 0
		self.range_ahead = 999
		self.travel_init_wp()
		
        
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
  
	def scan_cb(self, msg):
		depth = []
		for dist in msg.ranges:
			if not np.isnan(dist):
				depth.append(dist)
		if len(depth) == 0:
			self.range_ahead = 999
		else:
			self.range_ahead = min(depth)
		
	def travel_init_wp(self):
		print "TRAVELED TO INIT WP"	
		counter = 0
		counter_true = 0 
		while (counter<5):
			self.ua_obj = detect_logo()
			rospy.sleep(2)
			self.ua_logo_flag = self.ua_obj.return_ua_val()
			print self.ua_logo_flag
			if (self.ua_logo_flag == True):
				self.ua_logo_errx = self.ua_obj.return_ua_errx()
				counter_true = counter_true + 1
				print self.ua_logo_errx, "counter_true while", counter_true
			elif (self.ua_logo_flag == False):
				print "NO UA LOGO"
			counter = counter+ 1
			
		if counter_true >=2:
			print "go to docking"
			self.docking() 
		else:
			print "GO CHECK AR"
		
				
	def docking(self):
		print "DOCKINGGGGGGGGGGGGGGGG"
		# TEST 3 CASES HERE EACH WITH DIFF MOVE & ROTATE 
		#Case 1: camera extreme left of tgt (face tgt)
		current_angle = 0
		rate = rospy.Rate(0.5)
		if (self.ua_logo_errx > -50 and self.range_ahead >= 0.8 and self.range_ahead != 999):
		 	print "CASE 1"
			#rotate 90 deg & move forward
			while (current_angle < 2):
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1)
				print "CURR ANGLE:", current_angle
				self.twist.angular.z = -16.0 #4.8
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
			
		#Case 2: camera mid (face tgt)
		elif (self.ua_logo_errx >= -130 and self.ua_logo_errx <= -50 and self.range_ahead >= 0.8 and self.range_ahead != 999):
			print "CASE 2"
			self.move(0.1, 0.7, 1) #(speed, dist, forward)		
			self.travel_init_wp()
			
		#Case 3: camera extreme right of tgt (face tgt)
		elif (self.ua_logo_errx < -130 and self.range_ahead >= 0.8 and self.range_ahead != 999):
			print "CASE 3"
			#rotate 90 deg & move forward
			while (current_angle < 2):
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1)
				print "CURR ANGLE:", current_angle
				self.twist.angular.z = 16.0
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

				
		elif (self.range_ahead < 0.8 or self.range_ahead == 999):
			print "STOP"
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.0
						
	
if __name__ == "__main__":
	rospy.init_node('comp4')
	comp4 = Comp4()
	rospy.spin()
	
		

