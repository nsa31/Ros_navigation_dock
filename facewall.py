#!/usr/bin/env python

# BEGIN ALL 
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

class Comp4:
	def __init__(self):	
		rospy.Subscriber('/scan', LaserScan, self.scan_cb)				
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
		#self.cmd_vel_pub = rospy.Publisher('cmd_vel/velocityramp', Twist, queue_size=1) #VELOCITY RAMP
		
		self.twist = Twist()
		self.range_ahead = 999 	
		self.time_loc = time.time() + 20.00 
		self.pose = None 
		self.px = 0 
		self.py = 0	
		self.shortest_wp = 0 
		self.goal_pos_counter = 0 
		self.laser_scan_arr = [] 
		self.min_laser_scan_arr = 0 
		self.theta = 0 

		self.theta1 = 0
		self.theta2 = 0
		
		self.num = 0
		self.denom = 0
		
		self.face_wall()		
		
	def scan_cb(self, msg):
		depth = []
		for dist in msg.ranges:
			if not np.isnan(dist):
				depth.append(dist)
		if len(depth) == 0:
			self.range_ahead = 999
		else:
			self.range_ahead = min(depth)
	
	def my_cb(self):
		print "Timer called at "
	
	def face_wall(self):
		print "FACE WALLLLLLLLLLLLLLLLLL"
		self.twist.linear.x=0
		self.twist.linear.y=0
		self.twist.linear.z=0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
				
		current_angle = 0
		rate = rospy.Rate(0.5) 
		while (current_angle < 12): #21 (class) 19(class) 18 11(sim)
			self.cmd_vel_pub.publish(self.twist) 
			rospy.sleep(1)
			print "CURR ANGLE:", current_angle
			self.twist.angular.z = 1.0 #1.3
			current_angle = current_angle + 1
			self.laser_scan_arr.append(self.range_ahead) #store laser scan data
			print "LASER SCAN:", self.laser_scan_arr
			rate.sleep()
		self.min_laser_scan_arr = np.argmin(self.laser_scan_arr)
		print "MIN INDEX", self.min_laser_scan_arr
		
		
		#rotate
		max_speed = 3.0
		current_angle2 = 0
		current_angle3 = 0
		self.num = int(np.floor(self.min_laser_scan_arr/3.0))
		print "num", self.num
		if (self.num <= 3.0): 
			while(current_angle2 < self.num):
				print "i :", current_angle2
				self.twist.angular.z = 3.0 
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1)
				current_angle2 = current_angle2 + 1
				rate.sleep()
			self.denom= self.min_laser_scan_arr - (self.num*3.0)
			print "denom", self.denom
			while (current_angle3 < 1):
				self.twist.angular.z = self.denom
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1)
				current_angle3 = current_angle3 + 1
				rate.sleep()
							
		else:
			self.num = int(np.floor(np.abs(self.min_laser_scan_arr - 20.0)/3.0))
			print "ELSE num:", self.num
			while(current_angle2 < self.num):
				print "ELSE i :", current_angle2
				self.twist.angular.z = -3.0 #rotate opp dir (cw)
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1)
				current_angle2 = current_angle2 + 1
				rate.sleep()
			self.denom = np.abs(self.min_laser_scan_arr-20.0) - (self.num*3.0)
			print "ELSE denom", self.denom
			while (current_angle3 < 1):
				self.twist.angular.z = -self.denom
				self.cmd_vel_pub.publish(self.twist)
				rospy.sleep(1)
				current_angle3 = current_angle3 + 1
				rate.sleep()
				
		
if __name__ == "__main__":
	rospy.init_node('comp4')	
	comp4 = Comp4()
	rospy.spin()



