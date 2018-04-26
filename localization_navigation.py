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

waypoints = [[(4.11, 7.33, 0.0), (0.0, 0.0, -0.15, 0.99)],
		[(3.82, 6.35, 0.0), (0.0, 0.0, -0.11, 0.99)],
		[(3.67, 5.75, 0.0), (0.0, 0.0, -0.17, 0.98)],
		[(3.19, 3.48, 0.0), (0.0, 0.0, -0.13, 0.99)],
		[(3.21, 2.71, 0.0), (0.0, 0.0, -0.011, 1.0)],		
		[(2.98, 1.94, 0.0), (0.0, 0.0, -0.1, 1.0)],
		[(2.77, -0.12, 0.0), (0.0, 0.0, -0.76, 0.65)],
		[(1.54, 0.07, 0.0), (0.0, 0.0, -0.75, 0.66)],
		[(0.46, 0.43, 0.0), (0.0, 0.0, -0.77, 0.64)],
		[(-0.45, 0.53, 0.0), (0.0, 0.0, -0.77, 0.64)],
		[(-0.2, 3.4, 0.0), (0.0, 0.0, 1.0, 0.06)],
		[(-0.12, 4.18, 0.0), (0.0, 0.0, 1.0, 0.07)],
		[(0.39, 7.84, 0.0), (0.0, 0.0, 0.98, 0.18)],
		[(1.39, 10.18, 0.0), (0.0, 0.0, 0.59, 0.81)],
		[(2.57, 10.02, 0.0), (0.0, 0.0, 0.57, 0.82)],
		[(3.65, 9.81, 0.0), (0.0, 0.0, 0.62, 0.79)],
		[(4.12, 9.66, 0.0), (0.0, 0.0, 0.66, 0.75)]
]

class Comp4:
	def __init__(self):	
		rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.caminfo_cb)
		rospy.Subscriber('/joy', Joy, self.joy_cb)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)
				
		self.cmd_vel_pub = rospy.Publisher('cmd_vel/velocityramp', Twist, queue_size=1)
		self.twist = Twist()

		self.service = rospy.ServiceProxy('global_localization', Empty)
		self.service()	
		
		self.state = "waiting"
		self.can_go = False #joy to start cmd_vel
		self.range_ahead = 0 #scan dist		
		self.time_loc = time.time() + 20.00 #~sec (timestamp) to localize 
		self.pose=None #store amcl msg.pose.pose (entire row)
		self.px = 0 #store amcl msg.pose.position.x bot current x pos
		self.py = 0	#store amcl msg.pose.position.y bot current y pos
		self.shortest_wp = 0 #starting wp from shortest dist
		self.goal_pos_counter = 0
		
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
		self.client.wait_for_server()
		
		"""
    STATES:
    				waiting 					(waiting for joy button A to be pressed)
    				localizing1				(localizing going in spiral circle)
    				find_shortest_wp	(finding closest dist wp to init loc)
    				travel_init_wp		(travelling to closest wp from init loc)
    				travel_wp					(travelling to all wp from init_wp)
    """
	
	def caminfo_cb(self, msg):
		self.K = np.array(msg.K).reshape(3,3)
		self.D = np.array(msg.D)
		
	def joy_cb(self, msg):
		print self.can_go
		if msg.buttons[0]:
			self.can_go = not self.can_go
			self.waiting()
				
	#get bot current pos 
	def amcl_cb(self, msg):
		self.pose = msg.pose.pose
		self.px = self.pose.position.x
		self.py = self.pose.position.y
	
	#turn waypoints into a movebase goal
	def goal_pose(self,pose):
		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.frame_id = 'map'
		goal_pose.target_pose.pose.position.x = pose[0][0]
		goal_pose.target_pose.pose.position.y = pose[0][1]
		goal_pose.target_pose.pose.position.z = pose[0][2]
		goal_pose.target_pose.pose.orientation.x = pose[1][0]
		goal_pose.target_pose.pose.orientation.y = pose[1][1]
		goal_pose.target_pose.pose.orientation.z = pose[1][2]
		goal_pose.target_pose.pose.orientation.w = pose[1][3]
		return goal_pose
		
	#STATES
	def waiting(self):
		if self.can_go:
			self.localizing2() # !!! SELECT TO TEST !!!
			print "LOCALIZING2"
  
  #localize2: with spin 	
	def localizing2(self):
		while time.time() < self.time_loc:
			
			self.twist.angular.z = 0.5
			self.twist.linear.x = 0
			self.cmd_vel_pub.publish(self.twist)
			
		print "FINISH CIRCLING ..."
		self.can_go = False
		self.find_shortest_wp()
				
	#find shortest wp from bot current pos
	def find_shortest_wp(self):
		d = []
		for i in range(0, len(waypoints)): #waypoints:
			d_temp = math.sqrt(((self.py - waypoints[i][0][1])**2) + ((self.px - waypoints[i][0][0])**2))
			#print d_temp
			d.append(d_temp)
			
		self.shortest_wp = np.argmin(d)
		print self.shortest_wp
		self.travel_init_wp()
		print "FINISH CALCULATING"		
	
	#travel to closest wp
	def travel_init_wp(self):
		init_pose = waypoints[self.shortest_wp] #get waypoint
		init_goal = self.goal_pose(init_pose) #create movebase goal
		self.client.send_goal(init_goal)
		self.client.wait_for_result()		
		self.travel_wp()
		print "FINISH MOVING"

	#travel to waypoints
	def travel_wp(self):
		#loop thru all waypoints
		goal_counter = 1
		
		while goal_counter <= len(waypoints):
			#start at idx after shortest_wp idx val
			self.goal_pos_counter = self.shortest_wp + goal_counter
			#if idx exceeds len(waypoints) 
			if self.goal_pos_counter >= len(waypoints):
				self.goal_pos_counter = self.goal_pos_counter - len(waypoints)
			
			#pass waypoint val 
			print waypoints[self.goal_pos_counter]
			wp_pose = waypoints[self.goal_pos_counter]
			goal = self.goal_pose(wp_pose)
			self.client.send_goal(goal)
			self.client.wait_for_result()
			
			goal_counter = goal_counter + 1
			
			print "FINISH GOALS"

if __name__ == "__main__":
	rospy.init_node('comp4')	
	comp4 = Comp4()
	rospy.spin()

# END ALL





			




