#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from geometry_msgs.msg import PoseStamped
from tf_conversions.transformations import quaternion_from_euler
'''
Created on 27.10.2020

@author: Kevin Daun
'''

class BuildWaypointFrom2DPose(EventState):
	'''
	The robot drives in a straight line.

	-- threshold          float64              Maximum distance to waypoint to return reached.



	<= reached 						Robot drove for the specified duration.
	<= failed 						Received a new motion command while driving.

	'''

	def __init__(self,x=0,y=0,Yaw=0,frame_id='world'):
		'''
		Constructor
		'''
		super(BuildWaypointFrom2DPose, self).__init__(outcomes=['succeeded', 'failed'], output_keys=['waypoint'])


		self._x = x
		self._y = y
		self._yaw = yaw
		self._frame_id = frame_id
		self._succeeded = False
		self._failed = False


		
		
	def execute(self, userdata):
		if self._succeeded:
			return 'succeeded'
		elif self._failed:
			return 'failed'
		

			
	def on_enter(self, userdata):
		
		self._succeeded = False
		self._failed = False

		userdata.waypoint = PoseStamped()
		userdata.waypoint.header.frame_id = self._frame_id
		userdata.waypoint.pose.position.x = self._x
		userdata.waypoint.pose.position.y = self._y
		userdata.waypoint.pose.position.z = 0.0
		userdata.waypoint.pose.orientation = quaternion_from_euler(0, 0, self._yaw)
		self._succeeded = True

	def on_stop(self):
		pass
            
	def on_pause(self):
		pass
            
	def on_resume(self, userdata):
		pass
