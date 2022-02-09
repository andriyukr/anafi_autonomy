#!/usr/bin/env python3

import olympe
import rospy
import math

from std_msgs.msg import Header
from geometry_msgs.msg import QuaternionStamped, Quaternion, Vector3Stamped
from tf.transformations import euler_from_quaternion

class EventListenerSkyController(olympe.EventListener):
	def __init__(self, skyctrl):
		self.skyctrl = skyctrl
		super().__init__(skyctrl.skyctrl)
	
	########## CALLBACKS ##########
	# SkyController buttons listener     
	@olympe.listen_event(olympe.messages.mapper.grab_button_event()) # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_button_event
	def on_grab_button_event(self, event, scheduler):
		# button: 	0 = RTL, 1 = takeoff/land, 2 = reset camera, 3 = reset zoom
		# axis_button:	4 = max CCW yaw, 5 = max CW yaw, 6 = max trottle, 7 = min trottle
		# 		8 = min roll, 9 = max roll, 10 = min pitch, 11 = max pitch
		# 		12 = max camera down, 13 = max camera up, 14 = min zoom, 15 = max zoom
		if event.args["event"] == olympe.enums.mapper.button_event.press:
			if event.args["button"] == 0: # RTL button
				self.skyctrl.msg_skycontroller.RTL = True
			elif event.args["button"] == 1: # takeoff/land button
				self.skyctrl.msg_skycontroller.takeoff_land = True
			elif event.args["button"] == 2: # reset camera button
				self.skyctrl.msg_skycontroller.reset_camera = True
			elif event.args["button"] == 3: # reset zoom button
				self.skyctrl.msg_skycontroller.reset_zoom = True
		   
      	# SkyController axes listener
	@olympe.listen_event(olympe.messages.mapper.grab_axis_event()) # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_axis_event
	def on_grab_axis_event(self, event, scheduler):	
		# axis: 	0 = yaw, 1 = z, 2 = y, 3 = x, 4 = camera, 5 = zoom
		if event.args["axis"] == 0: # yaw
			self.skyctrl.msg_skycontroller.yaw = event.args["value"]
		if event.args["axis"] == 1: # z
			self.skyctrl.msg_skycontroller.z = event.args["value"]
		if event.args["axis"] == 2: # y/pitch
			self.skyctrl.msg_skycontroller.y = event.args["value"]
		if event.args["axis"] == 3: # x/roll
			self.skyctrl.msg_skycontroller.x = event.args["value"]
		if event.args["axis"] == 4: # camera
			self.skyctrl.msg_skycontroller.camera = event.args["value"]
		if event.args["axis"] == 5: # zoom
			self.skyctrl.msg_skycontroller.zoom = event.args["value"]
	
	########## PUBLISHERS ##########	
	@olympe.listen_event(olympe.messages.skyctrl.SkyControllerState.AttitudeChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_skyctrl_skycontrollerstate.html#olympe.messages.skyctrl.SkyControllerState.AttitudeChanged
	def onAttitudeChanged(self, event, scheduler):	
		attitude = event.args
		
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = '/world'
			
		msg_attitude = QuaternionStamped()
		msg_attitude.header = header
		msg_attitude.quaternion = Quaternion(attitude['q1'], -attitude['q2'], -attitude['q3'], attitude['q0'])
		self.skyctrl.pub_skyctrl_attitude.publish(msg_attitude)
			
		quaternion = [attitude['q1'], -attitude['q2'], -attitude['q3'], attitude['q0']]
		(roll, pitch, yaw) = euler_from_quaternion(quaternion)
		msg_rpy = Vector3Stamped()
		msg_rpy.header = header
		msg_rpy.vector.x = roll*180/math.pi
		msg_rpy.vector.y = pitch*180/math.pi
		msg_rpy.vector.z = yaw*180/math.pi
		self.skyctrl.pub_skyctrl_rpy.publish(msg_rpy)
			
      	# All other events
	@olympe.listen_event()
	def default(self, event, scheduler):
		pass
