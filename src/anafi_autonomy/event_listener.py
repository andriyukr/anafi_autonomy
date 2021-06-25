#!/usr/bin/env python3

import olympe

from olympe.messages import mapper
from olympe.enums.mapper import button_event

from anafi_autonomy.msg import SkyControllerCommand

# Event listener		
class EventListener(olympe.EventListener):
	def __init__(self, skyctrl):
		self.skyctrl = skyctrl
		super().__init__(skyctrl.skyctrl)
		
	# RC buttons listener     
	@olympe.listen_event(mapper.grab_button_event()) # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_button_event
	def on_grab_button_event(self, event, scheduler):
		# button: 	0 = RTL, 1 = takeoff/land, 2 = reset camera, 3 = reset zoom
		# axis_button:	4 = max CCW yaw, 5 = max CW yaw, 6 = max trottle, 7 = min trottle
		# 		8 = min roll, 9 = max roll, 10 = min pitch, 11 = max pitch
		# 		12 = max camera down, 13 = max camera up, 14 = min zoom, 15 = max zoom
		if event.args["event"] == button_event.press:
			if event.args["button"] == 0: # RTL button
				self.skyctrl.msg_skycontroller.RTL = True
			elif event.args["button"] == 1: # takeoff/land button
				self.skyctrl.msg_skycontroller.takeoff_land = True
			elif event.args["button"] == 2: # reset camera button
				self.skyctrl.msg_skycontroller.reset_camera = True
			elif event.args["button"] == 3: # reset zoom button
				self.skyctrl.msg_skycontroller.reset_zoom = True
		   
      	# RC axis listener
	@olympe.listen_event(mapper.grab_axis_event()) # https://developer.parrot.com/docs/olympe/arsdkng_mapper.html#olympe.messages.mapper.grab_axis_event
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
		          
      	# All other events
	@olympe.listen_event()
	def default(self, event, scheduler):
		pass
