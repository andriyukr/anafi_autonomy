#!/usr/bin/env python3

import olympe
import rospy
import math

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix
	
class EventListenerAnafi(olympe.EventListener):
	def __init__(self, drone):
		self.drone = drone
		super().__init__(drone.drone)
	
	def show_motors(self, motors):
		show = ' '
		for i in range(4):
			if motors&(1<<i):
			    show += (chr(8598 + i) + ' ')
		return show
	
	########## FATAL ERRORS ##########
	@olympe.listen_event(olympe.messages.ardrone3.SettingsState.MotorErrorStateChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_settings_state.html#olympe.messages.ardrone3.SettingsState.MotorFlightsStatusChanged
	def onMotorErrorStateChanged(self, event, scheduler):
		motor_error = event.args
		if motor_error['motorError'] is not olympe.enums.ardrone3.SettingsState.MotorErrorStateChanged_MotorError.noError:
			rospy.logfatal('Motor Error: motors = %s, error = %s', self.show_motors(motor_error['motorIds']), motor_error['motorError'].name)
			
	@olympe.listen_event(olympe.messages.drone_manager.authentication_failed()) # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.authentication_failed
	def on_authentication_failed(self, event, scheduler):
		rospy.logfatal('Authentication failed because of a wrong key (passphrase)')
		
	@olympe.listen_event(olympe.messages.drone_manager.connection_refused()) # https://developer.parrot.com/docs/olympe/arsdkng_drone_manager.html#olympe.messages.drone_manager.connection_refused
	def on_connection_refused(self, event, scheduler):
		rospy.logfatal('Connection refused by the drone because another peer is already connected')		
		
	########## ERRORS ##########
	@olympe.listen_event(olympe.messages.battery.alert()) # https://developer.parrot.com/docs/olympe/arsdkng_battery.html#olympe.messages.battery.alert
	def onBatteryAlert(self, event, scheduler):
		alert = event.args	
		if alert['level'] is olympe.enums.battery.alert_level.critical:	
			rospy.logerr_throttle(1, "Battery Allert: " + alert['alert'].name)
		if alert['level'] is olympe.enums.battery.alert_level.warning:	
			rospy.logwarn_throttle(60, "Battery Allert: " + alert['alert'].name)
			
	@olympe.listen_event(olympe.messages.gimbal.alert()) # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.alert
	def onGimbalAlert(self, event, scheduler):
		alert = event.args
		for error in olympe.enums.gimbal.error:
			if (1<<error.value) & alert['error']:
				rospy.logerr("Gimbal Allert: " + error.name)
				
	@olympe.listen_event(olympe.messages.user_storage.format_result()) # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.format_result
	def on_format_result(self, event, scheduler):
		format_result = event.args['result'] # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.enums.user_storage.formatting_result
		if format_result is olympe.enums.user_storage.formatting_result.error:
			rospy.logerr('Formatting failed')
		if format_result is olympe.enums.user_storage.formatting_result.denied:
			rospy.logwarn('Formatting was denied')
		if format_result is olympe.enums.user_storage.formatting_result.success:
			rospy.loginfo('Formatting succeeded')
	
   	########## WARNINGS ##########
	@olympe.listen_event(olympe.messages.ardrone3.PilotingState.AlertStateChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.AlertStateChanged
	def onAlertStateChanged(self, event, scheduler):
		alert_state = event.args['state']
		if alert_state is not olympe.enums.ardrone3.PilotingState.AlertStateChanged_State.none and alert_state is not olympe.enums.ardrone3.PilotingState.AlertStateChanged_State.user:	
			rospy.logwarn_throttle(60, "Alert: " + alert_state.name) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.AlertStateChanged_State
   	
	@olympe.listen_event(olympe.messages.ardrone3.PilotingState.ForcedLandingAutoTrigger()) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.ForcedLandingAutoTrigger
	def onForcedLandingAutoTrigger(self, event, scheduler):
		forced_landing = event.args['reason']
		if forced_landing is not olympe.enums.ardrone3.PilotingState.ForcedLandingAutoTrigger_Reason.NONE:	
			rospy.logwarn_throttle(60, "Forced Landing Reason: " + forced_landing.name) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.ForcedLandingAutoTrigger_Reason
			
	@olympe.listen_event(olympe.messages.ardrone3.PilotingState.HoveringWarning()) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.HoveringWarning
	def onHoveringWarning(self, event, scheduler):
		hovering_warning = event.args
		if (self.drone.state == "HOVERING" or self.drone.state == "FLYING"):
			if hovering_warning['no_gps_too_dark']:
				rospy.logwarn_throttle(60, "Hovering Warning: the drone doesn’t have a GPS fix AND there is not enough light")
			if hovering_warning['no_gps_too_high']:
				rospy.logwarn_throttle(60, "Hovering Warning: the drone doesn’t have a GPS fix AND is flying too high")
			
	@olympe.listen_event(olympe.messages.ardrone3.PilotingState.VibrationLevelChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.VibrationLevelChanged
	def onVibrationLevelChanged(self, event, scheduler):
		vibration_level = event.args['state']
		if vibration_level is not olympe.enums.ardrone3.PilotingState.VibrationLevelChanged_State.ok:	
			rospy.logwarn_throttle(60, "Vibration Level: " + vibration_level.name) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.VibrationLevelChanged_State
			
	@olympe.listen_event(olympe.messages.ardrone3.PilotingState.WindStateChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.WindStateChanged
	def onWindStateChanged(self, event, scheduler):
		wind_state = event.args['state']
		if wind_state is not olympe.enums.ardrone3.PilotingState.WindStateChanged_State.ok:	
			rospy.logwarn_throttle(60, "Wind State: " + wind_state.name) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.enums.ardrone3.PilotingState.WindStateChanged_State
					
	@olympe.listen_event(olympe.messages.common.CommonState.LinkSignalQuality()) # https://developer.parrot.com/docs/olympe/arsdkng_common_common.html#olympe.messages.common.CommonState.LinkSignalQuality
	def onAlertStateChanged(self, event, scheduler):
		link_quality = event.args['value']
		if (link_quality>>6)&1:
			rospy.logwarn_throttle(60, "4G interference coming from the smartphone")
		if (link_quality>>7)&1:
			rospy.logwarn_throttle(60, "Radio link is perturbed by external elements")
			
	@olympe.listen_event(olympe.messages.common.CommonState.MassStorageInfoStateListChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_common_common.html#olympe.messages.common.CommonState.MassStorageInfoStateListChanged
	def onMassStorageInfoStateListChanged(self, event, scheduler):
		mass_storage = event.args
		rospy.loginfo_once('Mass storage info: id = %i, size = %iMB, used = %iMB', mass_storage['mass_storage_id'], mass_storage['size'], mass_storage['used_size'])
		if mass_storage['plugged'] == 0:
			rospy.logwarn_throttle(60, 'Mass storage is not plugged')
		if mass_storage['full'] == 1:
			rospy.logwarn_throttle(60, 'Mass storage is full')
			
	@olympe.listen_event(olympe.messages.common.CommonState.SensorsStatesListChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_common_common.html#olympe.messages.common.CommonState.SensorsStatesListChanged
	def onSensorsStatesListChanged(self, event, scheduler):
		sensor = event.args
		if sensor['sensorState'] == 0:
			rospy.logwarn('%s is NOT OK', sensor['sensorName'].name)
			
	@olympe.listen_event(olympe.messages.rth.home_reachability()) # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.home_reachability
	def on_home_reachability(self, event, scheduler):
		home_reachability = event.args['status'] # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.enums.rth.home_reachability
		if home_reachability is olympe.enums.rth.home_reachability.unknown:
			rospy.logwarn('Home reachability is unknown')
		if home_reachability is olympe.enums.rth.home_reachability.reachable:
			rospy.loginfo('Home is reachable')
		if home_reachability is olympe.enums.rth.home_reachability.critical:
			rospy.logwarn('Home is still reachable but won’t be if rth is not triggered now')
		if home_reachability is olympe.enums.rth.home_reachability.not_reachable:
			rospy.logwarn('Home is not reachable')
			
	@olympe.listen_event(olympe.messages.rth.rth_auto_trigger()) # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.rth_auto_trigger
	def on_home_reachability(self, event, scheduler):
		rth_auto_trigger = event.args['reason'] # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.enums.rth.auto_trigger_reason
		if rth_auto_trigger is olympe.enums.rth.auto_trigger_reason.battery_critical_soon:
			rospy.logwarn_throttle(60, 'Battery will soon be critical')
			
	########## INFO ##########
	@olympe.listen_event(olympe.messages.ardrone3.PilotingState.NavigateHomeStateChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.NavigateHomeStateChanged
	def onNavigateHomeStateChanged(self, event, scheduler):
		navigate_home_state = event.args
		rospy.loginfo("Navigate Home State: state = %s, reason = %s", navigate_home_state['state'].name, navigate_home_state['reason'].name)
	
	@olympe.listen_event(olympe.messages.common.MavlinkState.MavlinkFilePlayingStateChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.MavlinkState.MavlinkFilePlayingStateChanged
	def onMissionItemExecuted(self, event, scheduler):
		rospy.loginfo('FlightPlan state is %s', event.args['state'].name) # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.MavlinkState.MavlinkFilePlayingStateChanged
	
	@olympe.listen_event(olympe.messages.common.MavlinkState.MissionItemExecuted()) # https://developer.parrot.com/docs/olympe/arsdkng_common_mavlink.html#olympe.messages.common.MavlinkState.MissionItemExecuted
	def onMissionItemExecuted(self, event, scheduler):
		rospy.loginfo('Mission item #%i executed', event.args['idx'])
		
	#@olympe.listen_event(olympe.messages.drone_manager.connection_state()) #
	#def on_connection_state(self, event, scheduler):
	#	rospy.loginfo("connection_state: " + str(event.args))
	
	@olympe.listen_event(olympe.messages.follow_me.state()) # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.messages.follow_me.state
	def on_follow_me_state(self, event, scheduler):
		follow_me = event.args
		if follow_me['mode'] != olympe.enums.follow_me.mode.none:
			rospy.loginfo('FollowMe state: mode=%s, behavior=%s, animation=%s', follow_me['mode'].name, follow_me['behavior'].name, follow_me['animation'].name)
		
	@olympe.listen_event(olympe.messages.gimbal.calibration_result()) # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.calibration_result
	def on_gimbal_calibration_result(self, event, scheduler):
		calibration_result = event.args
		rospy.loginfo('Gimbal calibration result: ' + calibration_result['result'].name)
		
	@olympe.listen_event(olympe.messages.move.info()) # https://developer.parrot.com/docs/olympe/arsdkng_move.html#olympe.messages.move.info
	def on_move_info(self, event, scheduler):
		move_info = event.args
		rospy.loginfo('Move info: ' + str(move_info))
	
	@olympe.listen_event(olympe.messages.rth.state()) # https://developer.parrot.com/docs/olympe/arsdkng_rth.html#olympe.messages.rth.state
	def on_rth_state(self, event, scheduler):
		state = event.args
		rospy.loginfo('RTH: state=%s, reason=%s', state['state'].name, state['reason'].name)
		
	@olympe.listen_event(olympe.messages.user_storage.format_progress()) # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.format_progress
	def on_format_progress(self, event, scheduler):
		format_progress = event.args
		rospy.loginfo('Formatting --> %s (%i%%)', format_progress['step'].name, format_progress['percentage'])
		
	@olympe.listen_event(olympe.messages.user_storage.info()) # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.info
	def on_user_storage_info(self, event, scheduler):
		capacity = event.args['capacity']
		available_bytes = self.drone.drone.get_state(olympe.messages.user_storage.monitor)['available_bytes'] # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.monitor
		rospy.loginfo_once('Available space: %.1f/%.1fGB', available_bytes/(2**30), capacity/(2**30))
			
	########## DEBUG ##########
	@olympe.listen_event(olympe.messages.gimbal.calibration_state()) # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.calibration_state
	def on_gimbal_calibration_state(self, event, scheduler):
		calibration_state = event.args
		rospy.logdebug('Gimbal calibration state: ' + calibration_state['state'].name)
	
	########## PUBLISHERS ##########
	@olympe.listen_event(olympe.messages.camera.zoom_level()) # https://developer.parrot.com/docs/olympe/arsdkng_camera.html#olympe.messages.camera.zoom_level
	def onZoomLevel(self, event, scheduler):
		self.drone.pub_zoom.publish(event.args['level'])
	
	@olympe.listen_event(olympe.messages.ardrone3.GPSState.NumberOfSatelliteChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_gps.html#olympe.messages.ardrone3.GPSState.NumberOfSatelliteChanged
	def onNumberOfSatelliteChanged(self, event, scheduler):
		self.drone.pub_gps_satellites.publish(event.args['numberOfSatellite'])
		
	@olympe.listen_event(olympe.messages.ardrone3.PilotingState.AltitudeChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.AltitudeChanged
	def onAltitudeChanged(self, event, scheduler):
		self.drone.pub_altitude_above_TO.publish(event.args['altitude'])
		
	@olympe.listen_event(olympe.messages.ardrone3.PilotingState.AttitudeChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.AttitudeChanged
	def onAttitudeChanged(self, event, scheduler): # it is publishes at lower rate (5Hz) than 'pub_rpy' (30Hz) but has higher reaction time (approx. 100ms faster)
		attitude = event.args
		msg_attitude = Vector3Stamped()
		msg_attitude.header.stamp = rospy.Time.now()
		msg_attitude.header.frame_id = '/world'
		msg_attitude.vector.x = attitude['roll']*180/math.pi
		msg_attitude.vector.y = -attitude['pitch']*180/math.pi
		msg_attitude.vector.z = -attitude['yaw']*180/math.pi
		self.drone.pub_rpy_slow.publish(msg_attitude)
	
	@olympe.listen_event(olympe.messages.ardrone3.PilotingState.GpsLocationChanged()) # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.PilotingState.GpsLocationChanged
	def onGpsLocationChanged(self, event, scheduler):
		gps_location = event.args
		msg_gps_location = NavSatFix()
		msg_gps_location.header.stamp = rospy.Time.now()
		msg_gps_location.header.frame_id = '/world'
		msg_gps_location.status.status = (msg_gps_location.status.STATUS_FIX if self.drone.gps_fixed else msg_gps_location.status.STATUS_NO_FIX) # https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatStatus.html
		if self.drone.model == "4k" or self.drone.model == "thermal":
			msg_gps_location.status.service = msg_gps_location.status.SERVICE_GPS + msg_gps_location.status.SERVICE_GLONASS;
		if self.drone.model == "usa" or self.drone.model == "ai":
			msg_gps_location.status.service = msg_gps_location.status.SERVICE_GPS + msg_gps_location.status.SERVICE_GLONASS + msg_gps_location.status.SERVICE_GALILEO;
		msg_gps_location.latitude = (gps_location['latitude'] if gps_location['latitude'] != 500 else float('nan'))
		msg_gps_location.longitude = (gps_location['longitude'] if gps_location['longitude'] != 500 else float('nan'))
		msg_gps_location.altitude = (gps_location['altitude'] if gps_location['altitude'] != 500 else float('nan'))
		msg_gps_location.position_covariance[0] = (math.sqrt(gps_location['latitude_accuracy']) if gps_location['latitude_accuracy'] > 0 else float('nan'))
		msg_gps_location.position_covariance[4] = (math.sqrt(gps_location['longitude_accuracy']) if gps_location['longitude_accuracy'] > 0 else float('nan'))
		msg_gps_location.position_covariance[8] = (math.sqrt(gps_location['altitude_accuracy']) if gps_location['altitude_accuracy'] > 0 else float('nan'))
		msg_gps_location.position_covariance_type = msg_gps_location.COVARIANCE_TYPE_DIAGONAL_KNOWN;		
		self.drone.pub_gps_location.publish(msg_gps_location)
		
	@olympe.listen_event(olympe.messages.battery.voltage()) # https://developer.parrot.com/docs/olympe/arsdkng_battery.html#olympe.messages.battery.voltage
	def onBatteryVoltage(self, event, scheduler):
		self.drone.pub_battery_voltage.publish(event.args['voltage']/1000)
		
	@olympe.listen_event(olympe.messages.follow_me.target_trajectory()) # https://developer.parrot.com/docs/olympe/arsdkng_followme.html#olympe.messages.follow_me.target_trajectory
	def on_target_trajectory(self, event, scheduler):
		trajectory = event.args
		rospy.loginfo('target_trajectory: ' + str(trajectory))
		msg_trajectory = TargetTrajectory()
		msg_trajectory.header.stamp = rospy.Time.now()
		msg_trajectory.header.frame_id = '/world'
		msg_trajectory.latitude = trajectory['latitude']
		msg_trajectory.longitude = trajectory['longitude']
		msg_trajectory.altitude = trajectory['altitude']
		msg_trajectory.north_speed = trajectory['north_speed']
		msg_trajectory.east_speed = trajectory['east_speed']
		msg_trajectory.down_speed = trajectory['down_speed']
		self.drone.pub_target_trajectory.publish(msg_trajectory)
		
	@olympe.listen_event(olympe.messages.gimbal.attitude()) # https://developer.parrot.com/docs/olympe/arsdkng_gimbal.html#olympe.messages.gimbal.attitude
	def onGimbalAttitude(self, event, scheduler):
		gimbal = event.args
		msg_gimbal = Vector3Stamped()
		msg_gimbal.header.stamp = rospy.Time.now()
		msg_gimbal.header.frame_id = '/world'
		msg_gimbal.vector.x = gimbal['roll_absolute']
		msg_gimbal.vector.y = -gimbal['pitch_absolute']
		msg_gimbal.vector.z = -gimbal['yaw_absolute']
		self.drone.pub_gimbal_absolute.publish(msg_gimbal)
		msg_gimbal.header.frame_id = 'body'
		msg_gimbal.vector.x = gimbal['roll_relative']
		msg_gimbal.vector.y = -gimbal['pitch_relative']
		msg_gimbal.vector.z = -gimbal['yaw_relative']
		self.drone.pub_gimbal_relative.publish(msg_gimbal)
		
	@olympe.listen_event(olympe.messages.user_storage.monitor()) # https://developer.parrot.com/docs/olympe/arsdkng_user_storage.html#olympe.messages.user_storage.monitor
	def on_user_storage_monitor(self, event, scheduler):
		self.drone.pub_media_available.publish(event.args['available_bytes'])

      	# All other events
	@olympe.listen_event()
	def default(self, event, scheduler):
		pass
