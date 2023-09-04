std_msgs/Header header

# UAV actions
uint8 drone_action 0	# drone action

# UAV movements
int8 drone_x		# x-axis increment {-1,0,1} []
int8 drone_y		# y-axis increment {-1,0,1} []
int8 drone_z		# z-axis increment {-1,0,1} []
int8 drone_yaw 		# yaw-axis increment {-1,0,1} []

# Gimbal movements
int8 gimbal_roll	# roll increment {-1,0,1} [deg]
int8 gimbal_pitch	# pitch increment {-1,0,1} [deg]
int8 gimbal_yaw		# yaw increment {-1,0,1} [deg]

# Camera actions
uint8 camera_action	# action {0=no action,1=take picture,2=start recording,3=stop recording}
int8 zoom		# zoom increment {-1,0,1} [X]
