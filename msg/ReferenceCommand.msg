std_msgs/Header header

uint8 horizontal_mode 	# horizontal control mode {0/no control (ignored), 1/position, 2/velocity, 3/angle}
float64 x		# horizontal_mode == 1: x position [m], horizontal_mode == 2: x velocity [m/s], horizontal_mode == 3: roll angle [deg]
float64 y		# horizontal_mode == 1: y position [m], horizontal_mode == 2: y velocity [m/s], horizontal_mode == 3: pitch angle [deg]
uint8 vertical_mode 	# vertical control mode {0/no control (ignored), 1/position, 2/velocity}
float64 z		# vertical_mode == 1: z position [m], vertical_mode == 2: z velocity [m/s]
uint8 heading_mode 	# heading control mode {0/no control (ignored), 3/angle, 4/rate}
float32 yaw 		# heading_mode == 3: yaw angle [deg], heading_mode == 4: yaw rate [deg/s]
