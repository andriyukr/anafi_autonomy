# Usage: 
# 	- connection through Skycontroller [recommended]:
# 		ros2 launch anafi_autonomy anafi_autonomy_launch.py
#	- direct connection to Anafi:
# 		ros2 launch anafi_autonomy anafi_autonomy_launch.py ip:='192.168.42.1' model:='ai'
#	- connection to the simulated drone in Sphinx:
# 		ros2 launch anafi_autonomy anafi_autonomy_launch.py ip:='10.202.0.1' model:='ai'

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
	# args that can be set from the command line or a default will be used
	namespace_arg = DeclareLaunchArgument(
		'namespace', 
		default_value='anafi', 
		description='Namespace for this drone')
	ip_arg = DeclareLaunchArgument(
		'ip', 
		default_value='192.168.53.1',  # Anafi: '192.168.42.1', SkyController: '192.168.53.1', Sphinx: '10.202.0.1'
		description='IP address of the device')
	model_arg = DeclareLaunchArgument(
		'model', 
		default_value='ai',  # {'4k', 'thermal', 'usa', 'ai'}
		description='Model of the drone')

	anafi_include = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
			os.path.join(get_package_share_directory('anafi_ros_nodes')), 
			'/anafi_launch.py'
		]),
		launch_arguments={
			'drone/model': LaunchConfiguration('model'),
			'device/ip': LaunchConfiguration('ip')
		}.items()
	)
	
	autonomy_config = os.path.join(
		get_package_share_directory('anafi_autonomy'),
		'config/params_autonomy.yaml'
	)
	
	controller_pid_config = os.path.join(
		get_package_share_directory('anafi_autonomy'),
		'config/params_controller_pid.yaml'
	)

	trajectory_config = os.path.join(
		get_package_share_directory('anafi_autonomy'),
		'config/params_trajectory.yaml'
	)
	
	autonomy_node = Node(
		package='anafi_autonomy',
		namespace=LaunchConfiguration('namespace'),
		executable='autonomy',
		name='autonomy',
		output="screen",
		emulate_tty=True,
		arguments=['--ros-args', '--log-level', 'INFO'],
		parameters=[autonomy_config, controller_pid_config,
			{'fixed_frame': False}, 	# fixed frame for velocity commands
			{'flightplan_file': "~/ros2_ws/install/anafi_autonomy/share/anafi_autonomy/missions/test.mavlink"}, 	# absolute path to the FlightPlan file
			{'followme_mode': 2}, 		# followMe mode: 1 = look at the target without moving automatically, 2 = follow the target keeping the same vector, 3 = follow the target keeping the same orientation to its direction, 4 = follow the target as it was held by a leash
			{'hand_launch': False}, 	# enable hand launched takeoff
			{'landing_control': False}, 	# enable control during landing
			{'mission_type': 0}, 		# mission type: 0 = flight plan, 1 = follow me
			{'takingoff_control': False}, # enable control during takeoff
			{'world_frame': False}, 	# yaw in world frame
			{'bounds/x/min': -10.0}, 	# min x bound
			{'bounds/x/max': 10.0}, 	# max x bound
			{'bounds/y/min': -10.0}, 	# min y bound 
			{'bounds/y/max': 10.0}, 	# max y bound
			{'bounds/z/min': 0.0}, 		# min z bound 
			{'bounds/z/max': 2.0}, 		# max z bound
			{'gains/position/p': 2.0}, 	# position proportional gain	
			{'gains/position/i': 1.0}, 	# position integral gain
			{'gains/position/d': 1.0}, 	# position derivative gain
			{'gains/position/max_i': 0.1}, 	# position max integral component
			{'gains/velocity/p': 20.0}, 	# velocity proportional gain
			{'gains/velocity/i': 1.0}, 	# velocity integral gain
			{'gains/velocity/d': 3.0}, 	# velocity derivative gain
			{'gains/velocity/max_i': 0.5}, 	# velocity max integral component
			{'gains/yaw/p': 70.0} 		# yaw proportional gain
		]
	)
		
	trajectory_node = Node(
		package='anafi_autonomy',
		namespace=LaunchConfiguration('namespace'),
		executable='trajectory',
		name='trajectory',
		output="screen",
		emulate_tty=True,
		arguments=['--ros-args', '--log-level', 'INFO'],
		parameters=[trajectory_config,
			{'trajectory': 0}, 	# trajectory type: 0 = no trajectory, 1 = hover at (0, 0, 1, yaw_d), 2 = defined by user with (x_d, y_d, z_d, yaw_d)
			{'desired/x': 0.0}, 	# desired x position
			{'desired/y': 0.0}, 	# desired y position
			{'desired/z': 1.0}, 	# desired z position
			{'desired/yaw': 0.0} 	# desired yaw orientation		
		]
	)
	
	rqt_image_view_node = Node(
		package='rqt_image_view',
		namespace=LaunchConfiguration('namespace'),
		executable='rqt_image_view',
		name='rqt_image_view',
		arguments=[['/', LaunchConfiguration('namespace'), '/camera/image']]
	)
	
	rqt_reconfigure_node = Node(
		package='rqt_reconfigure',
		namespace=LaunchConfiguration('namespace'),
		executable='rqt_reconfigure',
		name='rqt_reconfigure'
	)

	return LaunchDescription([
		namespace_arg,
		ip_arg,
		model_arg,
		anafi_include,
		autonomy_node,
		trajectory_node,
		rqt_image_view_node,
		rqt_reconfigure_node
	])
