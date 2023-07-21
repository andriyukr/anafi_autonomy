# Usage: 
# 	- connection through Skycontroiller [recommended]:
# 		ros2 launch anafi_autonomy safe_anafi_launch.py ip:='192.168.53.1' model:='ai'
#	- direct connection to Anafi:
# 		ros2 launch anafi_autonomy safe_anafi_launch.py ip:='192.168.42.1' model:='ai'
#	- connection to the simulated drone in Sphinx:
# 		ros2 launch anafi_autonomy safe_anafi_launch.py ip:='10.202.0.1' model:='ai'

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
	
	safe_anafi_config = os.path.join(
		get_package_share_directory('anafi_autonomy'),
		'config/params_safe_anafi.yaml'
	)
	
	trajectory_config = os.path.join(
		get_package_share_directory('anafi_autonomy'),
		'config/params_trajectory.yaml'
	)
	
	safe_anafi_node = Node(
		package='anafi_autonomy',
		namespace=LaunchConfiguration('namespace'),
		executable='safe_anafi',
		name='safe_anafi',
		output="screen",
		emulate_tty=True,
		arguments=['--ros-args', '--params-file', safe_anafi_config, '--log-level', 'INFO'],
		parameters=[
			{'bounds/x/min': -1.0},
			{'bounds/x/max': 1.0},
			{'bounds/y/min': -1.0}, 
			{'bounds/y/max': 1.0},
			{'bounds/z/min': 0.0}, 
			{'bounds/z/max': 1.0} 
		]
	)
	
	trajectory_node = Node(
		package='anafi_autonomy',
		namespace=LaunchConfiguration('namespace'),
		executable='trajectory',
		name='trajectory',
		output="screen",
		emulate_tty=True,
		arguments=['--ros-args', '--params-file', trajectory_config, '--log-level', 'INFO']
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
		safe_anafi_node,
		trajectory_node,
		rqt_image_view_node,
		rqt_reconfigure_node
	])
