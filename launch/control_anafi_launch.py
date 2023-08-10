# Usage: 
# 	- connection through Skycontroiller [recommended]:
# 		ros2 launch anafi_autonomy control_anafi_launch.py ip:='192.168.53.1'
#	- direct connection to Anafi:
# 		ros2 launch anafi_autonomy control_anafi_launch.py ip:='192.168.42.1' model:='ai'
#	- connection to the simulated drone in Sphinx:
# 		ros2 launch anafi_autonomy control_anafi_launch.py ip:='10.202.0.1' model:='ai'

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
	
	safe_anafi_include = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
			os.path.join(get_package_share_directory('anafi_autonomy')), 
			'/launch/safe_anafi_launch.py'
		]),
		launch_arguments={
			'drone/model': LaunchConfiguration('model'),
			'device/ip': LaunchConfiguration('ip')
		}.items()
	)
	
	teleop_key_node = Node(
		package='anafi_autonomy',
		namespace=LaunchConfiguration('namespace'),
		executable='teleop_key',
		name='teleop_key',
		output="screen",
		emulate_tty=True,
		prefix='xterm -e',
		arguments=['--ros-args', '--log-level', 'INFO']
	)
	
	return LaunchDescription([
		namespace_arg,
		ip_arg,
		model_arg,
		safe_anafi_include,
		teleop_key_node
	])
