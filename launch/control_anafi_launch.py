# usage: ros2 launch anafi_autonomy control_anafi_launch.py ip:='192.168.53.1'
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
		"namespace", 
		default_value="anafi", 
		description="Namespace for this Anafi")
	ip_arg = DeclareLaunchArgument(
		"ip", 
		default_value="192.168.53.1",
		description="IP address of Anafi to connect")
	
	safe_anafi_include = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
			os.path.join(get_package_share_directory('anafi_autonomy')), 
			'/launch/safe_anafi_launch.py'
		]),
		launch_arguments={
			'model': '',
			'ip': LaunchConfiguration('ip'),
			'skycontroller': 'True',
			'drone_serial': '',
			'wifi_key': ''
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
	
	rqt_reconfigure_node = Node(
		package='rqt_reconfigure',
		namespace=LaunchConfiguration('namespace'),
		executable='rqt_reconfigure',
		name='rqt_reconfigure'
	)

	return LaunchDescription([
		namespace_arg,
		ip_arg,
		safe_anafi_include,
		teleop_key_node,
		rqt_reconfigure_node
	])
