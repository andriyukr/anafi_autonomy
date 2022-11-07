import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
	safe_anafi = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
			os.path.join(get_package_share_directory('anafi_autonomy')), 
			'/launch/safe_anafi_launch.py'
		])
	)

	return LaunchDescription([
		safe_anafi,
		Node(
			package='anafi_autonomy',
			namespace='anafi',
			executable='teleop_key',
			name='teleop_key',
			output="screen",
			emulate_tty=True,
			prefix='xterm -e',
			arguments=['--ros-args', '--log-level', 'INFO'],
		),
	])
