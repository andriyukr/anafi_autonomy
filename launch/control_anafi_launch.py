import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
	anafi = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
			os.path.join(get_package_share_directory('olympe_bridge_nodes')), 
			'/anafi_launch.py'
		]),
		launch_arguments={
			'model': '',
			'ip': '192.168.53.1',
			'skycontroller': 'True',
			'drone_serial': '',
			'wifi_key': ''
		}.items()
	)

	return LaunchDescription([
		anafi,
		Node(
			package='anafi_autonomy',
			namespace='anafi',
			executable='safe_anafi',
			name='safe_anafi',
			output="screen",
			#emulate_tty=True,
			arguments=['--ros-args', '--log-level', 'INFO'],
			parameters=[
				{'safe_anafi/bounds/min_x': -5.0},
				{'safe_anafi/bounds/min_x': 5.0},
				{'safe_anafi/bounds/min_x': -5.0}, 
				{'safe_anafi/bounds/min_x': 5.0},
				{'safe_anafi/bounds/min_x': 0.2}, 
				{'safe_anafi/bounds/min_x': 2.0} 
			]
		),
		Node(
			package='anafi_autonomy',
			namespace='anafi',
			executable='trajectory',
			name='trajectory',
			output="screen",
			#emulate_tty=True,
			arguments=['--ros-args', '--log-level', 'INFO'],
		),
		Node(
			package='anafi_autonomy',
			namespace='anafi',
			executable='teleop_key',
			name='teleop_key',
			output="screen",
			emulate_tty=True,
			arguments=['--ros-args', '--log-level', 'INFO'],
		),
		Node(
			package='rqt_image_view',
			namespace='anafi',
			executable='rqt_image_view',
			name='rqt_image_view',
			arguments=['/anafi/camera/image'],
		),
	])
