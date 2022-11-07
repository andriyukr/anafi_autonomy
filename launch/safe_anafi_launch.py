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
	
	config_safe_anafi = os.path.join(
		get_package_share_directory('anafi_autonomy'),
		'config/params_safe_anafi.yaml'
	)
	config_trajectory = os.path.join(
		get_package_share_directory('anafi_autonomy'),
		'config/params_trajectory.yaml'
	)

	return LaunchDescription([
		anafi,
		Node(
			package='anafi_autonomy',
			namespace='anafi',
			executable='safe_anafi',
			name='safe_anafi',
			output="screen",
			emulate_tty=False,
			arguments=['--ros-args', '--params-file', config_safe_anafi, '--log-level', 'INFO'],
			parameters=[
				{'bounds/x/min': -1.0},
				{'bounds/x/max': 1.0},
				{'bounds/y/min': -1.0}, 
				{'bounds/y/max': 1.0},
				{'bounds/z/min': 0.0}, 
				{'bounds/z/max': 1.0} 
			]
		),
		Node(
			package='anafi_autonomy',
			namespace='anafi',
			executable='trajectory',
			name='trajectory',
			output="screen",
			emulate_tty=False,
			arguments=['--ros-args', '--params-file', config_trajectory, '--log-level', 'INFO'],
		),
		Node(
			package='rqt_image_view',
			namespace='anafi',
			executable='rqt_image_view',
			name='rqt_image_view',
			arguments=['/anafi/camera/image'],
		),
		Node(
			package='rqt_reconfigure',
			namespace='anafi',
			executable='rqt_reconfigure',
			name='rqt_reconfigure',
		),
	])
