import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

	airframe_launch_arg = DeclareLaunchArgument(
		'airframe', default_value='gz_x500_depth'
		# 'airframe', default_value='gz_x500_base'
		# 'airframe', default_value='gz_x500_mono_cam'
	)
  
	ddsport_launch_arg = DeclareLaunchArgument(
	'port', default_value='8888'
	)

	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

	pkg_project_description = get_package_share_directory('ros_gz_example_description')
	pkg_traj = get_package_share_directory('traj')
	# sdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_mono_cam', 'model.sdf')
	sdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_depth', 'model.sdf')
	# sdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_base', 'model.sdf')
	with open(sdf_file, 'r') as infp:
		robot_desc = infp.read()

	ros_gz_example_gazebo_dir = get_package_share_directory('ros_gz_example_gazebo')

	airframe_value = LaunchConfiguration("airframe")
	ddsport_value = LaunchConfiguration("port")
	
	px4_src_dir =  os.path.expanduser('~/PX4-Autopilot')

	uxrce_dds_synct_env = SetEnvironmentVariable(
		'UXRCE_DDS_SYNCT', '0'
	)


	gz_sim = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
		launch_arguments={
			'gz_args': PathJoinSubstitution([
				ros_gz_example_gazebo_dir,
				'worlds',
				'default_custom.sdf', 
			]),
			'on_exit_shutdown': 'True',
			'paused': 'False',
			'use_sim_time': 'true'
		}.items(),
	)

	# Bridge ROS topics and Gazebo messages for establishing communication
	ros_gz_bridge = Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		parameters=[{
			'config_file': os.path.join(pkg_traj, 'resource', 'traj_bridge.yaml'),
			# 'qos_overrides./tf_static.publisher.durability': 'transient_local',
		}],
		prefix='gnome-terminal --tab --',
		output='screen'
	)
	
	use_sim_time_setter = SetParameter(name='use_sim_time', value=True)

	px4_sim_model_env = SetEnvironmentVariable(
		'PX4_SIM_MODEL', airframe_value
	)
	gz_standalone_env = SetEnvironmentVariable(
		'PX4_GZ_STANDALONE', "1"
	)


	px4_sim_cmd = ExecuteProcess(
		# cmd=[
		# 	'gnome-terminal',
		# 	'--',
		# 	# "PX4_GZ_STANDALONE=1",
		# 	"make", 
		# 	"px4_sitl",
		# 	airframe_value
		# ],
		cmd=[
			'gnome-terminal',
			'--',
			"build/px4_sitl_default/bin/px4",
			"-d",
			"-s",
			"etc/init.d-posix/rcS",
			"build/px4_sitl_default/etc"
		],
		cwd = px4_src_dir,
		output="screen"
	)

	QGC_cmd = ExecuteProcess(
		cmd = ['gnome-terminal', '--', './QGroundControl.AppImage'	],
		cwd =  os.path.expanduser('~'),
		output='screen'
	)

	dds_cmd = ExecuteProcess(
		cmd=['gnome-terminal', '--', "MicroXRCEAgent", "udp4", "-p", ddsport_value],
		cwd=os.getcwd(),
		output='screen'
	)

	map_frame_node = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='map_frame_publisher',
		arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
		output='screen'
	)


	px4_tf_node = Node(
		package='traj',
		executable='px4_tf',
		name='px4_tf',
		prefix='gnome-terminal --tab --',
		output='screen'
	)

	pointcloud_trafo_node = Node(
		package='tf_pcl_pub', 
		executable='tf_pcl_pub_node',
		name= 'pointcloud_trafo_node',
		output='screen'
	)
	
	
	depth_camera_pointcloud_node = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='depth_camera_pointcloud_publisher',
		arguments=['0', '0', '0', '0', '0', '0', 'drone', 'x500_depth_0/OakD-Lite/base_link/StereoOV7251'],
		output='screen'
	)


	visualizer_node = Node(
            package='traj',
            namespace='traj',
            executable='visualizer',
            name='visualizer',
            prefix='gnome-terminal --tab --',
        )

	rviz2_node = Node(
		package='rviz2',
		namespace='',
		executable='rviz2',
		name='rviz2',
		prefix='gnome-terminal --tab --',
		arguments=['-d', [os.path.join(pkg_traj, 'resource/visualize.rviz')]]
	)


	return LaunchDescription(
	[
	# use_sim_time_setter,
	# uxrce_dds_synct_env,
	airframe_launch_arg,
	ddsport_launch_arg,
	px4_sim_model_env,
	# gz_standalone_env,
	uxrce_dds_synct_env,
	dds_cmd,
	gz_sim,
	px4_sim_cmd,
	QGC_cmd, 
	ros_gz_bridge, 
	map_frame_node,
	px4_tf_node,
	pointcloud_trafo_node,
	visualizer_node,
	rviz2_node
	]
	)    