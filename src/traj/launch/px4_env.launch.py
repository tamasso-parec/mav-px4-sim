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

	# Declare launch arguments

	declare_gz_model_arg = DeclareLaunchArgument(
		'gz_model',
		default_value='x500_realsense',
		description='Gazebo model of the aircraft'
	)
	declare_gz_world_arg = DeclareLaunchArgument(
		'gz_world',
		default_value='default_custom.sdf',
		description='Gazebo world with aircraft'
	)

	gz_model_value = LaunchConfiguration('gz_model')
	gz_world_value = LaunchConfiguration('gz_world')
	
	
	
	set_px4_sim_model = SetEnvironmentVariable(
		name='PX4_SIM_MODEL',
		value=''
	)
	set_px4_model_name = SetEnvironmentVariable(
		name='PX4_GZ_MODEL_NAME',
		value=gz_model_value
	)
	
	set_px4_gz_world = SetEnvironmentVariable(
		name='PX4_GZ_WORLD',
		value=gz_world_value
	)

  
	ddsport_launch_arg = DeclareLaunchArgument(
	'port', default_value='8888'
	)

	ddsport_value = LaunchConfiguration("port")
	
	px4_src_dir =  os.path.expanduser('~/PX4-Autopilot')

	

	px4_sim_cmd = ExecuteProcess(
		cmd=[
			'gnome-terminal',
			'--',
			"build/px4_sitl_default/bin/px4"
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
	
	return LaunchDescription(
	[
	declare_gz_model_arg,
	declare_gz_world_arg,
	set_px4_sim_model, 
	set_px4_model_name,
	set_px4_gz_world,
	ddsport_launch_arg,
	dds_cmd,
	px4_sim_cmd,
	QGC_cmd, 
	px4_tf_node,
	pointcloud_trafo_node,
	]
	)    