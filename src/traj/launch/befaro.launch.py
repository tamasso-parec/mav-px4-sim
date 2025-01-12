#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """Launch Gazebo with a drone running PX4 communicating over ROS 2."""
    
    PX4_RUN_DIR = '/home/tom/Documents/PhD/px4_test/'
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    px4_autopilot_dir = os.path.expanduser('~/PX4-Autopilot')

    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'x500_base', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    ros_gz_example_gazebo_dir = get_package_share_directory('ros_gz_example_gazebo')
    world = os.path.join(ros_gz_example_gazebo_dir, 'worlds', 'labor_worlds', 'rondell.world')
    model = os.path.join(ros_gz_example_gazebo_dir, 'models', 'x500_base', 'model.sdf')
    #custom_gazebo_models = os.path.join(blackdrones_description_dir, 'models')
    #px4_init = os.path.join(blackdrones_description_dir, 'PX4-init')

    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            ros_gz_example_gazebo_dir,
            'worlds',
            'wall.sdf'
        ])}.items(),
    )
    # spawn_robot = 

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
                               '~/PX4-Autopilot/build/px4_sitl_default/build_gazebo'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', '~/PX4-Autopilot/Tools/simulation/gz/models'),

        SetEnvironmentVariable('PX4_SIM_MODEL', 'gz_x500'),

        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('model', default_value=model),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),

        gz_sim,
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzserver.launch.py']),
        #     launch_arguments={'world': LaunchConfiguration('world'),
        #                       'verbose': 'true'}.items(),
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzclient.launch.py'])
        # ),

        # Node(
        # package='ros_gz_sim',
        # executable='create',
        # arguments=[
        #     '-file', LaunchConfiguration('model'),
        #     '-name', 'drone',
        #     '-x', LaunchConfiguration('x'),
        #     '-y', LaunchConfiguration('y'),
        #     '-z', LaunchConfiguration('z'),
        #     '-R', LaunchConfiguration('R'),
        #     '-P', LaunchConfiguration('P'),
        #     '-Y', LaunchConfiguration('Y')
        # ],
        # output='screen'
        # ),

        # ExecuteProcess(
        #     cmd=[
        #         'gz', 'model',
        #         '--spawn-file', LaunchConfiguration('model'),
        #         '--model-name', 'drone',
        #         '-x', LaunchConfiguration('x'),
        #         '-y', LaunchConfiguration('y'),
        #         '-z', LaunchConfiguration('z'),
        #         '-R', LaunchConfiguration('R'),
        #         '-P', LaunchConfiguration('P'),
        #         '-Y', LaunchConfiguration('Y')
        #     ],
        #     prefix="bash -c 'sleep 5s; $0 $@'",
        #     output='screen'),
            # Setup to launch the simulator and Gazebo world

        
        ExecuteProcess(
            cmd=[
                'build/px4_sitl_default/bin/px4',
                # 'ROMFS/px4fmu_common/',
                # '-s',
                # 'ROMFS/px4fmu_common/init.d-posix/rcS'
            ],
            cwd= px4_autopilot_dir,
            output='screen'),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'),

])