from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'empty_room.sdf'
        ])}.items()
    )
    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        # gz_sim,
        Node(
            package='traj',
            namespace='traj',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        Node(
            package='traj',
            namespace='traj',
            executable='arm_takeoff',
            name='arm_takeoff',
            # prefix='gnome-terminal --'
        ),
        Node(
            package='traj',
            namespace='traj',
            executable='visualizer',
            name='visualizer',
            prefix='gnome-terminal --tab --',
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            prefix='gnome-terminal --tab --',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'traj', 'offboard_control_launch.py'],
        #     name='offboard_control',
        #     prefix='gnome-terminal --tab --',
        #     on_exit=[
        #     Node(
        #         package='traj',
        #         namespace='traj',
        #         executable='offboard_control',
        #         name='offboard_control',
        #         prefix='gnome-terminal --tab --',
        #     )
        #     ]
        # ),
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='velocity_control',
        #     name='velocity'
        # ),
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        # )
    ])
