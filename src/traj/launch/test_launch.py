from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='visualizer',
        #     name='visualizer'
        # ),
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
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='control',
        #     name='control',
        #     prefix='gnome-terminal --',
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
