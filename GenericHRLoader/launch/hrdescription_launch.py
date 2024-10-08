import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    default_rviz_config_path = os.path.join(get_package_share_directory('GenericHRLoader'), 'rviz/RobotViewer.rviz')

    urdf_file_name = 'urdf/h1.urdf'
    urdf = os.path.join(
        get_package_share_directory('GenericHRLoader'),
        urdf_file_name)


    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    pkg_mvconfig = get_package_share_directory('h1_moveitconfig')
    mv_launch_1 = PathJoinSubstitution([pkg_mvconfig, 'launch', 'move_group.launch.py'])
    mv_launch_2 = PathJoinSubstitution([pkg_mvconfig, 'launch', 'moveit_rviz.launch.py'])
    mv_launch_3 = PathJoinSubstitution([pkg_mvconfig, 'launch', 'rsp.launch.py'])
    mv_launch_5 = PathJoinSubstitution([pkg_mvconfig, 'launch', 'spawn_controllers.launch.py'])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(mv_launch_1)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(mv_launch_2)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(mv_launch_3)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(mv_launch_5)),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='GenericHRLoader',
            executable='HR_Joint_Pub',
            name='HR_Joint_Pub',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name = 'rviz2',
            arguments = ['-d' + os.path.join(get_package_share_directory('GenericHRLoader'), 'rviz', 'RobotViewer.rviz')]
        )
    ])