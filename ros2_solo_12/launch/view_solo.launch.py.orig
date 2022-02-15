import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # ----------------------- constants for paths -----------------------
    package_name = 'ros2_solo_12'
    urdf_file_path = 'urdf/solo12.urdf.xacro'
    rviz_config_file_path = 'rviz/rrbot.rviz'

    # set the path to different files and folders
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)

    # ----------------------- Create launch variables -----------------------
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    world_path = LaunchConfiguration('world_path')

    # ----------------------- Declare launch arguments -----------------------
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot URDF file')

    declare_world_path_cmd = DeclareLaunchArgument(
        name='world_path',
        default_value='',
        description='The world path, by default is empty.world')

    # ----------------------- Nodes -----------------------

    # Publish the joint states of the robot
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'robot_description': Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", urdf_model])}],
    )

    # ----------------------- Launch Gazebo -----------------------

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # Spawn SOLO 12
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'solo',
                   '-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0'],
        output='screen'
    )

    # ----------------------- Launch RViz -----------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_world_path_cmd)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gzclient)
    ld.add_action(gzserver)
    # ld.add_action(rviz_node)
    ld.add_action(spawn_robot)

    return ld
