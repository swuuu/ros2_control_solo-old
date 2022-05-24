from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_use_sim_time = (DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                                  description='Flag to enable use_sim'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "true", "pause": "true"}.items(),
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_description_solo"),
                    "urdf",
                    "solo12.urdf.xacro",
                ]
            ),
            " use_sim:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "solo", "-x 0", "-y 0", "-z 0.5"],
        output="screen",
    )
    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_controller_effort = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["effort_controllers"],
        # arguments=["forward_position_controller"],
        output="screen",
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare("ros2_control_solo_bringup"), "config", "ekf.yaml"])
            , {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            gazebo,
            node_robot_state_publisher,
            spawn_entity,
            spawn_controller,
            spawn_controller_effort,
            robot_localization_node
        ]
    )
