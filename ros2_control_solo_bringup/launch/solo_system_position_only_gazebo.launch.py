from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # sdf file
    sdf_model_path = PathJoinSubstitution(
        [
            FindPackageShare("ros2_description_solo"),
            "urdf",
            "solo12_simulation.sdf",
        ]
    )
    sdf_model = LaunchConfiguration('sdf_model')
    declare_sdf_model_path_cmd = DeclareLaunchArgument(
        name='sdf_model',
        default_value=sdf_model_path,
        description='Absolute path to robot sdf file')

    # world file
    world_model_path = PathJoinSubstitution(
        [
            FindPackageShare("models"),
            "worlds",
            "empty.world"
        ]
    )
    world_model = LaunchConfiguration('world_model')
    declare_world_model_path_cmd = DeclareLaunchArgument(
        name='world_model',
        default_value=world_model_path,
        description='Absolute path to world file'
    )

    declare_use_sim_time = (DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                                  description='Flag to enable use_sim'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "false", "pause": "true", "world": world_model}.items(),
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

    # Using a URDF file
    # spawn_entity = Node(
    #     package="gazebo_ros",
    #     executable="spawn_entity.py",
    #     arguments=["-topic", "robot_description", "-entity", "solo", "-x 0", "-y 0", "-z 0.5"],
    #     output="screen",
    # )

    # # Using a SDF file
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", sdf_model, "-entity", "solo", "-x 0", "-y 0", "-z 0.5"],
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
        output="screen",
    )

    return LaunchDescription(
        [
            declare_sdf_model_path_cmd,
            declare_world_model_path_cmd,
            declare_use_sim_time,
            gazebo,
            node_robot_state_publisher,
            spawn_entity,
            spawn_controller,
            spawn_controller_effort,
        ]
    )
