import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
				MoveItConfigsBuilder("leg_assembly")
				.robot_description(file_path="config/leg_urdf.urdf.xacro")
				.planning_scene_monitor(
					publish_robot_description=True, publish_robot_description_semantic=True
				)
				.planning_pipelines(
					pipelines=["ompl", "chomp"]
				)
				.to_moveit_configs()
    )
    rviz_config_file = (
				get_package_share_directory("leg_assembly_moveit_config") + "/config/config_used.rviz"
    )
    run_move_group_node = Node(
				package="moveit_ros_move_group",
				executable="move_group",
				output="screen",
				parameters=[moveit_config.to_dict()],
    )
		
    rviz_node = Node(
				package="rviz2",
				executable="rviz2",
				name="rviz2",
				output="log",
				arguments=["-d", rviz_config_file],
				parameters=[

						moveit_config.robot_description_semantic,
				],
    )
    # ------------------------
    # Launch arguments
    # ------------------------
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        description="Launch Gazebo GUI"
    )

    urdf_package_arg = DeclareLaunchArgument(
        name="urdf_package",
        default_value="leg_assembly_moveit_config",
        description="Package with robot description"
    )

    urdf_path_arg = DeclareLaunchArgument(
        name="urdf_package_path",
        default_value="config/leg_assembly_gazebo.urdf.xacro",
        description="URDF/Xacro path inside package"
    )

    # ------------------------
    # Gazebo (CLASSIC, STABLE)
    # ------------------------
    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-u",
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "worlds",
                "empty.world",
            ]),
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    gazebo_client = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
    )

    # ------------------------
    # Robot description (URDF + robot_state_publisher)
    # ------------------------
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("urdf_launch"),
                "launch",
                "description.launch.py",
            ])
        ),
        launch_arguments={
            "urdf_package": LaunchConfiguration("urdf_package"),
            "urdf_package_path": LaunchConfiguration("urdf_package_path"),
            "use_sim_time": "true",
        }.items(),
    )

    # ------------------------
    # Spawn robot into Gazebo
    # ------------------------
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "leg_assembly",
        ],
        output="screen",
    )

    # ------------------------
    # ros2_control
    # ------------------------

    polarix_arm_controller_spawner = Node(
				package="controller_manager",
				executable="spawner",
				arguments=["polarix_arm_controller", "-c", "/controller_manager"],
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "300",
        ],
        output="screen",
    )

    # ------------------------
    # Launch description
    # ------------------------
    return LaunchDescription([
        gui_arg,
        urdf_package_arg,
        urdf_path_arg,

        gazebo_server,
        gazebo_client,

        description_launch,
        spawn_entity,
        rviz_node,
        run_move_group_node,


        joint_state_broadcaster_spawner,
        polarix_arm_controller_spawner,
    ])

