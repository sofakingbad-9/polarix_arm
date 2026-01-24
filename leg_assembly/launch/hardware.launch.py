import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    moveit_config = (
				MoveItConfigsBuilder("leg_assembly")
				.robot_description(file_path="config/leg_assembmly_gazebo.urdf.xacro")
				.planning_scene_monitor(
					publish_robot_description=False, publish_robot_description_semantic=True
				)
				.planning_pipelines(
					pipelines=["ompl", "chomp"]
				)
				.to_moveit_configs()
    )
    rviz_config_file = (
				get_package_share_directory("leg_assembly_moveit_config") + "/config/config_used.rviz"
    )
    
    robot_description_content= ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"), ' ',
                PathJoinSubstitution(
                    [FindPackageShare('leg_assembly_moveit_config'), 'config', 'leg_assembly_hardware.urdf.xacro']
                ),
            ]
        ),
    )
    robot_description={'robot_description': robot_description_content}
    
    
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






    # ------------------------
    # Robot description (URDF + robot_state_publisher)
    # ------------------------

    robot_state_publisher = Node(
				package="robot_state_publisher",
				executable="robot_state_publisher",
				parameters=[robot_description],
    )



    # ------------------------
    # ros2_control
    # ------------------------
    ros2_control_node = Node(
				package="controller_manager",
				executable="ros2_control_node",
				parameters=[
				    robot_description,
				    PathJoinSubstitution([
				        FindPackageShare("leg_assembly_moveit_config"),
				        "config",
				        "gz_ros2_controllers.yaml",
				    ]),
				],
				output="screen",
    )


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
    
    joy_node=Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )
    teleop_control=Node(
				package='teleop_collision',          # your package name
				executable='teleop_collision',      # your executable
				name='teleop_node',
				output='screen',
    )
    gripper_controller_spawner=Node(
				package="controller_manager",
				executable="spawner",
				arguments=["hand_controller", "-c", "/controller_manager"],
		)
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[polarix_arm_controller_spawner],
        )
    )


    # ------------------------
    # Launch description
    # ------------------------
    return LaunchDescription([

        ros2_control_node,
        robot_state_publisher,

		
        rviz_node,
        run_move_group_node,
        joy_node,
        teleop_control,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,


        joint_state_broadcaster_spawner,
      #  polarix_arm_controller_spawner,
        gripper_controller_spawner,
    ])

