# launch/leg_launch.py
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

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
						moveit_config.robot_description,
						moveit_config.robot_description_semantic,
				],
		)

		ros2_controllers_path = os.path.join(
				get_package_share_directory("leg_assembly_moveit_config"),
				"config",
				"ros2_controllers.yaml",
		)

		ros2_control_node = Node(
				package="controller_manager",
				executable="ros2_control_node",
				parameters=[moveit_config.robot_description, ros2_controllers_path],
				output="screen",
		)

		joint_state_broadcaster_spawner = Node(
				package="controller_manager",
				executable="spawner",
				arguments=[
				    "joint_state_broadcaster",
				    "--controller-manager-timeout",
				    "300",
				    "--controller-manager",
				    "/controller_manager",
				],
		)
		
	

		polarix_arm_controller_spawner = Node(
				package="controller_manager",
				executable="spawner",
				arguments=["polarix_arm_controller", "-c", "/controller_manager"],
		)
		
    # Launch as much as possible in components
		container = ComposableNodeContainer(
				name="polarix_leg_container",
				namespace="/",
				package="rclcpp_components",
				executable="component_container_mt",
				composable_node_descriptions=[
					# Example of launching Servo as a node component
					# Assuming ROS2 intraprocess communications works well, this is a more efficient way.
					# ComposableNode(
					#     package="moveit_servo",
					#     plugin="moveit_servo::ServoServer",
					#     name="servo_server",
					#     parameters=[
					#         servo_params,
					#         moveit_config.robot_description,
					#         moveit_config.robot_description_semantic,
					#     ],
					# ),
					ComposableNode(
						package="robot_state_publisher",
						plugin="robot_state_publisher::RobotStatePublisher",
						name="robot_state_publisher",
						parameters=[moveit_config.robot_description],
					),
					ComposableNode(
						package="tf2_ros",
						plugin="tf2_ros::StaticTransformBroadcasterNode",
						name="static_tf2_broadcaster",
						parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
					),
					ComposableNode(
						package="joy",
						plugin="joy::Joy",
						name="joy_node",
					),
				],
				output="screen",
		)

		return LaunchDescription(
				[
						rviz_node,
						ros2_control_node,
						run_move_group_node,
						joint_state_broadcaster_spawner,
						polarix_arm_controller_spawner,
						container,
				]
		)
    				
    		
