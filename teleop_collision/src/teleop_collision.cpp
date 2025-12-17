#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>



class TeleopWithCollision : public rclcpp::Node
{
public:
		TeleopWithCollision() : Node("collision_checker")
		{
			RCLCPP_INFO(get_logger(), "Teleop node with collision checking started");
			
			joint_names_ = {
				"base_joint",
				"shoulder_joint",
				"elbow_joint",
				"wrist_roll",
				"wrist_pitch"
			};
			

			
			current_positions_.resize(joint_names_.size(), 0.0);
			max_vel_ = 0.3;         // rad/s
			dt_ = 0.05;             // 20 Hz update
			
			
			joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"/joy", 10, std::bind(&TeleopWithCollision::JoyCallback, this, std::placeholders::_1));
			
			
			
			js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 50,
            std::bind(&TeleopWithCollision::JointStateCallback, this, std::placeholders::_1));

        // ---- Publisher ----
      traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/polarix_arm_controller/joint_trajectory", 10);
      
    }
		void initMoveIt()
		{
			auto node=shared_from_this();									
			robot_model_loader_ =
				std::make_shared<robot_model_loader::RobotModelLoader>(
					node, "robot_description");

			psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
				node, robot_model_loader_);

			if (!psm_->getPlanningScene())
			{
				RCLCPP_ERROR(get_logger(), "Failed to initialize PlanningSceneMonitor");
				return;
			}

			psm_->startSceneMonitor();
			psm_->startWorldGeometryMonitor();
			psm_->startStateMonitor("/joint_states");

			RCLCPP_INFO(get_logger(), "PlanningSceneMonitor active.");
		}
		



private:
		void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
		{
			latest_joint_state_=*msg;
		}
		
		
		void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
		{
			
      double dpad_x = msg->axes[6];
      double dpad_y = msg->axes[7];
      double right_x = msg->axes[4];
      double right_y = -msg->axes[3];
      
  
      std::vector<double> increments = {
        dpad_x * max_vel_ * dt_,
        dpad_y * max_vel_ * dt_,
        right_x * max_vel_ * dt_,
        right_y * max_vel_ * dt_,
        0.0 // if fifth joint unused on joystick
   		};
   		
   		
   		std::vector<double> updated_angles = current_positions_;
   		
   		
      for(size_t i=0;i<size(updated_angles);i++){
      	updated_angles[i]+=increments[i];
      }
      
      if(!isCollisionFree(updated_angles)){
      	RCLCPP_WARN(get_logger(), "COLLISION NIGGA MOVEEEEEEEEEE");
      	return;
      }
      
      current_positions_=updated_angles;
      
      trajectory_msgs::msg::JointTrajectory traj;
      
      traj.joint_names = joint_names_;

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = current_positions_;
      pt.time_from_start = rclcpp::Duration::from_nanoseconds(50'000'000); // 50ms

      traj.points.push_back(pt);

      traj_pub_->publish(traj);
			
			
		}
		
		
		bool isCollisionFree(const std::vector<double> &positions)
		{
			planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
			
			moveit::core::RobotState& state= scene->getCurrentStateNonConst();
			
			state.setJointGroupPositions("arm", positions);
			
			state.update();
			
			return scene->isStateValid(state, "arm");
			
		}
		
		
		std::vector<std::string> joint_names_;
    std::vector<double> current_positions_;
    sensor_msgs::msg::JointState latest_joint_state_;

    double max_vel_, dt_;
    
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};










int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
	auto node = std::make_shared<TeleopWithCollision>();
	node->initMoveIt();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
  
  
  
  
