#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

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
			
			timer_ = create_wall_timer(
				std::chrono::milliseconds(20),
				std::bind(&TeleopWithCollision::update, this));

			
			current_positions_.resize(joint_names_.size(), 0.0);
			max_vel_ = 0.5;         // rad/s
		
			max_acc_=90.0;
			joint_vel_.resize(joint_names_.size(),0.0);
			
			joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"/joy", 10, std::bind(&TeleopWithCollision::JoyCallback, this, std::placeholders::_1));
			

			
			js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 50,
            std::bind(&TeleopWithCollision::JointStateCallback, this, std::placeholders::_1));

        // ---- Publisher ----
      traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/polarix_arm_controller/joint_trajectory", 10);
      gripper_client_ =
		    rclcpp_action::create_client<control_msgs::action::GripperCommand>(
		      this,
		      "/hand_controller/gripper_cmd"
      );
      
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
			// Build index map ONCE
			if (!js_initialized_)
			{
				for (size_t i = 0; i < msg->name.size(); ++i)
				  js_index_[msg->name[i]] = i;

				js_initialized_ = true;
			}

			// Fast indexed access from here on
			for (size_t i = 0; i < joint_names_.size(); ++i)
			{
				auto it = js_index_.find(joint_names_[i]);
				if (it == js_index_.end()) continue;

				size_t idx = it->second;

				if (idx < msg->position.size())
					current_positions_[i] = msg->position[idx];
				if (idx < msg->position.size())
					joint_vel_[i] = msg->velocity[idx];


			}
		}

		

		
		void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
		{
			
				joy_cmd[0]= msg->axes[6];
				joy_cmd[1]= msg->axes[7];
				if(msg->buttons[6])joy_cmd[2]=-msg->buttons[6];
				else if(msg->buttons[7])joy_cmd[2]=msg->buttons[7];
				else joy_cmd[2]=0.0;
				joy_cmd[3] = msg->axes[2];
				joy_cmd[4] = msg->axes[3];
				if(msg->buttons[1] && !prev_grip_){
					closeGripper();
					prev_grip_=true;
				}
				if(msg->buttons[0] && true ){
					openGripper();
					prev_grip_=false;
				}
  
      
  	}
  	
  	void update(){
  		if(!js_initialized_)return;
  	
      std::vector<double> v_des = {
        joy_cmd[0] * max_vel_ ,
        joy_cmd[1] * max_vel_ ,
        joy_cmd[2] * max_vel_,
        joy_cmd[3] * max_vel_,
        joy_cmd[4] * max_vel_ ,
        
   		};
   		
   		dt_=0.02;
   		
   		std::vector<double> updated_angles = current_positions_;

   		
   		

      double velocity_scale=1.0;
      
      if(!isCollisionFree(updated_angles)){
      	RCLCPP_WARN(get_logger(), "COLLISION NIGGA MOVEEEEEEEEEE");
      	velocity_scale=0.0;
      }
      for(size_t i=0;i<size(updated_angles);i++){
      
      	double dv= v_des[i]-joint_vel_[i];
      	double max_dv= max_acc_*dt_;
      	
      	dv=std::clamp(dv,-max_dv,max_dv);
      	updated_angles[i]+=velocity_scale*(dv+joint_vel_[i])*dt_;
      }

      
     // current_positions_=updated_angles;
      
      trajectory_msgs::msg::JointTrajectory traj;
      
      traj.joint_names = joint_names_;

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = updated_angles;
      pt.time_from_start = rclcpp::Duration::from_nanoseconds(50'000'000); // 50ms

      traj.points.push_back(pt);

      traj_pub_->publish(traj);
			
			
		}
    void closeGripper()
		{
			if (!gripper_client_->wait_for_action_server(std::chrono::seconds(1))) {
				RCLCPP_ERROR(get_logger(), "Gripper action server not available");
				return;
			}

			control_msgs::action::GripperCommand::Goal goal;
			double deg=-7.0;
			double rad=deg* M_PI/180.0;
			goal.command.position=rad;
			goal.command.max_effort=0.0;
			

			gripper_client_->async_send_goal(goal);

			RCLCPP_INFO(get_logger(), "Sent gripper CLOSE command");
    }
		
		
		bool isCollisionFree(const std::vector<double> &positions)
		{
			planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
			
			moveit::core::RobotState& state= scene->getCurrentStateNonConst();
			
			state.setJointGroupPositions("arm", positions);
			
			state.update();
			
			return scene->isStateValid(state, "arm");
			
		}
		void openGripper(){
			if(!gripper_client_->wait_for_action_server(std::chrono::seconds(1))){
				RCLCPP_ERROR(get_logger(), "Gripper action server not available");
				return;
			}
			control_msgs::action::GripperCommand::Goal goal;
			double deg=4.0;
			double rad=deg* M_PI/180.0;
			goal.command.position=rad;
			goal.command.max_effort=0.0;
			

			gripper_client_->async_send_goal(goal);

			RCLCPP_INFO(get_logger(), "Sent gripper open command");
    }
		
		rclcpp::TimerBase::SharedPtr timer_;

		std::vector<std::string> joint_names_;
    std::vector<double> current_positions_{0.0, 0.0, 0.0, 0.0, 0.0};
    sensor_msgs::msg::JointState latest_joint_state_;
		std::vector<double> joint_vel_;
    double max_vel_, dt_,max_acc_;
    
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;
    bool prev_grip_{false}, js_initialized_{false};
		std::unordered_map<std::string, size_t> js_index_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
   	std::vector<double> joy_cmd{0.0, 0.0, 0.0, 0.0, 0.0};

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
