#ifndef ARM_HARDWARE_INTERFACE_H
#define ARM_HARDWARE_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/macros.hpp"
#include <hardware_interface/system_interface.hpp>

#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <cstddef>
#include <vector>
#include <string>
#include <memory>
#include <termios.h>


namespace arm_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  ArmHardwareInterface();
  virtual ~ArmHardwareInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // Implementing hardware_interface::SystemInterface
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  std::vector<double> position_commands_;

  std::vector<double> velocity_states_; //optional
  std::vector<double> position_states_;
	
  int SerialPort = -1;
  std::string serial_buffer_;	
  struct termios tty;
	std::vector<double> hw_pos_;
	std::vector<double> hw_vel_;
	
  int WriteToSerial(const void* buf, size_t nBytes);
  void send_joint_setpoint(size_t joint, double target_rad);

  void read_buffer();
  bool line_read(std::string &line);
  void line_parse(const std::string &line);


};
} 

#endif  
