#include "arm_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstddef>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <string>

#include <cmath>

#include <sstream>
#include <algorithm>
#pragma pack(push,1)
struct ArmPacket{
		uint8_t start;
		float joints[7];
		uint8_t end;
};
#pragma pack(pop)
namespace arm_hardware
{
ArmHardwareInterface::ArmHardwareInterface()
{
		RCLCPP_FATAL(rclcpp::get_logger("ArmHardware"), "ARM HARDWARE CONSTRUCTOR CALLED");
}

ArmHardwareInterface::~ArmHardwareInterface()
{
		if(SerialPort!=-1){
				close(SerialPort);
		}
}


CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
		CallbackReturn result= hardware_interface::SystemInterface::on_init(hardware_info);
		if(result != CallbackReturn::SUCCESS)
		{
				return result;
		}
		


		
		position_commands_.resize(info_.joints.size(),0.0);
		position_states_.resize(info_.joints.size(),0.0);
		velocity_states_.resize(info_.joints.size(),0.0);
		hw_pos_.resize(info_.joints.size(),0.0);
		hw_vel_.resize(info_.joints.size(),0.0);
		
		
		
		return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces()
{
		std::vector<hardware_interface::StateInterface> state_interfaces;
		
		for(size_t i=0; i<info_.joints.size(); i++)
		{
				state_interfaces.emplace_back(hardware_interface::StateInterface(
						info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
		}
		for(size_t i=0; i<info_.joints.size(); i++)
		{
				state_interfaces.emplace_back(hardware_interface::StateInterface(
						info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
		}
		
		return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
{
		std::vector<hardware_interface::CommandInterface> command_interfaces;
		
		for(size_t i=0; i<info_.joints.size(); i++)
		{
				command_interfaces.emplace_back(hardware_interface::CommandInterface(
						info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
		}
		return command_interfaces;
}

CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
		RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "starting robot hardware...");
		try
		{
		
				std::string port = "/dev/ttyACM0";
				
				SerialPort = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

				if (SerialPort<0){
						RCLCPP_WARN(rclcpp::get_logger("ArmHardware"), 
						"Unable to open serial port ");
						SerialPort=-1;
				}
				
				
				if (tcgetattr(SerialPort, &tty) != 0)
				{
						RCLCPP_ERROR(rclcpp::get_logger("ArmHardware"), "Error %i from tcgetattr: %s", errno, strerror(errno));
						close(SerialPort);
						return CallbackReturn::ERROR;
				}
				
				tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
				tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
				tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
				tty.c_cflag |= CS8; // 8 bits per byte (most common)
				tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
				tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

				tty.c_lflag &= ~ICANON;
				tty.c_lflag &= ~ECHO; // Disable echo
				tty.c_lflag &= ~ECHOE; // Disable erasure
				tty.c_lflag &= ~ECHONL; // Disable new-line echo
				tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
				tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
				tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

				tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
				tty.c_oflag &= ~ONLCR;

				tty.c_cc[VTIME] = 1;
				tty.c_cc[VMIN] = 0;

				speed_t speed = B115200;
				cfsetospeed(&tty, speed);
				cfsetispeed(&tty, speed);

				tcflush(SerialPort, TCIFLUSH);
				if (tcsetattr(SerialPort, TCSANOW, &tty) != 0)
				{
						RCLCPP_ERROR(rclcpp::get_logger("ArmHardware"), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
						return hardware_interface::CallbackReturn::ERROR;
				}
						
		}	
		catch(const std::exception &e)
		{
				RCLCPP_WARN(
					rclcpp::get_logger("ArmHardware"),
					"error: %s", e.what()
				);
				return CallbackReturn::ERROR;
		}
		std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
		
		RCLCPP_INFO(rclcpp::get_logger("ArmHardware"), "started");
		return CallbackReturn::SUCCESS;
}

int ArmHardwareInterface::WriteToSerial(const void* buf, size_t nBytes)
{
		return ::write(SerialPort, buf, nBytes);
}


/*void ArmHardwareInterface::send_joint_setpoint(size_t joint, double target_rad){
		std::ostringstream ss;
		ss<<"J"<<joint<<" "<<target_rad<<"\n";
		std::string cmd = ss.str();
		ssize_t n=WriteToSerial(cmd.data(),cmd.size());
		if(n<0){
				RCLCPP_WARN(rclcpp::get_logger("ArmHardware"),"serial write failed ");
		}
		else{
			//	RCLCPP_WARN(rclcpp::get_logger("ArmHardware"),"sent: %s", cmd.c_str());
		}
		
}*/


/*hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &){
		std::ostringstream ss;
		ss<<"J";
		for(size_t i=0; i<info_.joints.size();i++){
				double target_rad=position_commands_[i];
				ss<<" "<<target_rad;

		}
		ss<<"\n";
		std::string cmd = ss.str();
		ssize_t n=WriteToSerial(cmd.data(),cmd.size());
		if(n<0){
				RCLCPP_WARN(rclcpp::get_logger("ArmHardware"),"serial write failed ");
		}
		else{
			//	RCLCPP_WARN(rclcpp::get_logger("ArmHardware"),"sent: %s", cmd.c_str());
		}
		return hardware_interface::return_type::OK;
}*/
hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &){
		ArmPacket pac{};
		pac.start= 0xAA;
		for(size_t i=0;i<info_.joints.size();i++){
				pac.joints[i]=position_commands_[i];
		}
		pac.end= 0x0A;
		ssize_t n= WriteToSerial(&pac, sizeof(pac));
		if(n!=sizeof(pac)){
				RCLCPP_WARN(rclcpp::get_logger("ArmHardware"),"serial write failed ");
		}
		
		return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &){
		read_buffer();
		std::string line;
		while(line_read(line)){
				line_parse(line);
		}
		
		for(size_t i=0; i<info_.joints.size();i++){
				position_states_[i]=hw_pos_[i];
				velocity_states_[i]=hw_vel_[i];
		}
		return hardware_interface::return_type::OK;
		
}

void ArmHardwareInterface::read_buffer(){
		char buff[256];
		int n=::read(SerialPort, buff, sizeof(buff));
		if(n>0){
				serial_buffer_.append(buff,n);
		}
}

bool ArmHardwareInterface::line_read(std::string &line){
		size_t pos= serial_buffer_.find("\n");
		if(pos ==std::string::npos){
				return false;
		}
		
		line = serial_buffer_.substr(0,pos);
		serial_buffer_.erase(0,pos+1);
		return true;
}

void ArmHardwareInterface::line_parse(const std::string &line){

		//RCLCPP_WARN(rclcpp::get_logger("ArmHardware"),"serial: %s", line.c_str());
		std::istringstream ss(line);
		char tag;
		size_t joint;
		double pos, vel;
		std::string cmd_r=ss.str();
		ss>>tag>>joint>>pos>>vel;
		if(!ss.fail() && tag =='S' && joint<hw_pos_.size()){
				hw_pos_[joint]=pos;
				hw_vel_[joint]=vel;
		}

}
CallbackReturn ArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ArmHardwareInterface"),
    "Stopping robot hardware..."
  );

  if (SerialPort != -1) {
    close(SerialPort);
    SerialPort = -1;
  }

  return CallbackReturn::SUCCESS;
}

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface);
}







				
				
		
