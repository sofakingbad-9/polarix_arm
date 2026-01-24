cmake_minimum_required(VERSION 3.14.4)

project(leg_assembly)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
	src/arm_hardware_interface.cpp
)


target_include_directories(
	${PROJECT_NAME}
	PUBLIC
	include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle

)
pluginlib_export_plugin_description_file(hardware_interface arm_hardware_interface.xml)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp hardware_interface pluginlib rclcpp_lifecycle)

install(
	TARGETS
		${PROJECT_NAME}
	DESTINATION lib/${PROJECT_NAME}
)


install(
	DIRECTORY config include launch meshes urdf 
	

	DESTINATION share/${PROJECT_NAME}
)


ament_package()
