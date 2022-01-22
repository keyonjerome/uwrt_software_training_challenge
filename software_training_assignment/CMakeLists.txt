cmake_minimum_required(VERSION 3.5)
project(software_training_assignment)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


#include the 'include' directory
include_directories(include)

# custom services and messages and actions
rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/ResetMovingTurtle.srv"
	"msg/Software.msg"
	"action/Software.action"
	DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
	)
ament_export_dependencies(rosidl_default_runtime)

#create resource which references the libraries in the binary bin
set(node_plugins "")


#add clear_turtles as plugin
add_library(clear_turtles SHARED
	src/clear_turtles.cpp)
target_compile_definitions(clear_turtles
	PRIVATE "SOFTWARE_TRAINING_DLL")
ament_target_dependencies(clear_turtles
	"rclcpp"
	"rclcpp_components"
	"turtlesim"
	"std_msgs"
	"geometry_msgs")
# rosidl_target_interfaces(clear_turtles ${PROJECT_NAME} "rosidl_typesupport_cpp") # need this for custom messages - idk why tho
rclcpp_components_register_node(clear_turtles PLUGIN "composition::clear_turtles"  EXECUTABLE turtle_clear)
# set(node_plugins PLUGIN "${node_plugins}composition::clear_turtles;$<TARGET_FILE:clear_turtles\n>" EXECUTABLE turtle_clear)



#add spawn_turtle_nodelets as plugin
add_library(turtle_spawn SHARED
	src/spawn_turtle_nodelet.cpp)
target_compile_definitions(turtle_spawn
	PRIVATE "SOFTWARE_TRAINING_DLL")
ament_target_dependencies(turtle_spawn
	"rclcpp"
	"rclcpp_components"
	"turtlesim"
	"std_msgs"
	"geometry_msgs")
rclcpp_components_register_node(turtle_spawn PLUGIN "composition::spawn_turtle_nodelet" EXECUTABLE turtle_spawner)
# set(node_plugins PLUGIN "${node_plugins}composition::spawn_turtle_nodelet;$<TARGET_FILE:turtle_spawn\n>" EXECUTABLE turtle_spawner)


#add turtle publisher nodelet
add_library(turtle_pub SHARED
	src/turtle_publisher.cpp)
target_compile_definitions(turtle_pub
	PRIVATE "SOFTWARE_TRAINING_DLL"
	)
ament_target_dependencies(turtle_pub
	"rclcpp"
	"rclcpp_components"
	"turtlesim"
	"std_msgs")
rosidl_target_interfaces(turtle_pub ${PROJECT_NAME} "rosidl_typesupport_cpp") # need this for custom messages - idk why tho
rclcpp_components_register_node(turtle_pub PLUGIN "composition::turtle_publisher" EXECUTABLE turtle_main_pub)
# set(node_plugins PLUGIN "${node_plugins}composition::turtle_publisher;$<TARGET_FILE:turtle_pub\n>" EXECUTABLE turtle_main_pub)

#add moving turtle service 
add_library(turtle_service SHARED
	src/reset_moving_turtle.cpp)
target_compile_definitions(turtle_service
	PRIVATE "SOFTWARE_TRAINING_DLL"
	)
ament_target_dependencies(turtle_service
	"rclcpp"
	"rclcpp_components"
	"turtlesim"
	"turtlesim"
	"std_msgs")
rosidl_target_interfaces(turtle_service ${PROJECT_NAME} "rosidl_typesupport_cpp") # need for custom srv
rclcpp_components_register_node(turtle_service PLUGIN "composition::reset_moving_turtle" EXECUTABLE moving_turtle_reset)
# set(node_plugins PLUGIN "${node_plugins}composition::reset_moving_turtle;$<TARGET_FILE:turtle_service\n" EXECUTABLE moving_turtle_reset)


# add turtle circle  publisher nodelet
add_library(turtle_circle SHARED
	src/turtle_circle_publisher.cpp)
target_compile_definitions(turtle_circle
	PRIVATE "SOFTWARE_TRAINING_DLL"
	)
ament_target_dependencies(turtle_circle
	"rclcpp"
	"rclcpp_components"
	"turtlesim"
	"geometry_msgs"
	"std_msgs")
# rosidl_target_interfaces(turtle_circle ${PROJECT_NAME} "rosidl_typesupport_cpp") # need this for custom messages - idk why tho
rclcpp_components_register_node(turtle_circle PLUGIN "composition::turtle_circle_publisher" EXECUTABLE turtle_circle_pub)
# set(node_plugins "${node_plugins}composition::turtle_circle_publisher;$<TARGET_FILE:turtle_circle\n>")


add_library(turtle_action_server SHARED
	src/action_turtle.cpp)
target_compile_definitions(turtle_action_server
	PRIVATE "SOFTWARE_TRAINING_DLL")
ament_target_dependencies(turtle_action_server
       "rclcpp"
       "rclcpp_components" 
       "turtlesim"
       "rclcpp_action"
       "std_msgs"
       "geometry_msgs")
rosidl_target_interfaces(turtle_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp") # needed for custom action
# with this we can execute the component as a node -cd ros2 run software_training turtle_action
rclcpp_components_register_node(turtle_action_server PLUGIN "composition::action_turtle" EXECUTABLE turtle_action)
# set(node_plugins "${node_plugins}composition::action_turtle;$<TARGET_FILE:turtle_action_server\n>")


# add_library(turtle_action_client SHARED
# 	src/action_client.cpp)
# target_compile_definitions(turtle_action_client
# 	PRIVATE "SOFTWARE_TRAINING_DLL")
# ament_target_dependencies(turtle_action_client
#        "rclcpp"
#        "rclcpp_components" 
#        "turtlesim"
#        "rclcpp_action"
#        "std_msgs"
#        "geometry_msgs")
# rosidl_target_interfaces(turtle_action_client ${PROJECT_NAME} "rosidl_typesupport_cpp") # needed for custom action
# # with this we can execute the component as a node - ros2 run software_training turtle_action
# rclcpp_components_register_node(turtle_action_client PLUGIN "composition::action_turtle" EXECUTABLE turtle_client)
# set(node_plugins "${node_plugins}composition::action_client;$<TARGET_FILE:turtle_action_client\n>")


#tell where to put binaries
install(TARGETS
	# turtle_request
	turtle_spawn
	turtle_circle
	clear_turtles
	turtle_service
	turtle_pub
	turtle_action_server
	# turtle_action_client
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)

#install launch file
install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
	)

#install configuration file (yaml files) for param server
install(DIRECTORY
	config
	DESTINATION share/${PROJECT_NAME}
	)

ament_package()