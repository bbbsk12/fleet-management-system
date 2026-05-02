cmake_minimum_required(VERSION 3.8)
project(fleet_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FleetStatus.msg"
  "msg/RobotStatus.msg"
  "msg/TaskInfo.msg"
  "msg/TaskCmd.msg"
  "msg/TaskFb.msg"
  "msg/Waypoint.msg"
  "msg/TrafficMap.msg"
  "msg/LEDTask.msg"
  "msg/LEDStatus.msg"
  "srv/SubmitTask.srv"
  "srv/CancelTask.srv"
  "srv/GetRobotStatus.srv"
  "srv/LoadTrafficMap.srv"
  "srv/SaveTrafficMap.srv"
  "srv/RemoveRobot.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()