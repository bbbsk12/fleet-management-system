cmake_minimum_required(VERSION 3.8)
project(fleet_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fconstexpr-steps=10000000)
  add_compile_options(-fbracket-depth=1024)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(fleet_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# ── Core library (map, routing, scheduler, occupancy) ──
add_library(fleet_manager_core STATIC
  src/traffic_manager.cpp
  src/traffic_manager_map_io.cpp
  src/traffic_manager_routing.cpp
  src/task_scheduler.cpp
  src/occupancy_manager.cpp)

target_include_directories(fleet_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(fleet_manager_core
  "rclcpp"
  "fleet_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_msgs")
target_link_libraries(fleet_manager_core yaml-cpp)

# ── Node logic library ──
add_library(fleet_manager_node_lib STATIC
  src/fleet_manager_node.cpp
  src/fleet_manager_node_navigation.cpp
  src/fleet_manager_node_services.cpp)

target_include_directories(fleet_manager_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(fleet_manager_node_lib
  "rclcpp"
  "rclcpp_action"
  "fleet_msgs"
  "geometry_msgs"
  "nav2_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_msgs")
target_link_libraries(fleet_manager_node_lib fleet_manager_core yaml-cpp)

# ── Executable ──
add_executable(fleet_manager_node src/fleet_manager_main.cpp)

target_include_directories(fleet_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(fleet_manager_node PUBLIC c_std_99 cxx_std_17)

target_link_libraries(fleet_manager_node fleet_manager_node_lib fleet_manager_core yaml-cpp)

ament_target_dependencies(fleet_manager_node
  "rclcpp"
  "rclcpp_action"
  "fleet_msgs"
  "geometry_msgs"
  "nav2_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_msgs")

install(TARGETS fleet_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
