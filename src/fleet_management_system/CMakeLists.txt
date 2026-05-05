cmake_minimum_required(VERSION 3.8)
project(fleet_management_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Zenoh 配置，供 launch 文件按包路径访问
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/config/zenoh
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装 maps 目录
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/maps
  DESTINATION share/${PROJECT_NAME}
)

# 安装脚本目录
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# 安装文档目录
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/docs
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Web 后端脚本，避免 launch 依赖源码绝对位置
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../../apps/fleet_web_ui/backend
  DESTINATION share/${PROJECT_NAME}/web_ui
  PATTERN ".venv" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "__pycache__/*" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
