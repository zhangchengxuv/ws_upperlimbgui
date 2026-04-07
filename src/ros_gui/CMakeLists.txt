cmake_minimum_required(VERSION 3.8)
project(ros_gui)

cmake_policy(SET CMP0071 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================
# find dependencies
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# ========================================
# ROS2 Interfaces
# ========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlCommand.msg"
  "msg/RobotState.msg"
  "msg/SystemState.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# ========================================
# Executable
# ========================================
add_executable(ros_gui_node
  src/main.cpp
  src/mainwindow.cpp
  src/mainwindow.h
  resources.qrc
)

target_include_directories(ros_gui_node PRIVATE
  include
  src
)

ament_target_dependencies(ros_gui_node
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(ros_gui_node
  Qt5::Widgets
)

# 让可执行文件依赖消息类型支持
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(ros_gui_node
  ${cpp_typesupport_target}
)

install(TARGETS
  ros_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
