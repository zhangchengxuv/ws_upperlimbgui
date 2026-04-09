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
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# 这里才是你要使用的自定义消息包
find_package(upperlimb_robot REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# ========================================
# Executable
# ========================================
add_executable(ros_gui_node
  src/main.cpp
  src/mainwindow.cpp
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
  upperlimb_robot
)

target_link_libraries(ros_gui_node
  Qt5::Widgets
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
