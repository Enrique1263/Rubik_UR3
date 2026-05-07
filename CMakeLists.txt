cmake_minimum_required(VERSION 3.8)
project(rubik_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)

file(GLOB KOCIEMBA_SOURCES "src/libkociemba/*.c")

# Silenciar avisos de la librería externa
set_source_files_properties(${KOCIEMBA_SOURCES} PROPERTIES COMPILE_FLAGS "-w")

add_executable(rubik_solver_node 
    src/rubik_solver_node.cpp 
    ${KOCIEMBA_SOURCES}
)
target_include_directories(rubik_solver_node PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/libkociemba
)

ament_target_dependencies(rubik_solver_node rclcpp std_msgs)

add_executable(rubik_sim_node src/rubik_sim_node.cpp)
target_include_directories(rubik_sim_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(
  rubik_sim_node
  "rclcpp"
  "std_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "std_srvs"
)

add_executable(rubik_sim_controller src/rubik_sim_controller.cpp)
target_include_directories(rubik_sim_controller PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(
  rubik_sim_controller
  "rclcpp"
  "std_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "std_srvs"
)

add_executable(ur_cube_controller src/ur_cube_controller.cpp)
target_include_directories(ur_cube_controller PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(
  ur_cube_controller
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "control_msgs"
  "sensor_msgs"
  "trajectory_msgs"
  "visualization_msgs"
  "geometry_msgs"
)

add_executable(cube_vision_node src/cube_vision_node.cpp)
target_include_directories(cube_vision_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(cube_vision_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)
target_link_libraries(cube_vision_node
  ${OpenCV_LIBRARIES}
)

add_executable(cube_sequencer src/cube_sequencer.cpp)
ament_target_dependencies(cube_sequencer 
  rclcpp 
  std_msgs
)

install(TARGETS rubik_sim_node ur_cube_controller cube_vision_node rubik_solver_node cube_sequencer rubik_sim_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
