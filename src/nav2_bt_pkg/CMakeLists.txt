cmake_minimum_required(VERSION 3.8)
project(nav2_bt_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

set(DEPENDENCIES
  ament_cmake
  ament_index_cpp
  behaviortree_cpp_v3
  geometry_msgs
  nav2_behavior_tree
  nav2_msgs
  nav2_msgs
  nav2_util
  rclcpp
  tf2_ros
)

add_library(nav2_talker_action_bt_node SHARED plugins/action/talker_action.cpp)
ament_target_dependencies(nav2_talker_action_bt_node ${DEPENDENCIES})
target_compile_definitions(nav2_talker_action_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_compile_features(nav2_talker_action_bt_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(nav2_bt_runner 
  src/nav2_bt_runner.cpp
  src/server_handler.cpp
)
ament_target_dependencies(nav2_bt_runner ${DEPENDENCIES})
target_include_directories(nav2_bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nav2_bt_runner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS nav2_bt_runner nav2_talker_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include DESTINATION include)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
