cmake_minimum_required(VERSION 3.16)
project(warehouse_astar_planner VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(grid_astar_core SHARED src/grid_astar.cpp)
target_include_directories(grid_astar_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(grid_astar_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(warehouse_astar_planner SHARED src/astar_planner.cpp)
target_include_directories(warehouse_astar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(warehouse_astar_planner PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(warehouse_astar_planner grid_astar_core)
ament_target_dependencies(warehouse_astar_planner
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros)

pluginlib_export_plugin_description_file(nav2_core warehouse_astar_planner_plugin.xml)

install(TARGETS grid_astar_core warehouse_astar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES warehouse_astar_planner_plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(grid_astar_core_test test/grid_astar_test.cpp)
  target_link_libraries(grid_astar_core_test grid_astar_core)
  target_compile_options(grid_astar_core_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME grid_astar_core_test COMMAND grid_astar_core_test)
endif()

ament_export_include_directories(include)
ament_export_libraries(grid_astar_core warehouse_astar_planner)
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros)

ament_package()
