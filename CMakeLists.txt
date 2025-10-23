cmake_minimum_required(VERSION 3.8)
project(patrol_behavior_tree)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_executable(patrol_bt_node
  src/patrol_bt_node.cpp
  src/bt_actions.cpp
)

ament_target_dependencies(patrol_bt_node
  rclcpp
  geometry_msgs
  sensor_msgs
  behaviortree_cpp
  ament_index_cpp
)

install(TARGETS patrol_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_bt_actions test/test_bt_actions.cpp)
  
  if(TARGET test_bt_actions)
    target_link_libraries(test_bt_actions patrol_bt_node)
    ament_target_dependencies(test_bt_actions
      rclcpp
      behaviortree_cpp
    )
  endif()
endif()

ament_package()