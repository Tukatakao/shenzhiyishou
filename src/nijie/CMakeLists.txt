cmake_minimum_required(VERSION 3.8)
project(nijie)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_library(kinematics_lib
  src/ik.cpp
)
ament_target_dependencies(kinematics_lib
  rclcpp
)
target_include_directories(kinematics_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(ik_topic_node
  src/topic_ik.cpp
  src/ik.cpp
)
target_link_libraries(ik_topic_node
  kinematics_lib
)
ament_target_dependencies(ik_topic_node
  rclcpp
  geometry_msgs
  sensor_msgs
)
target_include_directories(ik_topic_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS
  kinematics_lib
  ik_topic_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)


ament_package()
