cmake_minimum_required(VERSION 3.8)
project(mybot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(autopatrol_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(local_init src/local_init.cpp)
add_executable(tf_listen src/tf_listen.cpp)
add_executable(action_client src/action_client.cpp)
add_executable(follow_way_point src/follow_way_point.cpp)
add_executable(PatrolController src/PatrolController.cpp)


ament_target_dependencies(tf_listen rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs)
ament_target_dependencies(local_init rclcpp geometry_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(action_client rclcpp geometry_msgs tf2 tf2_geometry_msgs nav2_msgs)
ament_target_dependencies(follow_way_point rclcpp geometry_msgs tf2 tf2_geometry_msgs nav2_msgs)
ament_target_dependencies(
  PatrolController 
  rclcpp 
  geometry_msgs 
  tf2 
  tf2_geometry_msgs 
  nav2_msgs 
  autopatrol_interface 
  sensor_msgs
  cv_bridge
  )

 target_link_libraries(
    PatrolController
    ${OpenCV_LIBRARIES}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(
  DIRECTORY
  urdf
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
  local_init
  tf_listen
  action_client
  follow_way_point
  PatrolController
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
