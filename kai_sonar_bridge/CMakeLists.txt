cmake_minimum_required(VERSION 3.16)
project(kai_sonar_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sonar REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# --------------------------------------------------------------------------
# The bridge: one node per vehicle, converting every sonar it is given from
# its gz-transport topic to marine_acoustic_msgs, with optional images for
# viewing. Composable, and an executable of the same name.
# --------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED src/SonarBridge.cc)
target_link_libraries(${PROJECT_NAME}
  gz_sonar::gz_sonar-msgs
  gz-transport::core
  gz-msgs::core
  ${marine_acoustic_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "kai_sonar_bridge::SonarBridge"
  EXECUTABLE sonar_bridge)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# An example ROS 2 consumer of the sonar messages: ros2 run kai_sonar_bridge read_sonar.py
install(PROGRAMS examples/read_sonar.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
