cmake_minimum_required(VERSION 3.14)
project(tutorial_usv_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Expand the xacro into a concrete URDF at build time, installed next to the
# xacro as urdf/tutorial_usv.urdf. tutorial_usv_gazebo merges this file into
# its simulation model.
xacro_add_files(urdf/tutorial_usv.urdf.xacro INSTALL DESTINATION urdf)

install(DIRECTORY
  urdf
  rviz
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py TIMEOUT 120)
endif()

ament_package()
