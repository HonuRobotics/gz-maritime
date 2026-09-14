cmake_minimum_required(VERSION 3.14)
project(tutorial_usv_gazebo)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(tutorial_usv_description REQUIRED)

# Put this package's models directory on GZ_SIM_RESOURCE_PATH so
# model://tutorial_usv resolves. The model is deliberately not named after the
# package: Gazebo tries model:// names relative to the current directory
# first, and a source checkout has a directory called tutorial_usv_gazebo.
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh.in")

# The default instance, model://tutorial_usv: model.sdf expanded from
# model.sdf.xacro with the default name, its model.config, and the URDF it
# merges, copied from the description's install so the model directory is
# self-contained. configure_vehicle.py produces the same three files, plus
# the bridge config, for any other instance name.
set(model_dir share/${PROJECT_NAME}/models/tutorial_usv)
xacro_add_files(model.sdf.xacro INSTALL DESTINATION models/tutorial_usv)
install(FILES
  models/tutorial_usv/model.config
  ${tutorial_usv_description_DIR}/../urdf/tutorial_usv.urdf
  DESTINATION ${model_dir})

# The bridge config for the default instance, rendered from the same
# template configure_vehicle.py fills in for the others.
set(name tutorial_usv)
configure_file(config/ros_gz_bridge.yaml.in
  ${CMAKE_CURRENT_BINARY_DIR}/ros_gz_bridge.yaml @ONLY)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/ros_gz_bridge.yaml
  config/ros_gz_bridge.yaml.in
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES model.sdf.xacro DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/configure_vehicle.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_configure_vehicle test/test_configure_vehicle.py
    TIMEOUT 180)
  ament_add_pytest_test(test_model_generation test/test_model_generation.py
    TIMEOUT 120)
  # NOCAPTURE so the sim output printed at teardown reaches the test logs
  # even when every assertion passes.
  ament_add_pytest_test(test_gz_launch test/test_gz_launch.py TIMEOUT 420
    NOCAPTURE)
  ament_add_pytest_test(test_ros_launch test/test_ros_launch.py TIMEOUT 300
    NOCAPTURE)
endif()

ament_package()
