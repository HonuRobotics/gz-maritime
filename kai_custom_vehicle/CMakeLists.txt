cmake_minimum_required(VERSION 3.14)
project(kai_custom_vehicle)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Put the models directory on GZ_SIM_RESOURCE_PATH so model://custom_usv
# resolves. The model is deliberately not named after the package: Gazebo
# tries model:// names relative to the current directory first, and a source
# checkout has a directory called kai_custom_vehicle.
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh.in")

# The default instance, model://custom_usv, for plain `gz sim` and for world
# files: the URDF and the model expanded with the default name, the bridge
# config rendered from its template, and model.config. kai_bringup's
# spawn_vehicle.launch.xml renders the same three sources for any other name.
set(model_dir models/custom_usv)
xacro_add_files(
  urdf/custom_usv.urdf.xacro
  ${model_dir}/model.sdf.xacro
  INSTALL DESTINATION ${model_dir})
set(name custom_usv)
configure_file(config/ros_gz_bridge.yaml.in
  ${CMAKE_CURRENT_BINARY_DIR}/ros_gz_bridge.yaml @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ros_gz_bridge.yaml
  DESTINATION share/${PROJECT_NAME}/${model_dir})

install(DIRECTORY urdf models config rviz launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/instance_rviz.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_generation test/test_generation.py TIMEOUT 120)
  # NOCAPTURE so the launch output printed at teardown reaches the test logs
  # even when every assertion passes.
  ament_add_pytest_test(test_two_instances test/test_two_instances.py
    TIMEOUT 300 NOCAPTURE)
endif()

ament_package()
