cmake_minimum_required(VERSION 3.16)
project(gz_maritime_gui)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No CMAKE_BUILD_TYPE given; defaulting to RelWithDebInfo")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
    STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED COMPONENTS gui)
find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_wind REQUIRED)
find_package(gz_ocean_current REQUIRED)

find_package(tinyxml2 REQUIRED)

# --------------------------------------------------------------------------
# Core library, no Qt: reading the wind and the ocean current from their
# recipes, the messages that change them, the arrows that draw them, telling
# a refused change from an applied one, and a field's link to the world (its
# topic, the state service, its arrows) that both panels share.
# --------------------------------------------------------------------------
add_library(${PROJECT_NAME}_core SHARED
  src/Fields.cc
  src/FieldLink.cc
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    gz-sim::core
    gz-msgs::core
    gz_wind::gz_wind
    gz_ocean_current::gz_ocean_current
  PRIVATE
    gz-transport::core
    tinyxml2::tinyxml2
)

# --------------------------------------------------------------------------
# The GUI plugins, one per field. A library's name is the name a
# <plugin filename="..."> in a world's <gui> block uses, and its QML sits in
# its .qrc under the same name.
# --------------------------------------------------------------------------
find_package(Qt6 REQUIRED COMPONENTS Core Quick QuickControls2)
set(gui_plugins OceanCurrentPanel WindPanel)
foreach(plugin ${gui_plugins})
  # moc by hand: AUTOMOC does not take .hh files as headers.
  qt6_wrap_cpp(${plugin}_moc src/${plugin}.hh)
  qt6_add_resources(${plugin}_rcc src/${plugin}.qrc)
  add_library(${plugin} SHARED
    src/${plugin}.cc
    ${${plugin}_moc}
    ${${plugin}_rcc}
  )
  target_link_libraries(${plugin}
    PRIVATE
      ${PROJECT_NAME}_core
      gz-sim::gui
      gz-gui::gz-gui
      tinyxml2::tinyxml2
  )
endforeach()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# The plugins in a folder of their own: the GUI's plugin menu lists every
# library on GZ_GUI_PLUGIN_PATH, and a merged install puts every package's
# libraries in lib.
install(TARGETS ${gui_plugins}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # The core library, without a GUI.
  ament_add_gtest(fields_test test/fields_test.cc)
  if(TARGET fields_test)
    target_link_libraries(fields_test ${PROJECT_NAME}_core)
  endif()

  # The plugin in a running GUI, from the build tree.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gui_reads_recipes test/test_gui_reads_recipes.py
    TIMEOUT 240
    APPEND_ENV GZ_GUI_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR})
endif()

# GZ_GUI_PLUGIN_PATH hook so gz-sim's GUI finds the plugin library.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_package()
