cmake_minimum_required(VERSION 3.16)
project(kai_gazebo)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_worlds test/test_worlds.py TIMEOUT 600)
endif()

# Hooks — register with ament so GZ_SIM_RESOURCE_PATH is set on source install/setup.bash
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

install(DIRECTORY
  worlds
  models
  DESTINATION share/${PROJECT_NAME})

# Terrain mesh of the La Spezia world. It is a Virtual Ocean Robotics
# Challenge (VORC) asset (Apache 2.0), tens of megabytes, so it is not kept in
# this repository: the build fetches it from a pinned commit, checks its hash
# and installs it next to the model that references it. The Sydney Regatta,
# Benderson Park and Sand Island sites come from Fuel at run time.
function(kai_fetch_asset url sha256 destination)
  # A file already there is trusted only if its hash matches: a partial or
  # stale download, or a bumped pin, triggers a new download.
  if(EXISTS ${destination})
    file(SHA256 ${destination} found)
    if(found STREQUAL sha256)
      return()
    endif()
    message(STATUS "Refetching ${destination}: hash ${found} is not the pinned one")
    file(REMOVE ${destination})
  endif()
  get_filename_component(dir ${destination} DIRECTORY)
  file(MAKE_DIRECTORY ${dir})
  message(STATUS "Fetching ${url}")
  # Downloaded to a temporary name and checked by hand, so that neither a
  # failed transfer nor a wrong hash ever leaves a file at the destination.
  set(partial ${destination}.part)
  file(DOWNLOAD ${url} ${partial} STATUS status TLS_VERIFY ON)
  list(GET status 0 code)
  if(NOT code EQUAL 0)
    list(GET status 1 reason)
    file(REMOVE ${partial})
    message(FATAL_ERROR "Could not fetch ${url}: ${reason}")
  endif()
  file(SHA256 ${partial} found)
  if(NOT found STREQUAL sha256)
    file(REMOVE ${partial})
    message(FATAL_ERROR "${url} has SHA256 ${found}, not the pinned ${sha256}")
  endif()
  file(RENAME ${partial} ${destination})
endfunction()

set(vorc https://raw.githubusercontent.com/osrf/vorc/bc71dd4/vorc_gazebo/models/marina/meshes)
set(assets ${CMAKE_CURRENT_BINARY_DIR}/assets)
kai_fetch_asset(${vorc}/terrain.dae
  0df3826427eba276adf34795e79f9702b4707a73d50a8dc68204232c4cdffe8e
  ${assets}/la_spezia/meshes/terrain.dae)
kai_fetch_asset(${vorc}/terrain.png
  9ac2bd2df5431a3b5222649f4773858099876299662c2de6237e99c206adfd04
  ${assets}/la_spezia/meshes/terrain.png)
install(DIRECTORY ${assets}/ DESTINATION share/${PROJECT_NAME}/models)

ament_package()
