# gz_find_ogre_next(): set OGRE_NEXT_INCLUDE_DIR to the Ogre Next headers
# the resolved gz-rendering-ogre2 uses. Installed with gz_waves_rendering, so
# every package that talks to Ogre directly finds the same copy the same way:
# find_package(gz_waves_rendering) then gz_find_ogre_next().
macro(gz_find_ogre_next)
  # Discover the Ogre Next headers. We must compile against the SAME Ogre Next the
  # resolved gz-rendering-ogre2 uses (mixing patch versions silently swaps in slow
  # Hlms/barrier paths). The previous approach parsed gz-rendering::ogre2's
  # LOCATION, but that property is empty for imported/interface targets (it is
  # NOTFOUND on the ROS gz vendor + Gazebo Jetty), so it wrongly fell back to a
  # hardcoded /usr/include/OGRE-2.3. Instead, search for the OGRE-Next headers,
  # seeded by the ogre2 target's own interface include dirs. PATH_SUFFIXES exclude
  # a bare "OGRE" so we never pick up classic Ogre 1.x (/usr/include/OGRE,
  # rviz_ogre_vendor), whose ABI-incompatible headers would break the build.
  get_target_property(_ogre2_incs gz-rendering::ogre2 INTERFACE_INCLUDE_DIRECTORIES)

  # A cached OGRE_NEXT_INCLUDE_DIR can go stale (e.g. the vendor was uninstalled);
  # find_path would keep the old value even though the dir is gone. Drop it first
  # so the search below re-runs whenever OgreRoot.h is no longer where we cached.
  if(OGRE_NEXT_INCLUDE_DIR AND NOT EXISTS "${OGRE_NEXT_INCLUDE_DIR}/OgreRoot.h")
    unset(OGRE_NEXT_INCLUDE_DIR CACHE)
  endif()

  # Pass 1: vendor / target-hinted headers only. NO_DEFAULT_PATH skips the system
  # dirs so a stray /usr/include/OGRE-2.x (a plain libogre-next-dev) can't shadow
  # the gz vendor copy that gz-rendering-ogre2 was actually built against — mixing
  # patch versions silently swaps in the slow Hlms/barrier paths.
  find_path(OGRE_NEXT_INCLUDE_DIR
    NAMES OgreRoot.h
    HINTS ${_ogre2_incs}
    PATHS /opt/ros/$ENV{ROS_DISTRO}/opt/gz_ogre_next_vendor/include
    PATH_SUFFIXES OGRE-Next OGRE-2.4 OGRE-2.3
    NO_DEFAULT_PATH
  )
  # Pass 2: fall back to the system paths (a plain libogre-next-dev with no vendor
  # present). Only runs when pass 1 left OGRE_NEXT_INCLUDE_DIR unset.
  find_path(OGRE_NEXT_INCLUDE_DIR
    NAMES OgreRoot.h
    PATH_SUFFIXES OGRE-Next OGRE-2.4 OGRE-2.3
  )
  if(NOT OGRE_NEXT_INCLUDE_DIR)
    message(FATAL_ERROR
      "OGRE-Next headers (OgreRoot.h) not found. Install libogre-next-dev or the "
      "gz_ogre_next_vendor package.")
  endif()
  message(STATUS "Ogre Next headers: ${OGRE_NEXT_INCLUDE_DIR}")
endmacro()
