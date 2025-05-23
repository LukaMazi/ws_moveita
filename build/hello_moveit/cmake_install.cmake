# Install script for directory: /home/lampa66/ws_moveita/src/hello_moveit

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lampa66/ws_moveita/install/hello_moveit")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/collision_detection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/collision_detection")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/collision_detection"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hello_moveit" TYPE EXECUTABLE FILES "/home/lampa66/ws_moveita/build/hello_moveit/collision_detection")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/collision_detection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/collision_detection")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/collision_detection"
         OLD_RPATH "/home/lampa66/ws_moveit/install/moveit_ros_planning_interface/lib:/home/lampa66/ws_moveit/install/moveit_visual_tools/lib:/home/lampa66/ws_moveit/install/moveit_ros_move_group/lib:/home/lampa66/ws_moveit/install/moveit_ros_warehouse/lib:/home/lampa66/ws_moveit/install/moveit_ros_planning/lib:/home/lampa66/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/lampa66/ws_moveit/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/lampa66/ws_moveit/install/srdfdom/lib:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/collision_detection")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/hello_moveit" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/hello_moveit")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/hello_moveit"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hello_moveit" TYPE EXECUTABLE FILES "/home/lampa66/ws_moveita/build/hello_moveit/hello_moveit")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/hello_moveit" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/hello_moveit")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/hello_moveit"
         OLD_RPATH "/home/lampa66/ws_moveit/install/moveit_ros_planning_interface/lib:/home/lampa66/ws_moveit/install/moveit_visual_tools/lib:/home/lampa66/ws_moveit/install/moveit_ros_move_group/lib:/home/lampa66/ws_moveit/install/moveit_ros_warehouse/lib:/home/lampa66/ws_moveit/install/moveit_ros_planning/lib:/home/lampa66/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/lampa66/ws_moveit/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/lampa66/ws_moveit/install/srdfdom/lib:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/hello_moveit")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/mojmoj" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/mojmoj")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/mojmoj"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hello_moveit" TYPE EXECUTABLE FILES "/home/lampa66/ws_moveita/build/hello_moveit/mojmoj")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/mojmoj" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/mojmoj")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/mojmoj"
         OLD_RPATH "/home/lampa66/ws_moveit/install/moveit_ros_planning_interface/lib:/home/lampa66/ws_moveit/install/moveit_visual_tools/lib:/home/lampa66/ws_moveit/install/moveit_ros_move_group/lib:/home/lampa66/ws_moveit/install/moveit_ros_warehouse/lib:/home/lampa66/ws_moveit/install/moveit_ros_planning/lib:/home/lampa66/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/lampa66/ws_moveit/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/lampa66/ws_moveit/install/srdfdom/lib:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/mojmoj")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/srcek" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/srcek")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/srcek"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hello_moveit" TYPE EXECUTABLE FILES "/home/lampa66/ws_moveita/build/hello_moveit/srcek")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/srcek" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/srcek")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/srcek"
         OLD_RPATH "/home/lampa66/ws_moveit/install/moveit_ros_planning_interface/lib:/home/lampa66/ws_moveit/install/moveit_visual_tools/lib:/home/lampa66/ws_moveit/install/moveit_ros_move_group/lib:/home/lampa66/ws_moveit/install/moveit_ros_warehouse/lib:/home/lampa66/ws_moveit/install/moveit_ros_planning/lib:/home/lampa66/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/lampa66/ws_moveit/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/lampa66/ws_moveit/install/srdfdom/lib:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/srcek")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/simple_mover" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/simple_mover")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/simple_mover"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hello_moveit" TYPE EXECUTABLE FILES "/home/lampa66/ws_moveita/build/hello_moveit/simple_mover")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/simple_mover" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/simple_mover")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/simple_mover"
         OLD_RPATH "/home/lampa66/ws_moveit/install/moveit_ros_planning_interface/lib:/home/lampa66/ws_moveit/install/moveit_visual_tools/lib:/home/lampa66/ws_moveit/install/moveit_ros_move_group/lib:/home/lampa66/ws_moveit/install/moveit_ros_warehouse/lib:/home/lampa66/ws_moveit/install/moveit_ros_planning/lib:/home/lampa66/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/home/lampa66/ws_moveit/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/lampa66/ws_moveit/install/srdfdom/lib:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hello_moveit/simple_mover")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit/launch" TYPE DIRECTORY FILES "/home/lampa66/ws_moveita/src/hello_moveit/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/hello_moveit")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/hello_moveit")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit/environment" TYPE FILE FILES "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit/environment" TYPE FILE FILES "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit" TYPE FILE FILES "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit" TYPE FILE FILES "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit" TYPE FILE FILES "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit" TYPE FILE FILES "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit" TYPE FILE FILES "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_index/share/ament_index/resource_index/packages/hello_moveit")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit/cmake" TYPE FILE FILES
    "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_core/hello_moveitConfig.cmake"
    "/home/lampa66/ws_moveita/build/hello_moveit/ament_cmake_core/hello_moveitConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hello_moveit" TYPE FILE FILES "/home/lampa66/ws_moveita/src/hello_moveit/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/lampa66/ws_moveita/build/hello_moveit/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
