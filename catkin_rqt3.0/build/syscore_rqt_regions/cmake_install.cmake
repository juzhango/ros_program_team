# Install script for directory: /home/teabot/my_workSpace/a/ros_program_team/catkin_rqt3.0/src/syscore_rqt_regions

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/teabot/my_workSpace/a/ros_program_team/catkin_rqt3.0/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/teabot/my_workSpace/a/ros_program_team/catkin_rqt3.0/build/syscore_rqt_regions/catkin_generated/installspace/syscore_rqt_regions.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/syscore_rqt_regions/cmake" TYPE FILE FILES
    "/home/teabot/my_workSpace/a/ros_program_team/catkin_rqt3.0/build/syscore_rqt_regions/catkin_generated/installspace/syscore_rqt_regionsConfig.cmake"
    "/home/teabot/my_workSpace/a/ros_program_team/catkin_rqt3.0/build/syscore_rqt_regions/catkin_generated/installspace/syscore_rqt_regionsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/syscore_rqt_regions" TYPE FILE FILES "/home/teabot/my_workSpace/a/ros_program_team/catkin_rqt3.0/src/syscore_rqt_regions/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syscore_rqt_regions/syscore_rqt_regions" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syscore_rqt_regions/syscore_rqt_regions")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syscore_rqt_regions/syscore_rqt_regions"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/syscore_rqt_regions" TYPE EXECUTABLE FILES "/home/teabot/my_workSpace/a/ros_program_team/catkin_rqt3.0/devel/lib/syscore_rqt_regions/syscore_rqt_regions")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syscore_rqt_regions/syscore_rqt_regions" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syscore_rqt_regions/syscore_rqt_regions")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syscore_rqt_regions/syscore_rqt_regions")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/syscore_rqt_regions/syscore_rqt_regions")
    endif()
  endif()
endif()
