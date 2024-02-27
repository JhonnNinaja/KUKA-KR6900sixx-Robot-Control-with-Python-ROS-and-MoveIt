# Install script for directory: /home/jhonn/Documents/robo/kuka_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jhonn/Documents/robo/kuka_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jhonn/Documents/robo/kuka_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jhonn/Documents/robo/kuka_ws/install" TYPE PROGRAM FILES "/home/jhonn/Documents/robo/kuka_ws/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jhonn/Documents/robo/kuka_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jhonn/Documents/robo/kuka_ws/install" TYPE PROGRAM FILES "/home/jhonn/Documents/robo/kuka_ws/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jhonn/Documents/robo/kuka_ws/install/setup.bash;/home/jhonn/Documents/robo/kuka_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jhonn/Documents/robo/kuka_ws/install" TYPE FILE FILES
    "/home/jhonn/Documents/robo/kuka_ws/build/catkin_generated/installspace/setup.bash"
    "/home/jhonn/Documents/robo/kuka_ws/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jhonn/Documents/robo/kuka_ws/install/setup.sh;/home/jhonn/Documents/robo/kuka_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jhonn/Documents/robo/kuka_ws/install" TYPE FILE FILES
    "/home/jhonn/Documents/robo/kuka_ws/build/catkin_generated/installspace/setup.sh"
    "/home/jhonn/Documents/robo/kuka_ws/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jhonn/Documents/robo/kuka_ws/install/setup.zsh;/home/jhonn/Documents/robo/kuka_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jhonn/Documents/robo/kuka_ws/install" TYPE FILE FILES
    "/home/jhonn/Documents/robo/kuka_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/jhonn/Documents/robo/kuka_ws/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/jhonn/Documents/robo/kuka_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/jhonn/Documents/robo/kuka_ws/install" TYPE FILE FILES "/home/jhonn/Documents/robo/kuka_ws/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/jhonn/Documents/robo/kuka_ws/build/gtest/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_experimental/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_resources/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_rsi_simulator/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/moveit/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_kr10_support/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_kr120_support/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_kr150_support/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_kr16_support/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_kr210_support/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_kr3_support/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_kr5_support/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_kr6_support/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_lbr_iiwa_support/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_eki_hw_interface/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/kuka_experimental/kuka_rsi_hw_interface/cmake_install.cmake")
  include("/home/jhonn/Documents/robo/kuka_ws/build/tutorial/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/jhonn/Documents/robo/kuka_ws/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
