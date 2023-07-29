cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)
project(rf2o_laser_odometry)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  cmake_modules
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS cmake_modules nav_msgs roscpp sensor_msgs std_msgs tf
 DEPENDS Eigen
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  #${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} src/CLaserOdometry2D.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#Require C++11
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_EXTENSIONS OFF)
endif()

## Declare a cpp executable
add_executable(rf2o_laser_odometry_node src/CLaserOdometry2DNode.cpp)
target_link_libraries(rf2o_laser_odometry_node ${PROJECT_NAME} ${catkin_LIBRARIES})

#Require C++11
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(rf2o_laser_odometry_node PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET rf2o_laser_odometry_node PROPERTY CXX_STANDARD 11)
  set_property(TARGET rf2o_laser_odometry_node PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET rf2o_laser_odometry_node PROPERTY CXX_EXTENSIONS OFF)
endif()

################
##  Install   ##
################

install(TARGETS ${PROJECT_NAME} rf2o_laser_odometry_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
