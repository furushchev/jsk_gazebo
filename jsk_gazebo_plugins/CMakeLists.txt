cmake_minimum_required(VERSION 2.8.3)
project(jsk_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  magnetic_joint
  DEPENDES gazebo SDF)

set(DEPEND_LIBRARIES
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES})

add_library(magnetic_joint SHARED src/magnetic_joint.cpp)
target_link_libraries(magnetic_joint ${DEPEND_LIBRARIES})

install(TARGETS magnetic_joint
  DESTINATION ${CAKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
