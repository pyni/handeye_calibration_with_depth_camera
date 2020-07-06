cmake_minimum_required(VERSION 2.8.3)
project(3dposedetection)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
file(GLOB_RECURSE Demo_PCL_headers
                  ${CMAKE_CURRENT_SOURCE_DIR}/*.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
set(OpenGR_DIR /home/yuan/doc/objectpose/4PCS_ICP/OpenGR/build/)


find_package(PCL 1.7 REQUIRED)
find_package(OpenGR REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenGR_INCLUDE_DIR})
link_directories(${OpenGR_LIB_DIR}) 
find_package(catkin REQUIRED  COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
tf)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS}) 
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if( PCL_FOUND )
  message (STATUS "[Demos] PCL found, creating target OpenGR-PCLWrapper")
 catkin_package( )
  set(Demo_PCL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/3dpose.cpp ) 
  include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
  add_executable(singleobject ${Demo_PCL_SRC} ${Demo_PCL_headers})
  target_link_libraries(singleobject ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})


  set(Demo_PCL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/3dpose_forcalibration.cpp ) 
  include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
  add_executable(calibration ${Demo_PCL_SRC} ${Demo_PCL_headers})
  target_link_libraries(calibration ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})





 set(Demo_PCL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/real_test.cpp ) 
  include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
  add_executable(real_test ${Demo_PCL_SRC} ${Demo_PCL_headers})
  target_link_libraries(real_test ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})





endif()
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 3dposedetection
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
#)
  
