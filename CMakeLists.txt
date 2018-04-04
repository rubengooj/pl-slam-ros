SET(PROJECT_NAME plslam_ros)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

set(DEFAULT_HAS_MRPT ON)
set(HAS_MRPT ${DEFAULT_HAS_MRPT} CACHE BOOL "Build the PointGrey Bumblebee2 SVO application that employs the MRPT library")


SET(OpenCV_DIR "/home/ruben/libs/opencv/build")
SET(MRPT_DIR   "/home/ruben/libs/mrpt/build")

# add mrpt library if compiled with it
if(HAS_MRPT)
find_package(MRPT REQUIRED base opengl gui hwdrivers)
set(MRPT_DONT_USE_DBG_LIBS 1) #use release libraries for linking even if "Debug" CMake build
add_definitions(-DHAS_MRPT)
endif(HAS_MRPT)

# YAML library
find_library(YAML_CPP_LIBRARIES yaml-cpp)
if(NOT YAML_CPP_LIBRARIES)
  # If yaml-cpp not found in the system, try finding it as a user CMake-generated project
  find_package(yaml-cpp REQUIRED)
  include_directories(${YAML_CPP_INCLUDE_DIRS})
endif(NOT YAML_CPP_LIBRARIES)

# Add catkin and required ROS packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  cmake_modules
  nav_msgs
  std_msgs
  visualization_msgs
  sensor_msgs
  image_transport
  cv_bridge
  tf
)

# Add plain cmake packages 
find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system filesystem)	
find_package(Eigen3 REQUIRED)
find_package(X11 REQUIRED)

# Include dirs
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ~/code/stvo-pl/3rdparty/line_descriptor/include/
  ~/code/stvo-pl/include/
  ~/code/pl-slam/3rdparty/DBoW2/include/
  ~/code/pl-slam/include/
  ${catkin_INCLUDE_DIRS}
  ${X11_INCLUDE_DIRS}
)

# Set link libraries
list(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ~/code/stvo-pl/3rdparty/line_descriptor/lib/liblinedesc.so
  ~/code/stvo-pl/lib/libstvo.so
  ~/code/pl-slam/3rdparty/DBoW2/lib/libDBoW2.so
  ~/code/pl-slam/lib/libplslam.so
  ${catkin_LIBRARIES}
  ${X11_LIBRARIES}
)

# List all files (headers) contained by StVO-PL library
file(GLOB_RECURSE all_include_files RELATIVE "${CMAKE_SOURCE_DIR}" *.h *.hpp)

# Visualize the files of this directory in IDE creating an custom empty target
add_custom_target( plslam_includes DEPENDS ${all_include_files} SOURCES ${all_include_files} )
                                
# Create Executables                      
ADD_EXECUTABLE(plslam plslam_node.cpp)

if(HAS_MRPT)
TARGET_LINK_LIBRARIES(plslam ${LINK_LIBS} ${MRPT_LIBS} )
endif(HAS_MRPT)










