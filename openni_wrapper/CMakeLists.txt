cmake_minimum_required(VERSION 2.8.3)
project(openni_wrapper)

set(CMAKE_CXX_FLAGS "-g -fPIC -std=c++0x -fpermissive")

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge nodelet class_loader clams dynamic_reconfigure)
find_package( OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)

set(OPENNI2.2_DIR ../OpenNI2/)
set(OPENNI2.2_BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../OpenNI2/)
link_directories(${OPENNI2.2_DIR}/Bin/x64-Release)
include_directories(${OPENNI2.2_DIR}/Include)

generate_dynamic_reconfigure_options(
  cfg/dynamic_parameters.cfg
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${clams_INCLUDE_DIRS}
)

set(HDRS
    include/ColorCallback.h
    include/OpenniWrapperNodelet.h
    include/DepthCallback.h
    include/OpenniWrapperNode.h
)

set(SRCS
    src/ColorCallback.cpp
    src/OpenniWrapperNodelet.cpp
    src/DepthCallback.cpp
    src/OpenniWrapperNode.cpp
    src/main.cpp
)

add_executable(openni_wrapper ${HDRS} ${SRCS})

add_library(openni_wrapper_lib  ${HDRS} ${SRCS})

 target_link_libraries(openni_wrapper
   ${catkin_LIBRARIES} OpenNI2 ${OpenCV_LIBS}
 )

 target_link_libraries(openni_wrapper_lib
   ${catkin_LIBRARIES} OpenNI2 ${OpenCV_LIBS}
 )

add_dependencies(openni_wrapper openni_wrapper_lib ${PROJECT_NAME}_gencfg)

add_custom_command(TARGET openni_wrapper_lib PRE_LINK COMMAND make ARGS -j2 WORKING_DIRECTORY ${OPENNI2.2_BUILD_DIR} COMMENT "Compiling Openni2.2. This could take some time.")

execute_process(COMMAND make WORKING_DIRECTORY ${OPENNI2.2_DIR})
