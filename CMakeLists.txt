cmake_minimum_required(VERSION 3.5)
project(filter_node)
if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check https://doc.qt.io/qt/deployment-android.html for more information.
# They need to be set before the find_package( ...) calls below.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()
#find_package(QT NAMES Qt5 COMPONENTS Widgets REQUIRED)
#find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  dynamic-reconfigure
  geometry-msgs
  message-filters
  nodelet
  nodelet-topic-tools
  pcl-conversions
  pcl-msgs
  pluginlib
  rosbag
  rosconsole
  roscpp
  roslib
  sensor-msgs
  std-msgs
  message-generation
)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(
    SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


catkin_package(
  CATKIN_DEPENDS
    dynamic-reconfigure
    geometry-msgs
    message-filters
    nodelet
    nodelet-topic-tools
    pcl-conversions
    pcl-msgs
    rosbag
    roscpp
    sensor-msgs
    std-msgs
  DEPENDS
    Boost
    PCL
    message-runtime
)



add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp filters.cpp filters.h rosthread.cpp rosthread.h)

add_executable(filter_node ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
