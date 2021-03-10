cmake_minimum_required(VERSION 3.5)

project(filters_node )

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

find_package(VTK REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_filters
  nodelet
  nodelet_topic_tools
  pcl_conversions
  pcl_msgs
  pluginlib
  rosbag
  rosconsole
  roscpp
  roslib
  sensor_msgs
  std_msgs
  message_generation
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
    dynamic_reconfigure
    geometry_msgs
    message_filters
    nodelet
    nodelet_topic_tools
    pcl_conversions
    pcl_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS
    Boost
    PCL
    message_runtime
)



add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp filters.cpp filters.h rosthread.cpp rosthread.h)

add_executable(filters_node ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
