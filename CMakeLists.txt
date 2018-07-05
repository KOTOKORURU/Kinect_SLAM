cmake_minimum_required(VERSION 2.8.11)
cmake_policy(SET CMP0003 NEW)





#ros parallel
IF ("${ROS_PARALLEL_JOBS}" EQUAL "")
  set(ROS_PARALLEL_JOBS 1)
endif ("${ROS_PARALLEL_JOBS}" EQUAL "")

#for build executables
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

#library
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -msse3 -std=c++11 -pthread -O3 -march=native")
#ROS#
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

find_package(catkin REQUIRED COMPONENTS
             pcl_conversions pcl_ros roscpp image_transport cv_bridge sensor_msgs cmake_modules
             geometry_msgs visualization_msgs std_msgs tf message_generation message_filters rosbag rosconsole)

project(kinectslam)

add_service_files(DIRECTORY
srv
FILES
kinectslam_ros_ui_str.srv
kinectslam_ros_ui_float.srv
kinectslam_ros_ui_bool.srv
kinectslam_ros_ui.srv
)
generate_messages()

#ros package
catkin_package(
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs 
  DEPENDS message_runtime std_msgs 
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Required 3rd party
# PCL #######################
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Octomap ###################
find_package(octomap 1.6 REQUIRED)
include_directories(SYSTEM BEFORE ${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

# g2o #######################
#find_package(G2O REQUIRED)
#include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
#link_directories(${G2O_LIBRARY_DIRS})
#link_libraries(${G2O_LIBRARIES})
#message("G2o-libraries " ${G2O_LIBRARIES})

# Qt ########################
find_package(Qt5Widgets)
find_package(Qt5OpenGL)
find_package(Qt5Concurrent)

add_definitions(-DQT_NO_KEYWORDS)

# OpenMP ########################
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
# This is necessary as all ui files etc will get dumped in the bottom of then binary directory. 
include_directories(SYSTEM ${CMAKE_CURRENT_BINARY_DIR} ${QT_QTOPENGL_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

SET(${CMAKE_LIBRARY_PATH} ${LIBRARY_OUTPUT_PATH})

ADD_DEFINITIONS(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)

#opencv
set(OpenCV_DIR $ENV{OpenCV_DIR})
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#if the path exists
IF (NOT "${OpenCV_DIR}" EQUAL "")
    MESSAGE("Using OpenCV from " ${OpenCV_DIR})
ENDIF (NOT "${OpenCV_DIR}" EQUAL "")


#eigen
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# Some QT Magic (MOC Creation)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp  src/kinect_listener.h  src/gl_viewer.h)

#include/local/*.hpp src/qtros.h src/kinect_listener.h src/qt_gui.h src/graph_optimizer.h src/glviewer.h src/ros_service_ui.h)

qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)



SET(ADDITIONAL_SOURCES src/kinect_listener.cpp
    src/main.cpp src/parameter_setting.cpp
    src/model.cpp src/header.cpp
    src/scoped_timer.cpp
    src/Convert.cpp
    src/gl_viewer.cpp
    src/Frame.cpp
    src/MapPoint.cpp
    src/KeyFrame.cpp
    src/Map.cpp
    src/ORBextractor.cpp
    src/KeyFrameDataBase.cpp
    src/ORBmatcher.cpp
    src/Sim3Solver.cpp
    src/Optimizer.cpp
    src/LoopClosing.cpp
    src/LocalMapper.cpp
    src/Tracking.cpp
    )

include_directories(${PROJECT_SOURCE_DIR}/include/
                    ${PROJECT_SOURCE_DIR}
                    ${PROJECT_SOURCE_DIR}/Thirdparty/)

source_group("include" FILES ${PROJECT_SOURCE_DIR}/include/)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS)

include_directories(SYSTEM /usr/include/suitesparse/)
add_executable(kinectslam ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${ADDITIONAL_SOURCES})
add_dependencies(kinectslam kinectslam_gencpp)

SET(LIBS_LINK GL GLU cholmod cxsparse Qt5::Widgets Qt5::OpenGL Qt5::Concurrent ${G2O_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS})

#link_libraries
target_link_libraries(kinectslam ${LIBS_LINK} ${catkin_LIBRARIES}
                       ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
                       ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
                      )
add_executable(bin_vocabulary Vocabulary/bin_vocabulary.cpp)
target_link_libraries(bin_vocabulary ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so ${OpenCV_LIBS})
