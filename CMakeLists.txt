cmake_minimum_required(VERSION 3.24)
project(novatel_LiDAR_Mpping)


set(CMAKE_CXX_STANDARD 14 -O3)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
include_directories( include)
find_package(PCL REQUIRED)

ADD_LIBRARY(sophus SHARED include/sophus/se3.cpp
        include/sophus/so3.cpp
        include/sophus/so3.h
        include/sophus/se3.h)

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

FIND_PACKAGE (libLAS REQUIRED)

find_package(GTSAM REQUIRED QUIET)
include_directories(${GTSAM_INCLUDE_DIR})
find_package( Cholmod REQUIRED )
include_directories( ${CHOLMOD_INCLUDE_DIR} )

find_package(rosbag REQUIRED)
include_directories(${rosbag_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(tf_conversions REQUIRED)
include_directories(${tf_conversions_INCLUDE_DIRS})

find_package(cv_bridge REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS})

find_package(novatel_oem7_msgs REQUIRED)
include_directories(${novatel_oem7_msgs_INCLUDE_DIRS})

find_package(gps_common REQUIRED)
include_directories(${gps_common_REQUIRED_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

#ADD_LIBRARY(poseIO SHARED include/g2oIO/PoseGraphIO.h include/g2oIO/PoseGraphIO.cpp)
#target_link_libraries(poseIO g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES})

add_library(registration
        include/registration/registration.cpp
        include/registration/registration.h
        )
target_link_libraries(registration
        ${PCL_LIBRARIES})

add_library(GPS
          include/GPS/gpsTools.cpp include/GPS/gpsTools.h )
target_link_libraries(GPS
        ${novatel_oem7_msgs_LIBRARIES}
     )


add_library(novatel
        novatel_mapping.cpp novatel_mapping.h   )
target_link_libraries(novatel
        GPS
        ${libLAS_LIBRARIES}
        ${rosbag_LIBRARIES}
        ${novatel_oem7_msgs_LIBRARIES}
        ${PCL_LIBRARIES})


add_library(readbag
        include/DataIO/ReadBag.h include/DataIO/ReadBag.cpp)
target_link_libraries(readbag
        ${novatel_oem7_msgs_LIBRARIES}
        GPS
        ${rosbag_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        ${PCL_LIBRARIES})



add_executable(Mapping_Multi_thread main.cpp   include/CSV/CSVio.cpp include/CSV/CSVio.h
        )
target_link_libraries(Mapping_Multi_thread

        readbag
        sophus
        novatel
        pcl_filters
        ${GTSAM_LIBRARY_DIRS}
        ${tf_conversions_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${PCL_LIBRARIES}
        registration )