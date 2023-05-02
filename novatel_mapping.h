//
// Created by echo on 23-4-29.
//

#ifndef NOVATEL_LIDAR_MPPING_NOVATEL_MAPPING_H
#define NOVATEL_LIDAR_MPPING_NOVATEL_MAPPING_H

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include "tf/tf.h"
#include <Eigen/SVD>
#include "novatel_oem7_msgs/BESTPOS.h"
#include "GPS/gpsTools.h"
#include <Eigen/Geometry>
#include "CSV/CSVio.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include "pointType/pointTypes.h"
#include <pcl/io/pcd_io.h>
#include <liblas/liblas.hpp>
#include <iostream>
#include <fstream>
#include <iosfwd>
#include<algorithm>
#include <pcl/filters/voxel_grid.h>
class novatel_mapping {
public:
    novatel_mapping(){};
    gpsTools GT;
    Eigen::Vector4f quaternionAverage(std::vector<Eigen::Vector4f> quaternions);
    void movingAverangeFilterIMU( );
    void calculateOdomVector();
    void transformPointCloud();
    std::string bag_path;
    std::string save_path;
    std::string global_path;
    std::string las_path;
    pcl::PCDWriter writer;
    std::vector<novatel_oem7_msgs::BESTPOS> oem7720Odom;
    std::vector<sensor_msgs::Imu> oem7720Imu;
    std::vector<nav_msgs::Odometry> odom_buffer;
    std::vector<nav_msgs::Odometry> odom_buffer_fast;
    std::vector<Eigen::Quaterniond> q_buffer;
    std::vector<Eigen::Vector3d> t_buffer;
    std::vector<double> time_delay_fix_table;
    CSVio CSV;
    //las
    liblas::Header header;
    std::ofstream ofs;
    liblas::SpatialReference srs;
    double time_start;

private:
    //cloud bounding box

    float minPt[3] = { 9999, 9999, 9999};
    float maxPt[3] = { 0, 0, 0 };
    int Horizon_SCAN = 1800;     //lidar horizontal resolution (Velodyne:1800, Ouster:512,1024,2048, Livox Horizon: 4000)
};

#endif //NOVATEL_LIDAR_MPPING_NOVATEL_MAPPING_H
