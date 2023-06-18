#include <iostream>
#include "DataIO/ReadBag.h"
#include "novatel_mapping.h"

//bestpos + GPS imu(RPY), transformation is linear rotation is integral the angular velocity
//get RPY and moving average 3
int main(int argc, char** argv){
    ros::init(argc, argv, "novatel_mapping");
    ReadBag rb;
    novatel_mapping nm;
    std::string bag_path = "/home/echo/bag/merge/raw_imu.bag";
    rb.read7720_Odom_Imu(bag_path);
    std::cout<<"odom ok"<<std::endl;
    nm.bag_path = bag_path;
    nm.save_path = "/home/echo/pcd";
    nm.global_path = "/home/echo/bag/7720_Lidar/global";
    nm.las_path = "/home/echo/bag/7720_Lidar/las";
    std::cout<<rb.oem7720Imu.size()<<std::endl;
    std::cout<<rb.oem7720Odom.size()<<std::endl;

    nm.oem7720Imu = rb.oem7720Imu;
    nm.oem7720Odom = rb.oem7720Odom;
    nm.movingAverangeFilterIMU();
    nm.calculateOdomVector();
    nm.transformPointCloud();
//    rb.writeOdom(nm.odom_buffer,"/home/echo/bag/7720_Lidar/GOOD_imu.bag");
//    rb.writeOdom(nm.odom_buffer_fast,"/home/echo/bag/7720_Lidar/GOOD_imu_fast.bag");

}
