//
// Created by echo on 23-4-29.
//

#include "novatel_mapping.h"

void novatel_mapping::movingAverangeFilterIMU( ) {
    Eigen::Vector4f q_1;
    Eigen::Vector4f q_2;

    for (int i = 0; i < oem7720Imu.size(); ++i) {
        if(i==0){
            q_2 = Eigen::Vector4f(oem7720Imu[i].orientation.x,oem7720Imu[i].orientation.y,oem7720Imu[i].orientation.z,oem7720Imu[i].orientation.w);
        }else if(i==1){
            q_1 = Eigen::Vector4f(oem7720Imu[i].orientation.x,oem7720Imu[i].orientation.y,oem7720Imu[i].orientation.z,oem7720Imu[i].orientation.w);
        } else{
            Eigen::Vector4f  q_0(oem7720Imu[i].orientation.x,oem7720Imu[i].orientation.y,oem7720Imu[i].orientation.z,oem7720Imu[i].orientation.w);
            std::vector<Eigen::Vector4f> rotate_all;
            rotate_all.push_back(q_0);
            rotate_all.push_back(q_1);
            rotate_all.push_back(q_2);
            Eigen::Vector4f result = quaternionAverage(rotate_all);
            q_2 = q_1;
            q_1 = q_0;
            oem7720Imu[i].orientation.x = result.x();
            oem7720Imu[i].orientation.y = result.y();
            oem7720Imu[i].orientation.z = result.z();
            oem7720Imu[i].orientation.w = result.w();
           // std::cout<<" q_0: "<<q_0.transpose()<<" result: "<<result.transpose()<<std::endl;
        }
    }
}
/// Method to find the average of a set of rotation quaternions using Singular Value Decomposition
/*
 * The algorithm used is described here:
 * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
 */
Eigen::Vector4f novatel_mapping::quaternionAverage(std::vector<Eigen::Vector4f> quaternions)
{
    if (quaternions.size() == 0)
    {
        std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
        return Eigen::Vector4f::Zero();
    }

    // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
    Eigen::Matrix4f A = Eigen::Matrix4f::Zero();

    for (int q=0; q<quaternions.size(); ++q)
        A += quaternions[q] * quaternions[q].transpose();

    // normalise with the number of quaternions
    A /= quaternions.size();

    // Compute the SVD of this 4x4 matrix
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::VectorXf singularValues = svd.singularValues();
    Eigen::MatrixXf U = svd.matrixU();

    // find the eigen vector corresponding to the largest eigen value
    int largestEigenValueIndex;
    float largestEigenValue;
    bool first = true;

    for (int i=0; i<singularValues.rows(); ++i)
    {
        if (first)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
            first = false;
        }
        else if (singularValues(i) > largestEigenValue)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
        }
    }

    Eigen::Vector4f average;
    average(0) = U(0, largestEigenValueIndex);
    average(1) = U(1, largestEigenValueIndex);
    average(2) = U(2, largestEigenValueIndex);
    average(3) = U(3, largestEigenValueIndex);

    return average;
}

void novatel_mapping::calculateOdomVector() {
    double time_all = (oem7720Imu.back().header.stamp - oem7720Imu.front().header.stamp).toSec();
    time_start = oem7720Imu.front().header.stamp.toSec();
    int totalSize = ceil(time_all*18000);
    std::cout<<"odom prepare size: "<<totalSize<<" Imu based size "<<oem7720Imu.size()*180<<std::endl;
    std::cout<<"time Diff: "<<time_all/oem7720Imu.size()<<std::endl;
    std::cout<<"imu time exceed: "<<time_all - oem7720Imu.size()*0.01<<std::endl;
    Eigen::Vector3d lla,curr_pose,next_pose;
    rosbag::Bag bag;
    bag.open("/home/echo/bag/7720_Lidar/GOOD_imu_fast.bag", rosbag::bagmode::Write);

    //100hz
    for (int i = 0; i < oem7720Imu.size(); ++i) {
        nav_msgs::Odometry curr_odom;
        curr_odom.header.frame_id = "map";
        ros::Time curr_pose_time = ros::Time(time_start + i*0.01);
//        curr_odom.header.stamp = curr_pose_time;
        curr_odom.header.stamp = oem7720Imu[i].header.stamp;
        curr_odom.header.seq = i;
        curr_odom.pose.pose.orientation = oem7720Imu[i].orientation;
        int odom_index = floor(i/10.0);
        if(odom_index+1 < oem7720Odom.size()){
            lla = Eigen::Vector3d (oem7720Odom[odom_index].lat, oem7720Odom[odom_index].lon, oem7720Odom[odom_index].hgt);
            GT.updateGPSpose(lla); //get GPS odom;
            curr_pose = GT.gps_pos_;
            //+1 means forward
            lla = Eigen::Vector3d (oem7720Odom[odom_index+1].lat, oem7720Odom[odom_index+1].lon, oem7720Odom[odom_index+1].hgt);
            GT.updateGPSpose(lla); //get GPS odom;
            next_pose = GT.gps_pos_;
            curr_odom.pose.pose.position.x = curr_pose(0) + (next_pose(0) - curr_pose(0))*(i/10.0-odom_index);
            curr_odom.pose.pose.position.y = curr_pose(1) + (next_pose(1) - curr_pose(1))*(i/10.0-odom_index);
            curr_odom.pose.pose.position.z = curr_pose(2) + (next_pose(2) - curr_pose(2))*(i/10.0-odom_index);
            curr_odom.pose.covariance[0] = oem7720Odom[odom_index].lon_stdev;
            curr_odom.pose.covariance[7] = oem7720Odom[odom_index].lat_stdev;
            curr_odom.pose.covariance[14] = oem7720Odom[odom_index].hgt_stdev;
           // std::cout<<"a: "<<curr_pose(0)<<" b: "<<next_pose(0)<<" c: "<<(next_pose(2) - curr_pose(2))*(i/10-odom_index)<<" d "<<i/10.0-odom_index<<std::endl;
        }else{
            std::cout<<"exceed: "<<odom_index+1<<" should smaller than: "<<oem7720Odom.size()<<" curr imu index: "<<i<<std::endl;
            curr_odom.pose.pose.position.x = curr_pose(0)  ;
            curr_odom.pose.pose.position.y = curr_pose(1)  ;
            curr_odom.pose.pose.position.z = curr_pose(2)  ;
            curr_odom.pose.covariance[0] = oem7720Odom[odom_index].lon_stdev;
            curr_odom.pose.covariance[7] = oem7720Odom[odom_index].lat_stdev;
            curr_odom.pose.covariance[14] = oem7720Odom[odom_index].hgt_stdev;
        }
        bag.write("/odom_100hz", oem7720Imu[i].header.stamp, curr_odom);
        time_delay_fix_table.push_back(oem7720Imu[i].header.stamp.toSec());
        odom_buffer.push_back(curr_odom);
        Time_buffer.push_back(oem7720Imu[i].header.stamp);
    }
    std::cout<<" origin: "<<GT.lla_origin_.transpose()<<std::endl;
    //18000 hz
/*
    for (int i = 0; i < odom_buffer.size()-1; ++i) {
        nav_msgs::Odometry curr_odom = odom_buffer[i];
        curr_odom.header.frame_id = "novatel_imu";
        Eigen::Quaterniond q0,q1;
        q0.x() = odom_buffer[i].pose.pose.orientation.x;
        q0.y() = odom_buffer[i].pose.pose.orientation.y;
        q0.z() = odom_buffer[i].pose.pose.orientation.z;
        q0.w() = odom_buffer[i].pose.pose.orientation.w;
        q1.x() = odom_buffer[i+1].pose.pose.orientation.x;
        q1.y() = odom_buffer[i+1].pose.pose.orientation.y;
        q1.z() = odom_buffer[i+1].pose.pose.orientation.z;
        q1.w() = odom_buffer[i+1].pose.pose.orientation.w;
        Eigen::Vector3d trans1 = Eigen::Vector3d(odom_buffer[i].pose.pose.position.x,
                                                 odom_buffer[i].pose.pose.position.y,odom_buffer[i].pose.pose.position.z);
        Eigen::Vector3d trans2 = Eigen::Vector3d(odom_buffer[i+1].pose.pose.position.x,
                                                 odom_buffer[i+1].pose.pose.position.y,odom_buffer[i+1].pose.pose.position.z);

        for (int j = 0; j < 180; ++j) {
            ros::Time curr_pose_time = ros::Time(time_start + i * 0.01 + j * 0.01/180);
            curr_odom.header.stamp = curr_pose_time;
            curr_odom.header.seq = i*180 + j;
            Eigen::Vector3d result;
            Eigen::Quaterniond q_result;
            result  = (1.0 - j/180.0) * trans2 + j/180.0 * trans1;
            q_result = q0.slerp(j/180.0, q1);
            curr_odom.pose.pose.position.x = result.x();
            curr_odom.pose.pose.position.y = result.y();
            curr_odom.pose.pose.position.z = result.z();
            curr_odom.pose.pose.orientation.x = q_result.x();
            curr_odom.pose.pose.orientation.y = q_result.y();
            curr_odom.pose.pose.orientation.z = q_result.z();
            curr_odom.pose.pose.orientation.w = q_result.w();
            //bag.write("/odom_18000hz", curr_odom.header.stamp, curr_odom);
            q_buffer.push_back(q_result);
            t_buffer.push_back(result);
            Eigen::Isometry3d odom;
            odom.setIdentity();
            odom.rotate(q_result);
            odom.translation() = result;
        }
    }*/
    bag.close();
    std::cout<<" q_buffer: "<<q_buffer.size()<<" odom_buffer: "<<odom_buffer.size()<<std::endl;
}

void novatel_mapping::transformPointCloud() {

    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");
    ros::Publisher test_frame;
    test_frame = node.advertise<sensor_msgs::PointCloud2>("/local_map", 5);


    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    float time_last = 0;
    topics.emplace_back("/velodyne_points");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int inter_times =0;
    double time;
    pcl::PointCloud<pcl::PointXYZI> vlp_global;
    pcl::PointCloud<pcl::PointXYZI> vlp_global_unDownSample;
    pcl::PointXYZI vlp_global_point;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_unfiltered (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::VoxelGrid<pcl::PointXYZI> VoxelFilter;
    VoxelFilter.setLeafSize (0.05f, 0.05f, 0.05f);
    rosbag::Bag bagwrite;
    bagwrite.open("/home/echo/bag/7720_Lidar/gt.bag", rosbag::bagmode::Write);
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
                {
                    std::stringstream pcd_save;
                    std::stringstream pcd_save_raw;
                    std::stringstream pcd_save_icp;
                    pcl::PointCloud<VLPPoint> vlp_pcd;
                    pcl::PointCloud<VLPPoint> vlp_pcd_save;
                    VLPPointCloud raw_pcd;
                    pcd_save.precision(18);
                    sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
                    pcl::fromROSMsg(*s,vlp_pcd);
                    time = s->header.stamp.toSec();
                    Eigen::Vector3d result;
                    Eigen::Quaterniond q_result;
                    int index = 0;
                    int scanindex =0;
                    transformPoint(vlp_pcd,raw_pcd,index, vlp_pcd_save, time,vlp_global_point,vlp_global_unDownSample,scan_global);
                    pcl::PCLPointCloud2 pcl_frame;
                    sensor_msgs::PointCloud2 LocalMapPC2;
                    LocalMapPC2 = *s;
                    pcl::toPCLPointCloud2(vlp_pcd_save, pcl_frame);
                    pcl_conversions::fromPCL(pcl_frame, LocalMapPC2);
                    LocalMapPC2.header.frame_id = "map";
                    time_last += vlp_pcd[vlp_pcd.size()-1].time - vlp_pcd[0].time;
                    pcd_save<<save_path.c_str()<<"/tfed/"<<inter_times<<".pcd";
                    pcd_save_raw<<save_path.c_str()<<"/raw/"<<inter_times<<".pcd";
                    pcd_save_icp<<save_path.c_str()<<"/icp/"<<inter_times<<".pcd";
                    if(!vlp_pcd_save.empty()){
                        writer.write(pcd_save.str(),vlp_pcd_save,true);
                        writer.write(pcd_save_raw.str(),raw_pcd,true);
                        if(scan_global.size()!=0){
                            writer.write(pcd_save_icp.str(),scan_global,true);
                        }
                        std::cout<<"time sync diff: "<<index/100.0 + time_start - time<<" time_last "<<time_last<<" pcd_save:"<<pcd_save.str()<<" size: "<<vlp_pcd.size()<<" times: "<<inter_times<<std::endl;;//100hz to sec
                        sensor_msgs::PointCloud2 to_pub_frame_linear;
                        pcl::PCLPointCloud2 pcl_frame;
                        pcl::toPCLPointCloud2(vlp_pcd_save, pcl_frame);
                        pcl_conversions::fromPCL(pcl_frame, to_pub_frame_linear);
                        to_pub_frame_linear.header.frame_id = "map";
                        test_frame.publish(to_pub_frame_linear);
                        //ds [ 0.0006981, -0.0008727, 0.0004363, 0.9999993 ]
                        if((inter_times/1000.0 - floor(inter_times/1000.0)) == 0){
                            *cloud_unfiltered = vlp_global_unDownSample;
                            VoxelFilter.setInputCloud(cloud_unfiltered);
                            VoxelFilter.filter (*cloud_filtered);
                            vlp_global_unDownSample = *cloud_filtered;
                            writer.write(global_path+"/"+std::to_string(floor(inter_times/1000))+"_global.pcd",vlp_global_unDownSample,true);
                            vlp_global = vlp_global_unDownSample + vlp_global;
                            vlp_global_unDownSample.clear();
                        }
                        scanindex = floor(( time - time_start)*100.0);
                        if(scanindex >=0 && scanindex<odom_buffer.size()-1) {
                            scanindex = floor(-(time_delay_fix_table[scanindex] - ( time)) * 100.0) + index;
                        }
                        odom_buffer[scanindex].header.stamp = s->header.stamp;
                        bagwrite.write("/icp_error",s->header.stamp,icp_error);
                        bagwrite.write("/gt", s->header.stamp, odom_buffer[scanindex]);
                        bagwrite.write("/gt_scan",s->header.stamp,LocalMapPC2);
                    }
                    inter_times++;
                }

    ofs.open(las_path+"/global.las", std::ios::out | std::ios::binary);
    header.SetPointRecordsCount(vlp_global.points.size());
    header.SetPointRecordsByReturnCount(0,vlp_global.points.size());
    header.SetMax(maxPt[0]+1, maxPt[1]+1, maxPt[2]+1);
    header.SetMin(minPt[0]-1, minPt[1]-1, minPt[2]-1);
    header.SetDataFormatId(liblas::ePointFormat1);
    header.SetScale(0.01,0.01,0.01);
    srs.SetFromUserInput("EPSG:5180");
    header.SetSRS(srs);
    liblas::Writer writer_las(ofs, header);
    liblas::Point point(&header);

    for(int j=0; j<vlp_global.size(); j++) {
        point.SetCoordinates(vlp_global.points[j].x, vlp_global.points[j].y, vlp_global.points[j].z);
        point.SetIntensity(std::min(vlp_global.points[j].intensity,float(80))); //set max is 80
        writer_las.WritePoint(point);
    }

    writer.write(global_path+"/last_global.pcd",vlp_global,true);
}

void novatel_mapping::transformPoint(pcl::PointCloud<VLPPoint> vlp_pcd,VLPPointCloud &raw_pcd,
                                     int &index,pcl::PointCloud<VLPPoint> &vlp_pcd_save,  double time,
                                     pcl::PointXYZI &vlp_global_point,pcl::PointCloud<pcl::PointXYZI> &vlp_global_unDownSample,
                                     pcl::PointCloud<pcl::PointXYZI> &scan_global) {
    int index_stamp;
    Eigen::Quaterniond q_stamp;
    Eigen::Quaterniond q_offset;
    q_offset = Eigen::Quaterniond (0.9999993,0.0006981, -0.0008727, 0.0004363);
    pcl::PointCloud<pcl::PointXYZI> scan_local;

    index_stamp = floor(-(time_delay_fix_table[index] - time + time_start-time_start) * 100.0) + index;
    if(index_stamp<=0){index_stamp=0;}

    q_stamp.x() = odom_buffer[index_stamp].pose.pose.orientation.x;
    q_stamp.y() = odom_buffer[index_stamp].pose.pose.orientation.y;
    q_stamp.z() = odom_buffer[index_stamp].pose.pose.orientation.z;
    q_stamp.w() = odom_buffer[index_stamp].pose.pose.orientation.w;
    Eigen::Vector3d t_stamp(odom_buffer[index_stamp].pose.pose.position.x,odom_buffer[index_stamp].pose.pose.position.y,
                        odom_buffer[index_stamp].pose.pose.position.z);
    Eigen::Affine3d T_stamp;
    T_stamp.setIdentity();
    T_stamp.rotate(q_stamp);
    T_stamp.translation() = t_stamp;

    for (int i = 0; i < vlp_pcd.size(); i = i+5) {
        VLPPoint temp;
        temp.x = vlp_pcd[i].x;
        temp.y = vlp_pcd[i].y;
        temp.z = vlp_pcd[i].z;
        temp.intensity = vlp_pcd[i].intensity;
        temp.time = vlp_pcd[i].time + time - time_start;
        temp.ring = vlp_pcd[i].ring;

        index = floor((temp.time)*100.0);
        if(index >=0 && index<odom_buffer.size()-1) {
            index = floor(-(time_delay_fix_table[index] - (temp.time + time_start)) * 100.0) + index;
        }
        if(index >=0 && index<odom_buffer.size()-1){
            Eigen::Quaterniond q;
            q.x() = odom_buffer[index].pose.pose.orientation.x;
            q.y() = odom_buffer[index].pose.pose.orientation.y;
            q.z() = odom_buffer[index].pose.pose.orientation.z;
            q.w() = odom_buffer[index].pose.pose.orientation.w;
            Eigen::Vector3d t(odom_buffer[index].pose.pose.position.x - 0.0,
                              odom_buffer[index].pose.pose.position.y - 0.0,
                              odom_buffer[index].pose.pose.position.z + 0.0);
            Eigen::Vector3d point(temp.x,temp.y,temp.z);
            point =  q_offset.matrix()*q.matrix() * point + t;
            vlp_global_point.x = temp.x;
            vlp_global_point.y = temp.y;
            vlp_global_point.z = temp.z;
            vlp_global_point.intensity = temp.intensity;
            temp.x = point.x();
            temp.y = point.y();
            temp.z = point.z();
            if (sqrt(vlp_pcd[i].x * vlp_pcd[i].x + vlp_pcd[i].y * vlp_pcd[i].y + vlp_pcd[i].z * vlp_pcd[i].z) < 40) {
                vlp_pcd_save.push_back(temp);
                vlp_global_unDownSample.push_back(vlp_global_point);
                local_source.push_back(vlp_global_point);
            }
            maxPt[0] = std::max(maxPt[0],temp.x);
            maxPt[1] = std::max(maxPt[1],temp.y);
            maxPt[2] = std::max(maxPt[2],temp.z);
            minPt[0] = std::min(minPt[0],temp.x);
            minPt[1] = std::min(minPt[1],temp.y);
            minPt[2] = std::min(minPt[2],temp.z);
            point =   q_stamp.inverse().matrix()* (point - t_stamp);
            temp.x = point.x();
            temp.y = point.y();
            temp.z = point.z();
            vlp_global_point.x = temp.x;
            vlp_global_point.y = temp.y;
            vlp_global_point.z = temp.z;
            raw_pcd.push_back(temp);
            scan_local.push_back(vlp_global_point);
        }
    }
    if (local_map.size()>0){
        pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_local_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcd_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        *raw_pcd_ptr = scan_local;
        pcl::transformPointCloud(local_map,*local_map_local_ptr,T_stamp.inverse().matrix());
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setMaxCorrespondenceDistance(0.5);
        icp.setMaximumIterations (50);
        icp.setTransformationEpsilon(0.0001);
        icp.setInputSource (raw_pcd_ptr);
        icp.setInputTarget (local_map_local_ptr);
        icp.align (*raw_pcd_ptr);
        pcl::transformPointCloud(*raw_pcd_ptr,scan_global,T_stamp.matrix());
        icp_result = icp.getFinalTransformation().cast<double>();
        Eigen::Quaterniond qq = Eigen::Quaterniond(icp_result.rotation());
        icp_error.pose.pose.position.x = icp_result.translation().x();
        icp_error.pose.pose.position.y = icp_result.translation().y();
        icp_error.pose.pose.position.z = icp_result.translation().z();
        icp_error.pose.pose.orientation.x = qq.x();
        icp_error.pose.pose.orientation.y = qq.y();
        icp_error.pose.pose.orientation.z = qq.z();
        icp_error.pose.pose.orientation.w = qq.w();
        std::cout<<icp.getFinalTransformation () <<" size: "<<scan_global.size()<< std::endl;
    }
    for (int j = 0; j < vlp_pcd_save.size(); ++j) {
        pcl::PointXYZI temp;
        temp.x = vlp_pcd_save[j].x;
        temp.y = vlp_pcd_save[j].y;
        temp.z = vlp_pcd_save[j].z;
        local_map.push_back(temp);
    }
    if(local_map.size()>50*vlp_pcd_save.size()){
        pcl::PointCloud<pcl::PointXYZI> local;
        for (int j = local_map.size() - 50*vlp_pcd_save.size(); j < local_map.size(); ++j) {
            local.push_back(local_map[j]);
        }
        local_map = local;
    }
}
