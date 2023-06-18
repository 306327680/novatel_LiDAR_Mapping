//
// Created by echo on 2019/11/26.
//

#include "ReadBag.h"

//todo 这里的lidar的时间戳有问题,应该还是固定的时间
void ReadBag::getPath(std::string path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	
	//可以加挺多topic的?
	topics.push_back(std::string(lidarodom));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	//读广场的也就几秒
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
		if (bag_strat_time.isZero()){
			bag_strat_time = s->header.stamp;
		}
		if(fabs(s->pose.pose.position.x)<10000) {
			if (s != NULL) {
				pcl::PointXYZINormal temp;
				temp.x = s->pose.pose.position.x;
				temp.y = s->pose.pose.position.y;
				temp.z = s->pose.pose.position.z;
				temp.intensity = s->header.stamp.toSec()-bag_strat_time.toSec();
				Eigen_encoder.emplace_back(s->pose.pose.position.x, s->pose.pose.position.y, s->pose.pose.position.z);
				encoder_pcd.push_back(temp);
			}
		}
	}
	topics.clear();
	topics.push_back(std::string(gps_calibrate));
	rosbag::View view1(bag, rosbag::TopicQuery(topics));
	bag_strat_time.init();
	BOOST_FOREACH(rosbag::MessageInstance const m, view1) {
					nav_msgs::Odometry::ConstPtr gps = m.instantiate<nav_msgs::Odometry>();
					
			if (bag_strat_time.isZero()){
						bag_strat_time = gps->header.stamp;
			}
			if (gps != NULL) {
				if(fabs(gps->pose.pose.position.x)<10000){
					pcl::PointXYZINormal temp;
					temp.x = gps->pose.pose.position.x;
					temp.y = gps->pose.pose.position.y;
					temp.z = gps->pose.pose.position.z;
					temp.intensity = gps->header.stamp.toSec() - bag_strat_time.toSec();
					Eigen_GPS.emplace_back(gps->pose.pose.position.x, gps->pose.pose.position.y,gps->pose.pose.position.z);
					gps_pcd.push_back(temp);
				}
			}
	}

bag.close();
	std::cout<<"read bag finish! "<<Eigen_GPS.size()<<" "<<Eigen_encoder.size()<<std::endl;
}
/*
 *
header:
  seq: 11902
  stamp:
    secs: 1574931531
    nsecs: 757029056
  frame_id: "cpt"
status:
  status: 0
  service: 1
latitude: 22.5336322945
longitude: 113.945080181
altitude: 8.4084
position_covariance: [0.045125999318016086, 0.09610083080179557, 0.3187239400720881, 0.09610083080179557, 0.20465739973337754, 0.6787580530127076, 0.3187239400720881, 0.6787580530127076, 2.2511401744076007]
position_covariance_type: 0
 经纬度
 * */
void ReadBag::readCPT(sensor_msgs::NavSatFix input) {

}

void ReadBag::readHesai(std::string path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	bool pcd_start = false;
	double time_begin;
 	float maxintensity=0;
	float minintensity=1000;
	float setMinIntensity = 0.0000000000000000000000000000000000000000014349296274686126806258990933;
	float setMaxIntensity = 0.0000000000000000000000000000000000000000017922607358714410337114501370;
	float diff_intensity = setMaxIntensity-setMinIntensity;
	double timestamp;
	//可以加挺多topic的?
	topics.push_back(std::string("/points_raw"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	//读广场的也就几秒
	int inter_times =0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					pcdtosave.clear();
					std::stringstream pcd_save;
					pcd_save.precision(18);
					sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
					pcl::PCLPointCloud2 pcl_pc2;
			
		
					pcl::fromROSMsg(*s,hesai_pcd);
 
 					if(!pcd_start){
						time_begin = hesai_pcd[0].timestamp;
 					}
		
					for (int i = 0; i < hesai_pcd.size(); ++i) {
						//std::printf("%f\n",hesai_pcd[i].timestamp);
						mypcd temp;
						temp.x = hesai_pcd[i].x;
						temp.y = hesai_pcd[i].y;
						temp.z = hesai_pcd[i].z;
						temp.intensity = int(256*(hesai_pcd[i].intensity - setMinIntensity)/diff_intensity);
						temp.timestamp = hesai_pcd[i].timestamp - time_begin;
						temp.ring = hesai_pcd[i].ring;
						pcdtosave.push_back(temp);
						//用来找最大最小intensity的
			/*			if(maxintensity<hesai_pcd[i].intensity){
							maxintensity=hesai_pcd[i].intensity;
						}
						if(minintensity>hesai_pcd[i].intensity){
							minintensity=hesai_pcd[i].intensity;
						}*/
					}
					pcdtosave.width = pcdtosave.size();
					//用来找最大最小intensity的
/*					std::printf("maxintensity: %f minintensity: %f\n",maxintensity,minintensity);
					std::cout.precision(70);
					std::cout<<"maxintensity: "<<std::fixed<<maxintensity<<" minintensity: "<<minintensity<<std::endl;*/
					timestamp = s->header.stamp.toSec();
					pcd_save<<"map_pm/"<<timestamp<<".pcd";
					std::cout<<pcd_save.str()<<" size: "<<pcdtosave.size()<<" times: "<<inter_times<<std::endl;
					writer.write(pcd_save.str(),pcdtosave,true);
					std::cout<<pcd_save.str()<<" saved " <<std::endl;
					inter_times++;
				}
}

void ReadBag::gnssLiDARExtrinsicParameters(std::string path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	
	//可以加挺多topic的?
	topics.push_back(std::string(lidarodom));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	Eigen::Isometry3d EP ;
	//读广场的也就几秒
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
					if (bag_strat_time.isZero()){
						bag_strat_time = s->header.stamp;
					}
					if(fabs(s->pose.pose.position.x)<10000) {
						if (s != NULL) {
							EP.setIdentity();
							pcl::PointXYZINormal temp;
							Eigen::Quaterniond q_t;
							q_t.x() = s->pose.pose.orientation.x;
							q_t.y() = s->pose.pose.orientation.y;
							q_t.z() = s->pose.pose.orientation.z;
							q_t.w() = s->pose.pose.orientation.w;
							
							EP.pretranslate(Eigen::Vector3d(s->pose.pose.position.x,s->pose.pose.position.y,s->pose.pose.position.z));
							EP.rotate(q_t);
							EP.pretranslate(Eigen::Vector3d(-0.547,0,0));
							std::cout<<EP.matrix()<<std::endl;
							temp.x = EP(0,3);
							temp.y = EP(1,3);
							temp.z = EP(2,3);
							temp.intensity = s->header.stamp.toSec()-bag_strat_time.toSec();
							Eigen_encoder.emplace_back(s->pose.pose.position.x, s->pose.pose.position.y, s->pose.pose.position.z);
							encoder_pcd.push_back(temp);
						}
					}
				}
	topics.clear();
	topics.push_back(std::string(gps_calibrate));
	rosbag::View view1(bag, rosbag::TopicQuery(topics));
	bag_strat_time.init();
	BOOST_FOREACH(rosbag::MessageInstance const m, view1) {
					nav_msgs::Odometry::ConstPtr gps = m.instantiate<nav_msgs::Odometry>();
					
					if (bag_strat_time.isZero()){
						bag_strat_time = gps->header.stamp;
					}
					if (gps != NULL) {
						if(fabs(gps->pose.pose.position.x)<10000){
							pcl::PointXYZINormal temp;
							temp.x = gps->pose.pose.position.x;
							temp.y = gps->pose.pose.position.y;
							temp.z = gps->pose.pose.position.z;
							temp.intensity = gps->header.stamp.toSec() - bag_strat_time.toSec();
							Eigen_GPS.emplace_back(gps->pose.pose.position.x, gps->pose.pose.position.y,gps->pose.pose.position.z);
							gps_pcd.push_back(temp);
						}
					}
				}
 
	bag.close();
	std::cout<<"read bag finish! "<<Eigen_GPS.size()<<" "<<Eigen_encoder.size()<<std::endl;
}
//gps存成lla的pcd
void ReadBag::gnssPCDExtrinsicParameters(std::string path, std::vector<std::pair<Eigen::Isometry3d, double>> &gps_pose,
										 Eigen::Vector3d &lla_origin) {
	bool first_gps = true;
	gpsTools gt;
	std::vector<std::string> topics;
	std::vector<sensor_msgs::NavSatFix> gnss_tosave;//测试转换csv用
	pcl::PointCloud<pcl::PointXYZ>::Ptr gps_route(new pcl::PointCloud<pcl::PointXYZ>);//可视化gps路径
	
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	//topics.push_back(std::string("/cpt/ins_fix"));
	topics.push_back(std::string("/fix"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					sensor_msgs::NavSatFix::ConstPtr s = m.instantiate<sensor_msgs::NavSatFix>();
					gnss_tosave.push_back(*s);
					if(first_gps){
						//把第一个坐标转化位lla
						lla_origin = gt.GpsMsg2Eigen(*s);
						gps_pose.push_back(std::pair<Eigen::Isometry3d, double>());
						gt.lla_origin_ = lla_origin;
						gps_route->push_back(pcl::PointXYZ(0,0,0));
						first_gps = false;
					}else{
						gt.updateGPSpose(*s);
						gps_route->push_back(pcl::PointXYZ(gt.gps_pos_(0),gt.gps_pos_(1),gt.gps_pos_(2)));
					}
				}
	pcl::PCDWriter writer;
	std::cout<<"saving the data"<<std::endl;
//	csvio.NavSat2CSV(gnss_tosave,"",gnss_tosave[0].header.stamp);
	csvio.NavSat2CSVLLA(gnss_tosave,"",gnss_tosave[0].header.stamp,gnss_tosave[0]);
	writer.write("route/gnss.pcd",*gps_route);
}
//转换vlp16 到建图的数据格式
void ReadBag::readVLP16(std::string path,std::string save_path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	rosbag::Bag bag;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	bool pcd_start = false;
	float time_last = 0;
	float timestamp;
	//可以加挺多topic的?
	topics.push_back(std::string("/velodyne_points"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	//读广场的也就几秒
	int inter_times =0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					pcdtosave.clear();
					std::stringstream pcd_save;
					pcd_save.precision(18);
					sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
					pcl::PCLPointCloud2 pcl_pc2;
					pcl::fromROSMsg(*s,vlp_pcd);
			 
					if(!pcd_start){
						lidar_first = s->header.stamp;
						pcd_start = true;
					}
					
					for (int i = 0; i < vlp_pcd.size(); ++i) {
						//std::printf("%f\n",hesai_pcd[i].timestamp);
						VLPPoint temp;
						temp.x = vlp_pcd[i].x;
						temp.y = vlp_pcd[i].y;
						temp.z = vlp_pcd[i].z;
						temp.intensity = vlp_pcd[i].intensity;
						temp.time = vlp_pcd[i].time + time_last;
						temp.ring = vlp_pcd[i].ring;
						vlp_pcdtosave.push_back(temp);
					}

					time_last += vlp_pcd[vlp_pcd.size()-1].time - vlp_pcd[0].time;
					std::cout<<time_last<<std::endl;
					vlp_pcdtosave.width = vlp_pcdtosave.size();
					
					pcd_save<<save_path.c_str()<<"/"<<inter_times<<".pcd";
					std::cout<<pcd_save.str()<<" size: "<<vlp_pcd.size()<<" times: "<<inter_times<<std::endl;
					writer.write(pcd_save.str(),vlp_pcdtosave,true);
					vlp_pcdtosave.clear();
			 
					inter_times++;
				}
}
void ReadBag::readVLP16WoTime(std::string path, std::string save_path) {
//	mergePC mPc;
//	mPc.readYaml("/home/echo/jjh/config_zhubr.yaml");
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	bool pcd_start = false;
	int beam_size ;
	double timestamp;
	//可以加挺多topic的?
	topics.push_back(std::string("/top/rslidar_points"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	float time_last = 0;
	std::vector<int> indices1;
	std::vector<VLPPointCloud> left_all;
	std::vector<VLPPointCloud> right_all;
	//读广场的也就几秒
	int inter_times =0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					pcdtosave.clear();
					std::stringstream pcd_save;
					pcd_save.precision(18);
					sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
					pcl::PCLPointCloud2 pcl_pc2;
					pcl::fromROSMsg(*s,vlp_pcd);
		 
					for (int i = 0; i < vlp_pcd.size(); ++i) {
						VLPPoint temp;
						temp.x = vlp_pcd[i].x;
						temp.y = vlp_pcd[i].y;
						temp.z = vlp_pcd[i].z;
						temp.intensity = vlp_pcd[i].intensity;
						temp.time = i*0.1/vlp_pcd.size() +time_last;
						temp.ring = vlp_pcd[i].ring;
						vlp_pcdtosave.push_back(temp);
				 
					}
					
					time_last += 0.1;
					std::cout<<time_last<<std::endl;
					vlp_pcdtosave.width = vlp_pcdtosave.size();
					
					pcd_save<<save_path.c_str()<<"/"<<inter_times<<".pcd";
					std::cout<<pcd_save.str()<<" size: "<<vlp_pcd.size()<<" times: "<<inter_times<<std::endl;
					left_all.push_back(vlp_pcdtosave);
					writer.write(pcd_save.str(),vlp_pcdtosave,true);
					vlp_pcdtosave.clear();
					inter_times++;
				}
				//再加一个topic
	topics.clear();
	topics.push_back("/right/velodyne_points");
	
	rosbag::View view1(bag, rosbag::TopicQuery(topics));
	float time_last_r = 0;
	int inter_times_r =0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view1)
				{
					pcdtosave.clear();
					std::stringstream pcd_save;
					pcd_save.precision(18);
					sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
					pcl::PCLPointCloud2 pcl_pc2;
					pcl::fromROSMsg(*s,vlp_pcd);
					
					for (int i = 0; i < vlp_pcd.size(); ++i) {
						VLPPoint temp;
						temp.x = vlp_pcd[i].x;
						temp.y = vlp_pcd[i].y;
						temp.z = vlp_pcd[i].z;
						temp.intensity = vlp_pcd[i].intensity;
						temp.time = i*0.1/vlp_pcd.size() +time_last_r;
						temp.ring = vlp_pcd[i].ring;
						vlp_pcdtosave.push_back(temp);
						
					}
					
					time_last_r += 0.1;
					std::cout<<time_last_r<<std::endl;
					vlp_pcdtosave.width = vlp_pcdtosave.size();
					
					pcd_save<<save_path.c_str()<<"/"<<inter_times_r<<".pcd";
					std::cout<<pcd_save.str()<<" size: "<<vlp_pcd.size()<<" times: "<<inter_times_r<<std::endl;
					right_all.push_back(vlp_pcdtosave);
//					writer.write(pcd_save.str(),vlp_pcdtosave,true);
					vlp_pcdtosave.clear();
					inter_times_r++;
				}
	std::cout<<"right_all:   "<<right_all.size()<<"  left_all  "<<left_all.size()<<std::endl;
	int merge_id =0;
	for (int j = 0; j < right_all.size(); ++j) {
		std::stringstream pcd_save;
		pcd_save<<save_path.c_str()<<"/"<<merge_id<<".pcd";
		std::cout<<pcd_save.str()<<" size: "<<vlp_pcd.size()<<" times: "<<merge_id<<std::endl;
//		writer.write(pcd_save.str(),mPc.process(left_all[j],right_all[j]),true);
		merge_id ++;
	}
}
//读取robosense的雷达
void ReadBag::readTopRobosense(std::string path, std::string save_path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	bool pcd_start = false;
    int beam_size ;
	double timestamp;
	//可以加挺多topic的?
	topics.push_back(std::string("/top/rslidar_points"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
	std::vector<int> indices1;
 
	//读广场的也就几秒
	int inter_times =0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					pcdtosave.clear();
					std::stringstream pcd_save;
					pcd_save.precision(18);
					sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
					pcl::PCLPointCloud2 pcl_pc2;
					pcl::fromROSMsg(*s,robosense_pcd);
	 
					beam_size = robosense_pcd.size()/16;
					for (int i = 0; i < robosense_pcd.size(); ++i) {
						
						RoboPoint temp;
						temp.x = robosense_pcd[i].x;
						temp.y = robosense_pcd[i].y;
						temp.z = robosense_pcd[i].z;
						temp.intensity = robosense_pcd[i].intensity;
						temp.time =  (i%beam_size)*0.1/beam_size;
						if(floor(16*i/robosense_pcd.size())>7){
							temp.ring =  23-floor(16*i/robosense_pcd.size());
						}else{
							temp.ring =  floor(16*i/robosense_pcd.size());
						}
						if(!__isnan(temp.x)){
							robo_pcdtosave.push_back(temp);
						}
					}
					robo_pcdtosave.width = robo_pcdtosave.size();
					RoboPointCLoud robo_pcdtosave_temp;
					//pcl::removeNaNFromPointCloud(robo_pcdtosave, robo_pcdtosave_temp, indices1);
			
					//用来找最大最小intensity的
					int nsec_c[9];
					
					nsec_c[8] = s->header.stamp.nsec%10;
					nsec_c[7] = (s->header.stamp.nsec/10)%10;
					nsec_c[6] = (s->header.stamp.nsec/100)%10;
					nsec_c[5] = (s->header.stamp.nsec/1000)%10;
					nsec_c[4] = (s->header.stamp.nsec/10000)%10;
					nsec_c[3] = (s->header.stamp.nsec/100000)%10;
					nsec_c[2] = (s->header.stamp.nsec/1000000)%10;
					nsec_c[1] = (s->header.stamp.nsec/10000000)%10;
					nsec_c[0] = (s->header.stamp.nsec/100000000)%10;
					
					std::cout<<nsec_c[0]<<nsec_c[1]<<nsec_c[2]<<nsec_c[3]<<nsec_c[4]<<nsec_c[5]
							 <<nsec_c[6]<<nsec_c[7]<<nsec_c[8]<<" "<<s->header.stamp.nsec<<std::endl;
					timestamp = s->header.stamp.sec;
					pcd_save<<save_path.c_str()<<"/"<<timestamp<<"."<<nsec_c[0]<<nsec_c[1]<<nsec_c[2]<<nsec_c[3]
							<<nsec_c[4]<<nsec_c[5]<<nsec_c[6]<<nsec_c[7]<<nsec_c[8]<<".pcd";
 
					std::cout<<pcd_save.str()<<" size: "<<robo_pcdtosave_temp.size()<<" times: "<<inter_times<<std::endl;
					writer.write(pcd_save.str(),robo_pcdtosave_temp, true);
					robo_pcdtosave.clear();
					std::cout<<pcd_save.str()<<" saved " <<std::endl;
					inter_times++;
				}
}

void ReadBag::readStereoCamera(std::string path, std::string save_path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	rosbag::Bag bag;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	
	double timestamp;
	//可以加挺多topic的?

    topics.push_back(std::string("/stereo/frame_left/image_raw"));
//	topics.push_back(std::string("/stereo/left/image_color/compressed"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					//sensor_msgs::CompressedImage::ConstPtr s = m.instantiate<sensor_msgs::CompressedImage>();
					sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
					cv_bridge::CvImagePtr cv_cam = cv_bridge::toCvCopy(s, sensor_msgs::image_encodings::BGR8);
					cv::Mat src = cv_cam->image;
					//用来找最大最小intensity的
					std::stringstream ss;
					ss.setf(std::ios::fixed);
					ss<<setprecision(9)<<save_path.c_str()<<"/left"<<"/"<<s->header.stamp.toSec() <<".png";
					std::cout<<ss.str()<<std::endl;
					cv::imwrite(ss.str(),src);
				}
	topics.clear();

//    topics.push_back(std::string("/stereo/right/image_color/compressed"));
	topics.push_back(std::string("/stereo/frame_right/image_raw"));
	rosbag::View view1(bag, rosbag::TopicQuery(topics));
	
	BOOST_FOREACH(rosbag::MessageInstance const m, view1)
				{
                    //sensor_msgs::CompressedImage::ConstPtr s = m.instantiate<sensor_msgs::CompressedImage>();
                    sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
					cv_bridge::CvImagePtr cv_cam = cv_bridge::toCvCopy(s, sensor_msgs::image_encodings::BGR8);
					cv::Mat src = cv_cam->image;
					//用来找最大最小intensity的
					std::stringstream ss;
					ss.setf(std::ios::fixed);
					ss<<setprecision(9)<<save_path.c_str()<<"/right"<<"/"<<s->header.stamp.toSec() <<".png";
					std::cout<<ss.str()<<std::endl;
					cv::imwrite(ss.str(),src);
				}
}
void ReadBag::readcamera(std::string path, std::string save_path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	rosbag::Bag bag;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
 
	double timestamp;
	//可以加挺多topic的?
	topics.push_back(std::string("/stereo/right/image_color/compressed"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
				 
					sensor_msgs::CompressedImage::ConstPtr s = m.instantiate<sensor_msgs::CompressedImage>();
					cv_bridge::CvImagePtr cv_cam = cv_bridge::toCvCopy(s, sensor_msgs::image_encodings::BGR8);
					cv::Mat src = cv_cam->image;
					//用来找最大最小intensity的
					std::stringstream ss;
					ss.setf(std::ios::fixed);
					ss<<setprecision(9)<<save_path.c_str()<<"/"<<s->header.stamp.toSec() <<".png";
					std::cout<<ss.str()<<std::endl;
					cv::imwrite(ss.str(),src);
				}
}

void ReadBag::readCalibratedCamera(std::string path,std::string cali_path, std::string save_path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	
	double timestamp;
	//可以加挺多topic的?
	topics.push_back(std::string("/usb_cam/image_raw/compressed"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					
					sensor_msgs::CompressedImage::ConstPtr s = m.instantiate<sensor_msgs::CompressedImage>();
					cv_bridge::CvImagePtr cv_cam = cv_bridge::toCvCopy(s, sensor_msgs::image_encodings::BGR8);
					cv::Mat src = cv_cam->image;
					//用来找最大最小intensity的
					std::stringstream ss;
					ss.setf(std::ios::fixed);
					ss<<setprecision(9)<<save_path.c_str()<<"/"<<s->header.stamp.toSec() <<".png";
					std::cout<<ss.str()<<std::endl;
					cv::imwrite(ss.str(),src);
				}
}

void ReadBag::saveRTK2PCD(std::string path,std::string savepath) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	rosbag::Bag bag;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	std::vector<sensor_msgs::NavSatFix> gnss_tosave;//测试转换csv用
	pcl::PointCloud<pcl::PointXYZI>::Ptr gps_route(new pcl::PointCloud<pcl::PointXYZI>);//可视化gps路径
	topics.push_back(std::string("/fix"));
	gpsTools gt;
	rosbag::View view1(bag, rosbag::TopicQuery(topics));
	bool first_gps=true;
	Eigen::Vector3d lla_origin;
	bag_strat_time.init();
	BOOST_FOREACH(rosbag::MessageInstance const m, view1)
				{
					sensor_msgs::NavSatFix::ConstPtr s = m.instantiate<sensor_msgs::NavSatFix>();
					gnss_tosave.push_back(*s);
					pcl::PointXYZI onepoint;
					if(first_gps){
						//把第一个坐标转化位lla
						lla_origin = gt.GpsMsg2Eigen(*s);
						gt.lla_origin_ = lla_origin;
						gps_route->push_back(onepoint);
						first_gps = false;
					}else{
						gt.updateGPSpose(*s);
						onepoint.x = gt.gps_pos_(0);
						onepoint.y = gt.gps_pos_(1);
						onepoint.z = gt.gps_pos_(2);
						onepoint.intensity = s->status.status;
						gps_route->push_back(onepoint);
					}
				}
	pcl::PCDWriter writer;
	std::cout<<"saving the data"<<std::endl;
 //第一帧雷达来的时间
	csvio.NavSat2CSVLLA(gnss_tosave,savepath,lidar_first,gnss_tosave[0]);
	writer.write("gps.pcd",*gps_route);
}

void ReadBag::readImu(std::string path, std::string save_path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	rosbag::Bag bag;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	
	//可以加挺多topic的?
	topics.push_back(std::string("/stim300/imu/data_raw"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	std::vector<sensor_msgs::Imu> IMUs;
 
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					sensor_msgs::Imu::ConstPtr s = m.instantiate<sensor_msgs::Imu>();
					IMUs.push_back(*s);
				}
                std::cout<<"imu number: "<<IMUs.size()<<std::endl;
	if (lidar_first.isZero()){
		lidar_first = IMUs[0].header.stamp;
	}
    std::cout<<"save_path: "<<save_path<<std::endl;
	csvio.IMU2CSV(IMUs,save_path,lidar_first);
}

void ReadBag::readPandarXT32(std::string path, std::string save_path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	bool pcd_start = false;
	double time_begin;
 
	double timestamp;
	//可以加挺多topic的?
	topics.push_back(std::string("/pandar"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	//读广场的也就几秒
	int inter_times =0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					pcdtosave.clear();
					std::stringstream pcd_save;
					pcd_save.precision(18);
					sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
					pcl::PCLPointCloud2 pcl_pc2;
					pcl::fromROSMsg(*s,hesai_pcd);
					if(!pcd_start){
						time_begin = hesai_pcd[0].timestamp;
					}
					
					for (int i = 0; i < hesai_pcd.size(); ++i) {
						//std::printf("%f\n",hesai_pcd[i].timestamp);
						mypcd temp;
						temp.x = hesai_pcd[i].x;
						temp.y = hesai_pcd[i].y;
						temp.z = hesai_pcd[i].z;
						temp.intensity = hesai_pcd[i].intensity;
						temp.timestamp = hesai_pcd[i].timestamp - time_begin;
						temp.ring = hesai_pcd[i].ring;
						pcdtosave.push_back(temp);
					}
					pcdtosave.width = pcdtosave.size();
					timestamp = s->header.stamp.toSec();
					pcd_save <<save_path.c_str()<<"/"<<inter_times<<".pcd";
					std::cout<<pcd_save.str()<<" size: "<<pcdtosave.size()<<" times: "<<inter_times<<std::endl;
					writer.write(pcd_save.str(),pcdtosave,true);
					std::cout<<pcd_save.str()<<" saved " <<std::endl;
					inter_times++;
				}
}

void ReadBag::readINS(std::string path, std::string save_path) {
	std::cout<<"the bag path is: "<<path<<std::endl;
	bag.open(path, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	bool pcd_start = false;
	double time_begin;
	double timestamp;
	//可以加挺多topic的?
	topics.push_back(std::string("/top/rslidar_points"));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	//读何塞的点云
	
	int inter_times =0;
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
				{
					pcdtosave.clear();
					std::stringstream pcd_save;
					pcd_save.precision(18);
					sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
					pcl::PCLPointCloud2 pcl_pc2;
					pcl::fromROSMsg(*s,hesai_pcd);
					if(!pcd_start){
						time_begin = hesai_pcd[0].timestamp;
					}
					
					for (int i = 0; i < hesai_pcd.size(); ++i) {
						//std::printf("%f\n",hesai_pcd[i].timestamp);
						mypcd temp;
						temp.x = hesai_pcd[i].x;
						temp.y = hesai_pcd[i].y;
						temp.z = hesai_pcd[i].z;
						temp.intensity = hesai_pcd[i].intensity;
						temp.timestamp = hesai_pcd[i].timestamp - time_begin;
						temp.ring = hesai_pcd[i].ring;
						pcdtosave.push_back(temp);
					}
					pcdtosave.width = pcdtosave.size();
					timestamp = s->header.stamp.toSec();
					pcd_save <<save_path.c_str()<<"/"<<inter_times<<".pcd";
					std::cout<<pcd_save.str()<<" size: "<<pcdtosave.size()<<" times: "<<inter_times<<std::endl;
					writer.write(pcd_save.str(),pcdtosave,true);
					std::cout<<pcd_save.str()<<" saved " <<std::endl;
					inter_times++;
				}
	//2. 读取ins

	Eigen::Isometry3d is_test;
	is_test = Eigen::Isometry3d::Identity();
	tf::Transform tf_test;

	topics.clear();
	topics.push_back(std::string("/novatel718d/odometry_gps"));
	rosbag::View view1(bag, rosbag::TopicQuery(topics));
	std::stringstream ss;
	ss.setf(std::ios::fixed);
	ss<<setprecision(9)<<save_path.c_str()<<"/ins/ins.g2o" ;
	std::cout<<ss.str()<<std::endl;
	BOOST_FOREACH(rosbag::MessageInstance const m, view1)
				{
					nav_msgs::Odometry::ConstPtr s = m.instantiate<nav_msgs::Odometry>();
					tf::poseMsgToTF(s->pose.pose, tf_test);
					tf::transformTFToEigen(tf_test, is_test);

				}

}

void ReadBag::readOuster(std::string path, std::string save_path) {
    std::cout<<"the bag path is: "<<path<<std::endl;
    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    bool pcd_start = false;
    float time_last = 0;
    float timestamp;
    //The topic of subscribe
    topics.push_back(std::string("/os_cloud_node/points"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int inter_times =0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
                {
                    double time_range = 0;
                    pcdtosave.clear();
                    std::stringstream pcd_save;
                    pcd_save.precision(18);
                    sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
                    pcl::PCLPointCloud2 pcl_pc2;
                    pcl::fromROSMsg(*s,ouster_read);
                    time_range = ouster_read.size()*10;
                    time_range = ouster_read[ouster_read.size()-1].t - ouster_read[0].t;
                    if(!pcd_start){
                        lidar_first = s->header.stamp;// to imu time ref
                        pcd_start = true;
                    }

                    for (int i = 0; i < ouster_read.size(); ++i) {
                        VLPPoint temp;
                        temp.x = ouster_read[i].x;
                        temp.y = ouster_read[i].y;
                        temp.z = ouster_read[i].z;
                        temp.intensity = ouster_read[i].intensity;
//                        temp.intensity = int((ouster_read[i].reflectivity/65535.0)*256);
                        temp.time = (i%2048)/20480.000;
                        temp.ring = ouster_read[i].ring;
                        ouster_pcdtosave.push_back(temp);
                    }
//                    std::cout<<ouster_read[ouster_read.size()-1].t<< "  ss  "<<ouster_read[0].t<<std::endl;
//                    time_last += ouster_read[ouster_read.size()-1].t - ouster_read[0].t;
//                    std::cout<<time_last<<std::endl;
                    ouster_pcdtosave.width = ouster_pcdtosave.size();

                    pcd_save<<save_path.c_str()<<"/"<<inter_times<<".pcd";
                    std::cout<<pcd_save.str()<<" size: "<<ouster_read.size()<<" times: "<<inter_times<<std::endl;
                    writer.write(pcd_save.str(),ouster_pcdtosave,true);
                    ouster_pcdtosave.clear();
                    inter_times++;
                }
}
void ReadBag::read7720_Odom_Imu(std::string path){

    std::vector<std::string> topics;
    std::cout<<"the bag path is: "<<path<<std::endl;
    bag.open(path, rosbag::bagmode::Read);
    //topics.push_back(std::string("/cpt/ins_fix"));
    topics.push_back(std::string("/novatel/oem7/bestpos"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
                {
                    novatel_oem7_msgs::BESTPOS::ConstPtr s = m.instantiate<novatel_oem7_msgs::BESTPOS>();
                    novatel_oem7_msgs::BESTPOS temp = *s;
                    oem7720Odom.push_back(temp);
                }
    topics.clear();
    std::cout<<"bestpos ok"<<std::endl;
    topics.push_back(std::string("/gps/imu"));
    rosbag::View view1(bag, rosbag::TopicQuery(topics));
    BOOST_FOREACH(rosbag::MessageInstance const m, view1)
                {
                    sensor_msgs::Imu::ConstPtr s = m.instantiate<sensor_msgs::Imu>();
                    sensor_msgs::Imu temp1 ;
                    temp1 = *s;
                    oem7720Imu.push_back(temp1);
                }
    std::cout<<"imu ok"<<std::endl;
}

void ReadBag::writeOdom(std::vector<nav_msgs::Odometry> odom, std::string path) {
    bag.close();
    bag.open(path, rosbag::bagmode::Write);
    for (int i = 0; i < odom.size(); ++i) {
        bag.write("/odom_100hz", odom[i].header.stamp, odom[i]);
    }
    bag.close();
}
