#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <time.h>

using namespace std;

class Laser2PointCloud{
	private:
		ros::NodeHandle n_;
		laser_geometry::LaserProjection projector_;
		tf::TransformListener listener_;

		ros::Publisher pub_laser_debug1_;
		ros::Publisher pub_laser_debug2_;
		ros::Publisher pub_laser_debug3_;

		ros::Publisher pub_pointcloud_;
		ros::Subscriber sub_sq_lidar_;
		// message_filters::Subscriber<sensor_msgs::LaserScan> sub_sq_lidar_;
		// tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
	public:
		Laser2PointCloud();
		void sq_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

Laser2PointCloud::Laser2PointCloud(){
	sub_sq_lidar_ = n_.subscribe<sensor_msgs::LaserScan>("/sq_lidar/scan_raw", 1, &Laser2PointCloud::sq_lidar_callback, this);
	// sub_sq_lidar_.subscribe(n_, "/sq_lidar/scan_raw",1);
	// tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(sub_sq_lidar_, listener_, "/centerlaser_", 1);
	// tf_filter_->registerCallback(boost::bind(&Laser2PointCloud::sq_lidar_callback, this, _1));


	pub_pointcloud_ = n_.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points", 1);

	pub_laser_debug1_ = n_.advertise<sensor_msgs::LaserScan>("/sq_lidar/debug1", 1);
	pub_laser_debug2_ = n_.advertise<sensor_msgs::LaserScan>("/sq_lidar/debug2", 1);
	pub_laser_debug3_ = n_.advertise<sensor_msgs::LaserScan>("/sq_lidar/debug3", 1);

	listener_.setExtrapolationLimit(ros::Duration(0.1));
}

void Laser2PointCloud::sq_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	cout<<"================================================"<<endl;
	sensor_msgs::LaserScan output1;
	output1 = *msg;
	pub_laser_debug1_.publish(output1);
	sensor_msgs::PointCloud2 pc2;
	try{
		string frame_name = output1.header.frame_id;
		// projector_.transformLaserScanToPointCloud(frame_name, output1, pc2, listener_);
		projector_.transformLaserScanToPointCloud("/centerlaser_", output1, pc2, listener_);

		// sensor_msgs::PointCloud pc;
		// sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);

		// sensor_msgs::PointCloud pc_after;
		// listener_.transformPointCloud("/centerlaser_", pc, pc_after);
		// sensor_msgs::PointCloud2 pc2_after;
		// sensor_msgs::convertPointCloudToPointCloud2(pc_after, pc2_after);

		// pub_pointcloud_.publish(pc2_after);
		pub_pointcloud_.publish(pc2);
	}
	catch(tf::TransformException &ex){
		printf ("Failure %s\n", ex.what());
	}

	
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZ>);

	// pcl::fromROSMsg(pc2_after, *pcl_pc);

	// for(size_t i=0; i<pcl_pc->points.size();i++){
		// cout<<"id : "<<i<<endl;
		// cout<<pcl_pc->points[i].x<<endl;
		// cout<<pcl_pc->points[i].y<<endl;
		// cout<<pcl_pc->points[i].z<<endl;
	// }

}

// void Laser2PointCloud::sq_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	// if(msg->header.frame_id == "laser1_link"){
		// sensor_msgs::LaserScan output1;
		// output1 = *msg;
		// cout<<output1.ranges.size()<<endl;
		// pub_laser_debug1_.publish(output1);
		// sensor_msgs::PointCloud2 pc2;
		// string frame_name = output1.header.frame_id;
		// projector_.transformLaserScanToPointCloud(frame_name, output1, pc2, listener_);
		// // projector_.transformLaserScanToPointCloud("/centerlaser_", output1, pc2, listener_);

		// sensor_msgs::PointCloud pc;
		// sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);
		// sensor_msgs::PointCloud pc_after;
		// pc_after.header.frame_id = "/centerlaser_";
		// listener_.transformPointCloud("/centerlaser_", pc, pc_after);
		// cout<<"pc_after.header.frame_id : "<<pc_after.header.frame_id<<endl;
		// sensor_msgs::PointCloud2 pc2_after;
		// sensor_msgs::convertPointCloudToPointCloud2(pc_after, pc2_after);
		// cout<<"Subscribe sq_lidar/scan_raw"<<endl;

		// pub_pointcloud_.publish(pc2_after);
		// // pub_pointcloud_.publish(pc2);

	// }
	// // if(msg->header.frame_id == "laser2_link"){
		// // sensor_msgs::LaserScan output2;
		// // output2 = *msg;
		// // pub_laser_debug2_.publish(output2);
	// // }
	// // if(msg->header.frame_id == "laser3_link"){
		// // sensor_msgs::LaserScan output3;
		// // output3 = *msg;
		// // pub_laser_debug3_.publish(output3);
	// // }

// }

// void Laser2PointCloud::sq_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	// if(!listener_.waitForTransform(
				// msg->header.frame_id, 
				// "/pillar_link", 
				// msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
				// ros::Duration(1.0))){
		// return;
	// }

	// sensor_msgs::PointCloud2 pc;
	// projector_.transformLaserScanToPointCloud("/pillar_link", *msg, pc, listener_);
	// cout<<"Subscribe sq_lidar/scan_raw"<<endl;
	// pub_pointcloud_.publish(pc);
// }

// void Laser2PointCloud::sq_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg){

	// sensor_msgs::PointCloud2 pc;
	// projector_.transformLaserScanToPointCloud("/centerlaser_", *msg, pc, listener_);
	// cout<<"Subscribe sq_lidar/scan_raw"<<endl;
	// pub_pointcloud_.publish(pc);
// }


int main(int argc, char** argv){
	ros::init(argc, argv, "laser2pointcloud");

	cout<<"!!!!! Here we go !!!!!!"<<endl;

	Laser2PointCloud converter;

	ros::spin();
	return 0;
}
