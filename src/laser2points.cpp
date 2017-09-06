#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

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
	public:
		Laser2PointCloud();
		void sq_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

Laser2PointCloud::Laser2PointCloud(){
	sub_sq_lidar_ = n_.subscribe<sensor_msgs::LaserScan>("/sq_lidar/scan_raw", 1, &Laser2PointCloud::sq_lidar_callback, this);


	pub_pointcloud_ = n_.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points", 1);


	listener_.setExtrapolationLimit(ros::Duration(0.1));
}

void Laser2PointCloud::sq_lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	sensor_msgs::LaserScan output1;
	output1 = *msg;
	sensor_msgs::PointCloud2 pc2;
	try{
		string frame_name = output1.header.frame_id;
		projector_.transformLaserScanToPointCloud("/centerlaser_", output1, pc2, listener_);
		pub_pointcloud_.publish(pc2);
	}
	catch(tf::TransformException &ex){
		printf ("Failure %s\n", ex.what());
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "laser2pointcloud");

	cout<<"!!!!! Here we go !!!!!!"<<endl;

	Laser2PointCloud converter;

	ros::spin();
	return 0;
}