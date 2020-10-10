#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


int main (int argc, char** argv)
{
ros::init (argc, argv, "pcl_sample");
ros::NodeHandle nh;
ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
sensor_msgs::PointCloud2 output;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new
pcl::PointCloud<pcl::PointXYZ>);
// Fill in the cloud data
cloud->width = 100;
cloud->height = 1;
cloud->points.resize (cloud->width * cloud->height);
//Convert the cloud to ROS message
pcl::toROSMsg (*cloud, output);
pcl_pub.publish(output);
ros::spinOnce();
return 0;
}