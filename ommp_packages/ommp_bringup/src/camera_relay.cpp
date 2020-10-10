#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

class CameraRelay {
public:
  CameraRelay() {
    pc_sub = nh.subscribe("/camera/depth_registered/points", 2, &CameraRelay::pc_CB, this);
    imd_sub = nh.subscribe("/camera/depth_registered/image_raw", 2, &CameraRelay::imd_CB, this);
    imi_sub = nh.subscribe("/camera/rgb/image_raw", 2, &CameraRelay::imi_CB, this);
    info_sub = nh.subscribe("/camera/rgb/camera_info", 2, &CameraRelay::info_CB, this);

    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/my_camera/depth_registered/points", 2);
    imd_pub = nh.advertise<sensor_msgs::Image>("/my_camera/depth_registered/image_raw", 2);
    imi_pub = nh.advertise<sensor_msgs::Image>("/my_camera/rgb/image_raw", 2);
    info_pub = nh.advertise<sensor_msgs::CameraInfo>("/my_camera/rgb/camera_info", 2);
    
  }

  void pc_CB(const sensor_msgs::PointCloud2::ConstPtr &cmd) {
    pc_pub.publish(*cmd);
  }
  void imd_CB(const sensor_msgs::Image::ConstPtr &cmd) {
    imd_pub.publish(*cmd);
  }
  void imi_CB(const sensor_msgs::Image::ConstPtr &cmd) {
    imi_pub.publish(*cmd);
  }
  void info_CB(const sensor_msgs::CameraInfo::ConstPtr &cmd) {
    info_pub.publish(*cmd);
  }
protected:
  ros::NodeHandle nh;
  ros::Subscriber pc_sub, imi_sub, imd_sub, info_sub;
  ros::Publisher pc_pub, imi_pub, imd_pub, info_pub;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vel_to_arduino");
  CameraRelay handler;
  ros::spin();
  return 0;
}
