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
    if(counter1==skip){
      pc_pub.publish(*cmd);
      counter1=0;
    }
    counter1++;
      
  }
  void imd_CB(const sensor_msgs::Image::ConstPtr &cmd) {
    if(counter2==skip){
    imd_pub.publish(*cmd);
    counter2=0;
    }
    counter2++;
  }
  void imi_CB(const sensor_msgs::Image::ConstPtr &cmd) {
    if(counter3==skip){
    imi_pub.publish(*cmd);
          counter3=0;
    }
    counter3++;
  }
  void info_CB(const sensor_msgs::CameraInfo::ConstPtr &cmd) {
    if(counter4==skip){
    info_pub.publish(*cmd);
      counter4=0;
    }
    counter4++;
  }
protected:
  ros::NodeHandle nh;
  ros::Subscriber pc_sub, imi_sub, imd_sub, info_sub;
  ros::Publisher pc_pub, imi_pub, imd_pub, info_pub;
  int counter1=0, counter2=0, counter3=0, counter4=0, skip=5;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vel_to_arduino");
  CameraRelay handler;
  ros::spin();
  return 0;
}
