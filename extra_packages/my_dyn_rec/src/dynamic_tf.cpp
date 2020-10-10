#include <dynamic_reconfigure/server.h>
#include <my_dyn_rec/lidartfConfig.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void callback(my_dyn_rec::lidartfConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f \n", config.lidar_px,
           config.lidar_py, config.lidar_pz, config.lidar_or, config.lidar_op,
           config.lidar_oy);
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f \n", config.camera_px,
           config.camera_py, config.camera_pz, config.camera_or,
           config.camera_op, config.camera_oy);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(
      tf::Vector3(config.lidar_px, config.lidar_py, config.lidar_pz));
  tf::Quaternion q;
  q.setRPY(config.lidar_or, config.lidar_op, config.lidar_oy);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "chassis",
                                        "laser_link"));

  tf::Transform transform1;
  transform1.setOrigin(
      tf::Vector3(config.camera_px, config.camera_py, config.camera_pz));
  tf::Quaternion q1;
  q1.setRPY(config.camera_or, config.camera_op, config.camera_oy);
  transform1.setRotation(q1);
  br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(),
                                         "kinect_base2", "kinect_dummy"));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_dyn_tf");

  dynamic_reconfigure::Server<my_dyn_rec::lidartfConfig> server;
  dynamic_reconfigure::Server<my_dyn_rec::lidartfConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}