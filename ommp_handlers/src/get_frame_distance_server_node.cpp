#include <integration/GetFrameDistance.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

geometry_msgs::TransformStamped findTransform(std::string parent_frame, std::string child_frame)
{
  tf2_ros::Buffer br;
  br.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tf2_listener(br);
  geometry_msgs::TransformStamped transform;

  try
  {
    transform = br.lookupTransform(parent_frame, child_frame, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::LookupException &e)
  {
    ROS_ERROR("%s", e.what());
    transform.header.frame_id = "EXCEPTION";
  }
  catch (tf2::ConnectivityException &e)
  {
    ROS_ERROR("%s", e.what());
    transform.header.frame_id = "EXCEPTION";
  }

  return transform;
}

bool getFrameDistance(integration::GetFrameDistance::Request &req, integration::GetFrameDistance::Response &res)
{
  geometry_msgs::TransformStamped transform = findTransform(req.source_frame, req.target_frame);

  if (transform.header.frame_id == "EXCEPTION")
  {
    ROS_ERROR("Could not find the relation between the two given frames");
    return false;
  }
  else
  {
    res.pose.header = transform.header;
    res.pose.pose.position.x = transform.transform.translation.x;
    res.pose.pose.position.y = transform.transform.translation.y;
    res.pose.pose.position.z = transform.transform.translation.z;
    res.pose.pose.orientation.x = transform.transform.rotation.x;
    res.pose.pose.orientation.y = transform.transform.rotation.y;
    res.pose.pose.orientation.z = transform.transform.rotation.z;
    res.pose.pose.orientation.w = transform.transform.rotation.w;
    return true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_frame_distance");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("get_frame_distance", getFrameDistance);
  ROS_INFO("Server is ready to receive goals");
  ros::spin();

  return 0;
}