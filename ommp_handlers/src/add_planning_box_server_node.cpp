#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>

// Integration
#include <integration/AddBox.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

class AddPlanningBox
{
protected:
  ros::NodeHandle nh_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::CollisionObject collision_object;
  ros::ServiceServer service_;

public:
  explicit AddPlanningBox(ros::NodeHandle nh) : nh_(nh)
  {
    service_ = nh_.advertiseService("add_planning_box_routine", &AddPlanningBox::Routine, this);
    ROS_INFO("Object ADD-REMOVE Service Server Started");
  }
  ~AddPlanningBox()
  {
  }

  bool Routine(integration::AddBox::Request &req, integration::AddBox::Response &res)
  {
    if (req.add)
    {
      collision_object.header.frame_id = req.planning_frame;
      collision_object.id = req.object_id;
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = req.size_x;
      primitive.dimensions[1] = req.size_y;
      primitive.dimensions[2] = req.size_z;
      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(req.object_pose);
      collision_object.operation = collision_object.ADD;
      std::vector<moveit_msgs::CollisionObject> collision_objects;
      collision_objects.push_back(collision_object);
      planning_scene_interface.addCollisionObjects(collision_objects);
      /* Sleep so we have time to see the object in RViz */
      sleep(1.0);
    }
    else
    {
      collision_object.header.frame_id = req.planning_frame;
      collision_object.id = req.object_id;
      ROS_INFO("Remove the object from the world");
      std::vector<std::string> object_ids;
      object_ids.push_back(collision_object.id);
      planning_scene_interface.removeCollisionObjects(object_ids);
      /* Sleep to give Rviz time to show the object is no longer there. */
      sleep(1.0);
    }
    res.success = true;
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_planning_box_server_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  AddPlanningBox my_add_planning_box(nh);
  ros::waitForShutdown();
  return 0;
}
