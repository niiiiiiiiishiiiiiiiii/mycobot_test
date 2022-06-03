#include "ros/ros.h"

#include "moveit_msgs/PlanningScene.h"
#include "moveit_msgs/CollisionObject.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mycobot_obstacles");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  // We create a publisher and wait for subscribers.
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }

  // create objects
  moveit_msgs::CollisionObject collision_object;
  
  collision_object.id = "cuboid";
  collision_object.header.frame_id = "world";

  geometry_msgs::Pose pose;
  pose.position.x = private_node_handle.param("primitive/pose/position/x", 0.1);
  pose.position.y = private_node_handle.param("primitive/pose/position/y", 0.0);
  pose.position.z = private_node_handle.param("primitive/pose/position/z", 0.025);
  pose.orientation.x = private_node_handle.param("primitive/pose/orientation/x", 1.0);
  pose.orientation.y = private_node_handle.param("primitive/pose/orientation/y", 0.0);
  pose.orientation.z = private_node_handle.param("primitive/pose/orientation/z", 0.0);
  pose.orientation.w = private_node_handle.param("primitive/pose/orientation/w", 0.0);
  
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = private_node_handle.param(
    "primitive/dimensions/", std::vector<double>({ 0.05, 0.05, 0.05 })
  );

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);

  collision_object.operation = collision_object.ADD;

  ROS_INFO("Adding the object into the world");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(collision_object);
  planning_scene.is_diff = true;

  planning_scene_diff_publisher.publish(planning_scene);
  
  ros::shutdown();
  return 0;
}