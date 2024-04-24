/**
 * @file node.cpp
 * @brief Main file for the IK collision checker node.
 */

// ROS includes
#include <ros/ros.h>

// Local includes
#include <collision_checker_server/ik_collision_checker.h>
#include <moveit_msgs/GetPlanningScene.h>

/**
 * @brief Main function for the IK collision checker node. The node exposes a service for checking collisions. The scene
 * is updated by subscribing to the /get_planning_scene service. The collision checking is perform  by the
 * MoveCollisionChecker class. This class is inline commented inside the header and cpp files.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return 0 on successful execution, -1 otherwise.
 */
int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "ik_collision_check");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Start ROS spinner
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Create a service client for getting the planning scene
  ros::ServiceClient ps_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  ps_client.waitForExistence();
  moveit_msgs::GetPlanningScene ps_srv;

  // Call the service to get the planning scene
  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("Error on  get_planning_scene srv not ok");
    return -1;
  }

  // Initialize the collision checker
  ik_solver::MoveCollisionChecker cc(pnh);
  cc.init();

  // Loop for updating the planning scene
  ros::Rate lp(10);
  ROS_INFO("Infinite Loop for Update the Planning Scene");
  while (ros::ok())
  {
    // Call the service to get the planning scene
    if (!ps_client.call(ps_srv))
    {
      ROS_ERROR("Error on  get_planning_scene srv not ok. Brutal Exit.");
      return -1;
    }
    else
    {
      // Update the planning scene in the collision checker
      cc.updatePlanningScene(ps_srv.response.scene);
    }
    lp.sleep();
  }

  ROS_ERROR("Exit from infinite loop ");
  return 0;
}