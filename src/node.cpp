/**
 * @file node.cpp
 * @brief Main file for the IK collision checker node.
 */

// ROS includes
#include <rclcpp/rclcpp.hpp>

// Local includes
#include <collision_checker_server/ik_collision_checker.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief Main function for the IK collision checker node. The node exposes a service for checking collisions. The scene
 * is updated by subscribing to the /get_planning_scene service. The collision checking is perform  by the
 * MoveCollisionChecker class. This class is inline commented inside the header and cpp files.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return 0 on successful execution, -1 otherwise.
 */
int main(int argc, char **argv)
{
  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("ik_collision_check");

  // Start ROS spinner
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);
  std::thread spinner_thread([&executor]()
                             { executor.spin(); });

  // Create a service client for getting the planning scene
  auto ps_client = nh->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  while (!ps_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(nh->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return -1;
    }
  }

  auto ps_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  // Explicitly request the robot state and world objects
  ps_req->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                                  moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
                                  moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
                                  moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                  moveit_msgs::msg::PlanningSceneComponents::OCTOMAP |
                                  moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS |
                                  moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
                                  moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING;
  // Call the service to get the planning scene
  auto result_future = ps_client->async_send_request(ps_req);
  if (result_future.wait_for(10s) != std::future_status::ready)
  {
    RCLCPP_ERROR(nh->get_logger(), "Error on  get_planning_scene srv not ok");
    rclcpp::shutdown();
    spinner_thread.join();
    return -1;
  }

  // Initialize the collision checker
  ik_solver::MoveCollisionChecker cc(nh);
  cc.init();

  // Loop for updating the planning scene
  rclcpp::Rate lp(10);
  RCLCPP_INFO(nh->get_logger(), "Infinite Loop for Update the Planning Scene");
  bool first_lap = true;
  while (rclcpp::ok())
  {
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    // Explicitly request the robot state and world objects
    req->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                                 moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
                                 moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
                                 moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                 moveit_msgs::msg::PlanningSceneComponents::OCTOMAP |
                                 moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS |
                                 moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
                                 moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING; // Call the service to get the planning scene
    auto future = ps_client->async_send_request(req);
    if (future.wait_for(2s) != std::future_status::ready)
    {
      RCLCPP_ERROR(nh->get_logger(), "Error on  get_planning_scene srv not ok. Brutal Exit.");
      break;
    }
    else
    {
      auto response = future.get();

      response->scene.is_diff = true;             // Set is_diff to true to indicate that this is a differential update
      response->scene.robot_state.is_diff = true; // Set is_diff to true for the robot state as well
      // Update the planning scene in the collision checker
      cc.updatePlanningScene(response->scene);
    }
    lp.sleep();
  }

  RCLCPP_ERROR(nh->get_logger(), "Exit from infinite loop ");
  rclcpp::shutdown();
  spinner_thread.join();
  return 0;
}