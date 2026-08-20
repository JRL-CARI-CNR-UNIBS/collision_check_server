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
 * @brief Main function for the IK collision checker node.
 *
 * This node provides a service (`/collision_check`) for checking joint configurations against the current planning scene.
 * It periodically fetches the latest planning scene from the `/get_planning_scene` service (typically provided by MoveIt)
 * and uses it to update its internal state. The core collision checking logic is encapsulated in the `MoveCollisionChecker` class.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return 0 on successful execution, -1 otherwise.
 */
int main(int argc, char **argv)
{
  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("ik_collision_check");

  // Use a MultiThreadedExecutor to handle callbacks from the node and service client concurrently.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);
  // Run the executor in a separate thread to avoid blocking the main thread.
  std::thread spinner_thread([&executor]()
                             { executor.spin(); });

  // Create a service client for getting the planning scene
  auto ps_client = nh->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  // Wait for the planning scene service to become available.
  while (!ps_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(nh->get_logger(), "Interrupted while waiting for the '/get_planning_scene' service. Exiting.");
      return -1;
    }
  }

  // Perform an initial request to get the full planning scene.
  // This ensures the collision checker is initialized with a valid scene.
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
  // Asynchronously call the service to get the planning scene.
  auto result_future = ps_client->async_send_request(ps_req);
  if (result_future.wait_for(10s) != std::future_status::ready)
  {
    RCLCPP_ERROR(nh->get_logger(), "Failed to get initial planning scene. Shutting down.");
    rclcpp::shutdown();
    spinner_thread.join();
    return -1;
  }
  // Note: The result of this initial call is not directly used here, as the
  // MoveCollisionChecker constructor initializes its own scene. However, this call
  // confirms the service is responsive before proceeding.

  // Initialize the collision checker
  ik_solver::MoveCollisionChecker cc(nh);
  cc.init();

  // Start the main loop to periodically update the planning scene.
  rclcpp::Rate lp(10);
  RCLCPP_INFO(nh->get_logger(), "Starting periodic planning scene updates.");
  while (rclcpp::ok())
  {
    // If a collision check request is currently being processed, skip this update cycle
    // to avoid modifying the planning scene during a check.
    if (cc.is_processing_request_)
    {
      lp.sleep();
      continue;
    }

    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    // Request all scene components to get the latest updates.
    req->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                                 moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
                                 moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
                                 moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                 moveit_msgs::msg::PlanningSceneComponents::OCTOMAP |
                                 moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS |
                                 moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX | // The allowed collision matrix
                                 moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING;  // Link padding and scaling information
    // Asynchronously send the request to get the latest planning scene.
    auto future = ps_client->async_send_request(req);
    if (future.wait_for(20s) != std::future_status::ready)
    {
      RCLCPP_ERROR(nh->get_logger(), "Failed to get planning scene update. Exiting loop.");
      break;
    }
    else
    {
      auto response = future.get();

      // The planning scene is updated with 'is_diff' set to false,
      // which means a full scene update is performed.
      cc.updatePlanningScene(response->scene, false);
    }
    lp.sleep();
  }

  RCLCPP_INFO(nh->get_logger(), "Exiting planning scene update loop. Shutting down.");
  rclcpp::shutdown();
  spinner_thread.join();
  return 0;
}