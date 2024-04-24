/**
 * @file ik_collision_checker.h
 * @brief Defines the MoveCollisionChecker class for checking collisions in a planning scene.
 */

#pragma once

#include <ros/ros.h>
#include <ik_solver_msgs/CollisionChecking.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

/**
 * @namespace ik_solver
 * @brief Contains classes and functions for IK solver.
 */
namespace ik_solver
{

/**
 * @class MoveCollisionChecker
 * @brief Class for checking collisions in a planning scene.
 */
class MoveCollisionChecker
{
public:
  /**
   * @brief Constructor.
   * @param nh NodeHandle object.
   */
  MoveCollisionChecker(const ros::NodeHandle& nh);

  /**
   * @brief Destructor.
   */
  ~MoveCollisionChecker();

  /**
   * @brief Checks for collisions.
   * @param req CollisionChecking request.
   * @param res CollisionChecking response.
   * @return True if no collision, false otherwise.
   */
  bool collisionCheck(ik_solver_msgs::CollisionChecking::Request& req,
                      ik_solver_msgs::CollisionChecking::Response& res);

  /**
   * @brief Updates the planning scene.
   * @param scene PlanningScene object.
   */
  void updatePlanningScene(const moveit_msgs::PlanningScene& scene);

  /**
   * @brief Initializes the MoveCollisionChecker object.
   */
  void init();

protected:
  ros::NodeHandle nh_;  ///< NodeHandle object.
  ros::ServiceServer server_;  ///< ServiceServer object.

  std::shared_ptr<planning_scene::PlanningScene> planning_scene_;  ///< Shared pointer to PlanningScene.

  std::mutex scene_mtx_;  ///< Mutex for scene.
  std::string group_name_;  ///< Group name.

  robot_model_loader::RobotModelLoader robot_model_loader_;  ///< RobotModelLoader object.

};

}  // end namespace ik_solver
