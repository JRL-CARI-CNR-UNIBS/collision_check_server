/**
 * @file ik_collision_checker.h
 * @brief Defines the MoveCollisionChecker class for checking collisions in a planning scene.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ik_solver_msgs/srv/collision_checking.hpp>
#include <ik_solver_msgs/msg/collision_result.hpp>
#include <ik_solver_msgs/msg/configuration.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <mutex>

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
    MoveCollisionChecker(const rclcpp::Node::SharedPtr &nh);

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
    void collisionCheck(const std::shared_ptr<ik_solver_msgs::srv::CollisionChecking::Request> req,
                        std::shared_ptr<ik_solver_msgs::srv::CollisionChecking::Response> res);

    /**
     * @brief Updates the planning scene.
     * @param scene PlanningScene object.
     */
    void updatePlanningScene(const moveit_msgs::msg::PlanningScene &scene);

    /**
     * @brief Initializes the MoveCollisionChecker object.
     */
    void init();

  protected:
    rclcpp::Node::SharedPtr nh_;                                                ///< NodeHandle object.
    rclcpp::Service<ik_solver_msgs::srv::CollisionChecking>::SharedPtr server_; ///< ServiceServer object.

    std::shared_ptr<planning_scene::PlanningScene> planning_scene_; ///< Shared pointer to PlanningScene.

    std::mutex scene_mtx_;   ///< Mutex for scene.
    std::string group_name_; ///< Group name.

    robot_model_loader::RobotModelLoader robot_model_loader_; ///< RobotModelLoader object.
  };

} // end namespace ik_solver