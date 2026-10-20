/**
 * @file ik_collision_checker.h
 * @brief Defines the MoveCollisionChecker class for checking collisions in a planning scene.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ik_solver_msgs/srv/collision_checking.hpp>
#include <ik_solver_msgs/msg/collision_result.hpp>
#include <ik_solver_msgs/msg/configuration.hpp>

// MoveIt >= Jazzy ships .hpp headers, Humble only .h
#if __has_include(<moveit/planning_scene/planning_scene.hpp>)
#include <moveit/planning_scene/planning_scene.hpp>
#else
#include <moveit/planning_scene/planning_scene.h>
#endif
#include <moveit_msgs/srv/get_planning_scene.hpp>
#if __has_include(<moveit/robot_model_loader/robot_model_loader.hpp>)
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#else
#include <moveit/robot_model_loader/robot_model_loader.h>
#endif
#if __has_include(<moveit/collision_detection/collision_matrix.hpp>)
#include <moveit/collision_detection/collision_matrix.hpp>
#else
#include <moveit/collision_detection/collision_matrix.h>
#endif
#if __has_include(<moveit/collision_detection/world.hpp>)
#include <moveit/collision_detection/world.hpp>
#else
#include <moveit/collision_detection/world.h>
#endif
#include <mutex>
#include <atomic>
#include <cnr_param/cnr_param.h>

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
     * @brief Atomic flag to indicate if a collision check request is currently being processed.
     * This is used to prevent concurrent modifications to the planning scene while a check is in progress.
     */
    std::atomic<bool> is_processing_request_{false};
    /**
     * @brief Constructor.
     * @param nh A shared pointer to the rclcpp::Node.
     */
    MoveCollisionChecker(const rclcpp::Node::SharedPtr &nh);

    /**
     * @brief Destructor.
     */
    ~MoveCollisionChecker();

    /**
     * @brief Service callback to perform collision checking for a set of joint configurations.
     * @param req The service request, containing the planning scene and solutions to check.
     * @param res The service response, which will be populated with the collision check results.
     */
    void collisionCheck(const std::shared_ptr<ik_solver_msgs::srv::CollisionChecking::Request> req,
                        std::shared_ptr<ik_solver_msgs::srv::CollisionChecking::Response> res);

    /**
     * @brief Updates the internal planning scene from a message.
     * @param scene The moveit_msgs::msg::PlanningScene message to apply.
     * @param is_diff True if `scene` is a differential update, false for a full scene update.
     */
    void updatePlanningScene(const moveit_msgs::msg::PlanningScene &scene, bool is_diff);

    /**
     * @brief Initializes the MoveCollisionChecker object, creating the service server.
     */
    void init();

  protected:
    rclcpp::Node::SharedPtr nh_;                                                ///< ROS 2 node shared pointer.
    rclcpp::Service<ik_solver_msgs::srv::CollisionChecking>::SharedPtr server_; ///< The service server for collision checking.

    std::shared_ptr<planning_scene::PlanningScene> planning_scene_; ///< Shared pointer to the MoveIt PlanningScene.

    std::mutex scene_mtx_;   ///< Mutex to protect access to the planning scene.
    std::string group_name_; ///< The name of the MoveIt joint model group to check.

    // std::string param_ns_ = "/collision_check";                ///< Parameter namespace.
    robot_model_loader::RobotModelLoader robot_model_loader_; ///< The robot model loader for MoveIt.

    double padding_ = 0.01; ///< Padding to be applied to robot links for collision checking.
    double scaling_ = 1.0;  ///< Scaling to be applied to robot links for collision checking.
  };

} // end namespace ik_solver