#include <collision_checker_server/ik_collision_checker.hpp>
#include <malloc.h>

// This code is part of the ik_solver namespace
namespace ik_solver
{
  /**
   * @brief Construct a new Move Collision Checker object.
   *
   * @param nh The ROS 2 node handle.
   */
  MoveCollisionChecker::MoveCollisionChecker(const rclcpp::Node::SharedPtr &nh) : robot_model_loader_(nh, "robot_description")
  {
    // Store the node handle
    nh_ = nh;

    // The group_name_ member variable is assigned the value of the "group_name" parameter
    // If the parameter is not specified, an exception is thrown
    if (!nh_->has_parameter("group_name"))
    {
      nh_->declare_parameter<std::string>("group_name", "");
    }

    if (!nh_->get_parameter("group_name", group_name_) || group_name_.empty())
    {
      throw std::invalid_argument(std::string(nh_->get_namespace()) + "/group_name is not specified");
    }

    // Initialize the planning scene from the robot model
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_loader_.getModel());

    // Get the current state of the robot from the planning scene
    moveit::core::RobotState state = planning_scene_->getCurrentState();

    // Get the names of the active joints in the specified group
    std::vector<std::string> jmg_names = state.getJointModelGroup(group_name_)->getActiveJointModelNames();

    // Log the name and index of each active joint in the group
    for (const std::string &jname : jmg_names)
    {
      RCLCPP_INFO(nh_->get_logger(), "joint %s, index = %d", jname.c_str(),
                  state.getJointModelGroup(group_name_)->getJointModel(jname)->getFirstVariableIndex());
    }

    // Get the names of all joints in the group, including passive and mimic joints
    std::vector<std::string> full_jmg_names = state.getJointModelGroup(group_name_)->getJointModelNames();

    // Log the name, index, and mimic joint index (if any) for all joints in the group
    for (const std::string &jname : full_jmg_names)
    {
      const auto *joint_model = state.getJointModelGroup(group_name_)->getJointModel(jname);

      if (joint_model->getMimic())
      {
        RCLCPP_INFO(nh_->get_logger(), "joint %s, index = %zu, mimic index = %zu", jname.c_str(), joint_model->getFirstVariableIndex(),
                    joint_model->getMimic()->getJointIndex());
      }
      else
      {
        RCLCPP_INFO(nh_->get_logger(), "joint %s, index = %zu", jname.c_str(), joint_model->getFirstVariableIndex());
      }
    }
    std::string what;
    std::string padding;
    std::string scaling;
    RCLCPP_INFO(nh_->get_logger(), "NH namespace: %s", nh_->get_namespace());
    // Get padding and scaling parameters for collision checking
    cnr::param::get(std::string(nh_->get_namespace()) + "/padding", padding_, what);
    cnr::param::get(std::string(nh_->get_namespace()) + "/scaling", scaling_, what);
    RCLCPP_INFO(nh_->get_logger(), "Collision checking padding: %.3f, scaling: %.3f", padding_, scaling_);
  }
  /**
   * @brief Destroy the Move Collision Checker object.
   */
  MoveCollisionChecker::~MoveCollisionChecker()
  {
    planning_scene_.reset();
  }

  void MoveCollisionChecker::init()
  {
    // Create the service server for collision checking
    server_ = nh_->create_service<ik_solver_msgs::srv::CollisionChecking>(
        "/collision_check",
        std::bind(&MoveCollisionChecker::collisionCheck, this, std::placeholders::_1, std::placeholders::_2));
  }

  /**
   * @brief Service callback to perform collision checking for a set of joint configurations.
   *
   * This function receives a request containing a planning scene and a vector of joint solutions.
   * For each solution, it checks for joint limit violations and collisions with the environment.
   * The results of these checks are returned in the response.
   *
   * @param req The service request, containing the planning scene and solutions to check.
   * @param res The service response, which will be populated with the collision check results.
   */
  void MoveCollisionChecker::collisionCheck(const std::shared_ptr<ik_solver_msgs::srv::CollisionChecking::Request> req,
                                            std::shared_ptr<ik_solver_msgs::srv::CollisionChecking::Response> res)
  {
    is_processing_request_ = true;
    // Update the planning scene with the scene provided in the request
    updatePlanningScene(req->planning_scene, true);
    RCLCPP_INFO(nh_->get_logger(), "Received collision check request with %zu solutions", req->solutions.size());
    // Get the current state of the robot from the planning scene
    moveit::core::RobotState state = planning_scene_->getCurrentState();

    // Get the names of the active joints in the specified group
    std::vector<std::string> jmg_names = state.getJointModelGroup(group_name_)->getActiveJointModelNames();

    // Check if the number of joint names in the request matches the number of active joints in the group
    if (jmg_names.size() != req->joint_names.size())
    {
      RCLCPP_WARN(nh_->get_logger(), "Joint names do not match");
      RCLCPP_WARN(nh_->get_logger(), "Request:");
      for (const std::string &s : req->joint_names)
        RCLCPP_WARN(nh_->get_logger(), "- %s", s.c_str());
      RCLCPP_WARN(nh_->get_logger(), "Joint model group %s:", group_name_.c_str());
      for (const std::string &s : jmg_names)
        RCLCPP_WARN(nh_->get_logger(), "- %s", s.c_str());
      RCLCPP_WARN(nh_->get_logger(), "RESORTING IS NOT IMPLEMENTED YET");
      is_processing_request_ = false;
      return;
    }

    // Configure the collision request
    collision_detection::CollisionRequest col_req;
    col_req.contacts = req->detailed;              // Request contact information if detailed results are needed
    col_req.distance = req->detailed;              // Request distance information if detailed results are needed
    col_req.max_contacts = req->detailed ? 10 : 0; // Max contacts to report
    col_req.group_name = "";                       // Check collisions for the entire robot

    // Lock the planning scene to prevent concurrent modifications during collision checking
    scene_mtx_.lock();

    const collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
    const size_t num_solutions = req->solutions.size();
    res->result.resize(num_solutions);

#pragma omp parallel
    {
      moveit::core::RobotState thread_state = planning_scene_->getCurrentState();
      collision_detection::CollisionRequest thread_col_req = col_req;
      const auto *jmg = thread_state.getJointModelGroup(group_name_);
      const auto &local_jmg_names = jmg->getActiveJointModelNames();

#pragma omp for schedule(dynamic, 32)
      for (size_t conf_idx = 0; conf_idx < num_solutions; ++conf_idx)
      {
        const auto &c = req->solutions[conf_idx];
        ik_solver_msgs::msg::CollisionResult r;
        r.solution = c;

        // Set the joint positions in the thread-local robot state
        for (size_t ij = 0; ij < req->joint_names.size(); ij++)
          thread_state.setJointPositions(req->joint_names.at(ij), &c.configuration.at(ij));

        // 1. JOINT LIMITS CHECK (OUT OF BOUNDS)
        if (!thread_state.satisfiesBounds(jmg))
        {
          r.feasibility = 0.0;
          r.out_of_bound = true;

          for (const std::string &jname : local_jmg_names)
          {
            if (!thread_state.satisfiesPositionBounds(jmg->getJointModel(jname)))
            {
              r.out_of_bound_links.push_back(jname);
            }
          }
        }
        else
        {
          r.out_of_bound = false;
          // Update the transforms of all collision bodies for this thread's state
          thread_state.updateCollisionBodyTransforms();

          // 2. COLLISION CHECK
          collision_detection::CollisionResult col_res;
          planning_scene_->checkCollision(thread_col_req, col_res, thread_state, acm);

          r.feasibility = (col_res.collision) ? 0.0 : 1.0;

          // Populate detailed data if requested by the client
          if (req->detailed)
          {
            r.distance = col_res.distance;
            for (const auto &contact : col_res.contacts)
            {
              r.colliding_obj_first.push_back(contact.first.first);
              r.colliding_obj_second.push_back(contact.first.second);
              RCLCPP_INFO(nh_->get_logger(), "Collision detected between '%s' and '%s'",
                          contact.first.first.c_str(), contact.first.second.c_str());
            }
          }
        }

        res->result[conf_idx] = std::move(r);
      }
    }

    // Unlock the scene mutex
    scene_mtx_.unlock();
    is_processing_request_ = false;
    malloc_trim(0);
  }

  /**
   * @brief Updates the planning scene with a given scene message, applying custom padding and scaling.
   *
   * This function takes a `moveit_msgs::msg::PlanningScene`, applies the configured padding
   * and scaling to all robot links, and then updates the internal planning scene.
   * The update can be a full scene update or a differential update.
   *
   * @param scene The `moveit_msgs::msg::PlanningScene` message to apply.
   * @param is_diff True if `scene` is a differential update, false for a full scene update.
   */
  void MoveCollisionChecker::updatePlanningScene(const moveit_msgs::msg::PlanningScene &scene, bool is_diff)
  {
    scene_mtx_.lock();
    // Create a local copy of the message to modify it
    moveit_msgs::msg::PlanningScene modified_scene = scene;

    // Get all link names from the robot model
    const std::vector<std::string> &link_names = planning_scene_->getRobotModel()->getLinkModelNames();

    // Apply padding and scaling to each link
    for (const std::string &link_name : link_names)
    {
      // Add the Padding directive
      moveit_msgs::msg::LinkPadding lp;
      lp.link_name = link_name;
      lp.padding = padding_;
      modified_scene.link_padding.push_back(lp);

      // Add the Scaling directive
      moveit_msgs::msg::LinkScale ls;
      ls.link_name = link_name;
      ls.scale = scaling_;
      modified_scene.link_scale.push_back(ls);
    }

    // Update the planning scene with the modified message
    bool success;
    if (is_diff)
    {
      // Apply the modified scene as a differential update
      success = planning_scene_->setPlanningSceneDiffMsg(modified_scene);
    }
    else
    {
      // Apply the modified scene as a full update
      success = planning_scene_->setPlanningSceneMsg(modified_scene);
    }

    if (!success)
    {
      RCLCPP_ERROR(nh_->get_logger(), "Unable to update the Planning Scene with custom parameters");
    }
    scene_mtx_.unlock();
  }
} // namespace ik_solver