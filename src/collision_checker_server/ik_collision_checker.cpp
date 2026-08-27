#include <collision_checker_server/ik_collision_checker.hpp>

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

    // Lock the planning scene to prevent concurrent modifications
    scene_mtx_.lock();

    // Iterate over each provided joint configuration (solution)
    for (size_t conf_idx = 0; conf_idx < req->solutions.size(); ++conf_idx)
    {
      auto &c = req->solutions[conf_idx];
      ik_solver_msgs::msg::CollisionResult r;
      r.solution = c;

      // Set the joint positions in the robot state for the current configuration
      for (size_t ij = 0; ij < req->joint_names.size(); ij++)
        state.setJointPositions(req->joint_names.at(ij), &c.configuration.at(ij));

      // 1. JOINT LIMITS CHECK (OUT OF BOUNDS)
      if (!state.satisfiesBounds(state.getJointModelGroup(group_name_)))
      {
        r.feasibility = 0.0;
        r.out_of_bound = true;

        // Identify which joints are out of bounds
        for (const std::string &jname : jmg_names)
        {
          if (!state.satisfiesPositionBounds(state.getJointModelGroup(group_name_)->getJointModel(jname)))
          {
            r.out_of_bound_links.push_back(jname);

            // Extract the actual value of the joint to print it
            const double *joint_value = state.getJointPositions(jname);
            RCLCPP_INFO(nh_->get_logger(), "[Configuration %zu Failed] Joint out of bounds: '%s' = %.3f", conf_idx,
                        jname.c_str(), joint_value[0]);
          }
        }
      }
      else
      {
        r.out_of_bound = false;
        // Update the transforms of all collision bodies
        state.updateCollisionBodyTransforms();
        // Get the allowed collision matrix from the planning scene
        collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
        moveit::core::RobotState copied_state = planning_scene_->getCurrentState();

        // RCLCPP_INFO(nh_->get_logger(), "Allowed Collision Matrix size: %zu entries", acm.getSize());
        // RCLCPP_INFO(nh_->get_logger(), "Allowed Collision Matrix:");
        // std::vector<std::string> entry_names;
        // acm.getAllEntryNames(entry_names);
        // for (size_t i = 0; i < entry_names.size(); ++i)
        // {
        //   std::string entry1 = entry_names[i];
        //   RCLCPP_INFO(nh_->get_logger(), "Entry %s - Allowed collisions: ", entry1.c_str());

        //   for (size_t j = 1; j < entry_names.size(); ++j)
        //   {
        //     std::string entry2 = entry_names[j];
        //     collision_detection::AllowedCollision::Type allowed_collision_type;
        //     if (acm.getAllowedCollision(entry1, entry2, allowed_collision_type))
        //     {
        //       if (allowed_collision_type == collision_detection::AllowedCollision::ALWAYS)
        //       {
        //         RCLCPP_INFO(nh_->get_logger(), "  - %s ", entry2.c_str());
        //       }
        //     }
        //   }
        // }
        // std::vector<std::string> world_objects = planning_scene_->getWorld()->getObjectIds();
        // RCLCPP_INFO(nh_->get_logger(), "Planning Scene has %zu collision objects.", world_objects.size());
        // for (const auto &id : world_objects)
        // {
        //   auto obj = planning_scene_->getWorld()->getObject(id);
        //   RCLCPP_INFO(nh_->get_logger(), "Collision Object: %s", obj->id_.c_str());
        //   RCLCPP_INFO(nh_->get_logger(), "  - Type: %s", obj->shapes_.empty() ? "Unknown" : obj->shapes_[0]->type == shapes::SPHERE ? "Sphere"
        //                                                                                                                             : "Other");
        //   RCLCPP_INFO(nh_->get_logger(), "  - pose: [%.3f, %.3f, %.3f]", obj->pose_.translation().x(), obj->pose_.translation().y(), obj->pose_.translation().z());
        // }
        // 2. COLLISION CHECK
        collision_detection::CollisionResult col_res;
        planning_scene_->checkCollision(col_req, col_res, state, acm);

        // Set feasibility based on collision result
        r.feasibility = (col_res.collision) ? 0.0 : 1.0;

        if (col_res.collision)
        {
          if (req->detailed)
          {
            // If detailed is true, we can extract the exact names of the colliding objects/links
            for (const auto &contact : col_res.contacts)
            {

              RCLCPP_INFO(nh_->get_logger(), "[Configuration %zu Failed] Collision detected!", conf_idx);

              RCLCPP_INFO(nh_->get_logger(), "    -> Contact between: '%s' and '%s'", contact.first.first.c_str(),
                          contact.first.second.c_str());

              const auto object = planning_scene_->getWorld()->getObject("apple_183");
              if (object)
              {
                const auto &translation = object->pose_.translation();
                const auto &rotation = object->pose_.rotation();
                const Eigen::Quaterniond quaternion(rotation);
                RCLCPP_INFO(nh_->get_logger(), "apple_183: id='%s', shapes=%zu, shape_poses=%zu",
                            object->id_.c_str(), object->shapes_.size(), object->shape_poses_.size());
                RCLCPP_INFO(nh_->get_logger(),
                            "  pose: position=[%.6f, %.6f, %.6f], rotation=[%.6f, %.6f, %.6f, %.6f]",
                            translation.x(), translation.y(), translation.z(), quaternion.x(), quaternion.y(),
                            quaternion.z(), quaternion.w());

                for (std::size_t shape_idx = 0; shape_idx < object->shapes_.size(); ++shape_idx)
                {
                  const auto &shape = object->shapes_[shape_idx];
                  RCLCPP_INFO(nh_->get_logger(), "  shape[%zu]: type=%d", shape_idx,
                              shape ? static_cast<int>(shape->type) : -1);
                }
              }
              else
              {
                RCLCPP_WARN(nh_->get_logger(), "Collision object 'apple_183' was not found in the planning scene");
              }
            }
          }
        }
        else
        {
          // Optional: log success
          // RCLCPP_INFO(nh_->get_logger(), "[Configuration %zu] Valid and collision-free.", conf_idx);
        }

        // Populate detailed data if requested by the client
        if (req->detailed)
        {
          r.distance = col_res.distance;
          for (const auto &contact : col_res.contacts)
          {
            r.colliding_obj_first.push_back(contact.first.first);
            r.colliding_obj_second.push_back(contact.first.second);
          }
        }
      }

      // Add the collision result for the current configuration to the response
      res->result.push_back(r);
    }

    // Unlock the scene mutex
    scene_mtx_.unlock();
    is_processing_request_ = false;
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