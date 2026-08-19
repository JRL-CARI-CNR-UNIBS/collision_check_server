#include <collision_checker_server/ik_collision_checker.hpp>

// This code is part of the ik_solver namespace
namespace ik_solver
{
  // The MoveCollisionChecker class is defined here
  MoveCollisionChecker::MoveCollisionChecker(const rclcpp::Node::SharedPtr &nh) : robot_model_loader_(nh, "robot_description")
  {
    // The NodeHandle object is assigned to the nh_ member variable
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

    // The planning_scene_ member variable is assigned a new PlanningScene object
    // The PlanningScene object is initialized with the model obtained from the robot_model_loader_
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_loader_.getModel());

    // The current state of the robot is obtained from the planning_scene_
    moveit::core::RobotState state = planning_scene_->getCurrentState();

    // The names of the active joints in the group are obtained
    std::vector<std::string> jmg_names = state.getJointModelGroup(group_name_)->getActiveJointModelNames();

    // For each joint in the group, its name and index are logged
    for (const std::string &jname : jmg_names)
    {
      RCLCPP_INFO(nh_->get_logger(), "joint %s, index = %d", jname.c_str(),
                  state.getJointModelGroup(group_name_)->getJointModel(jname)->getFirstVariableIndex());
    }

    // The names of all the joints in the group are obtained
    std::vector<std::string> full_jmg_names = state.getJointModelGroup(group_name_)->getJointModelNames();

    // For each joint in the group, its name, index, and the index of its mimic joint are logged
    for (const std::string &jname : full_jmg_names)
    {
      const auto *joint_model = state.getJointModelGroup(group_name_)->getJointModel(jname);

      if (joint_model->getMimic())
      {
        RCLCPP_INFO(nh_->get_logger(), "joint %s, index = %zu, mimic index = %zu", jname.c_str(),
                    joint_model->getFirstVariableIndex(),
                    joint_model->getMimic()->getJointIndex());
      }
      else
      {
        RCLCPP_INFO(nh_->get_logger(), "joint %s, index = %zu", jname.c_str(),
                    joint_model->getFirstVariableIndex());
      }
    }
  }
  // The destructor of the MoveCollisionChecker class
  // It resets the planning_scene_ member variable
  MoveCollisionChecker::~MoveCollisionChecker()
  {
    planning_scene_.reset();
  }

  void MoveCollisionChecker::init()
  {
    server_ = nh_->create_service<ik_solver_msgs::srv::CollisionChecking>(
        "collision_check",
        std::bind(&MoveCollisionChecker::collisionCheck, this, std::placeholders::_1, std::placeholders::_2));
  }

  // This is the collisionCheck function
  // It performs collision checking for a given set of joint configurations
  void MoveCollisionChecker::collisionCheck(const std::shared_ptr<ik_solver_msgs::srv::CollisionChecking::Request> req,
                                            std::shared_ptr<ik_solver_msgs::srv::CollisionChecking::Response> res)
  {
    // Update the planning scene with the provided scene
    // updatePlanningScene(req->planning_scene);

    // Get the current state of the robot
    moveit::core::RobotState state = planning_scene_->getCurrentState();

    // Get the names of the active joints in the group
    std::vector<std::string> jmg_names = state.getJointModelGroup(group_name_)->getActiveJointModelNames();

    // Check if the number of joint names match the number of active joints in the group
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
      return;
    }

    // Set up collision request parameters
    collision_detection::CollisionRequest col_req;
    col_req.contacts = req->detailed;
    col_req.distance = req->detailed;
    col_req.max_contacts = req->detailed ? 10 : 0;
    col_req.group_name = group_name_;

    // Lock the scene mutex to ensure thread safety
    scene_mtx_.lock();

    // Iterate over each provided joint configuration
    for (size_t conf_idx = 0; conf_idx < req->solutions.size(); ++conf_idx)
    {
      auto &c = req->solutions[conf_idx];
      ik_solver_msgs::msg::CollisionResult r;
      r.solution = c;

      // Set the joint positions in the robot state
      for (size_t ij = 0; ij < req->joint_names.size(); ij++)
        state.setJointPositions(req->joint_names.at(ij), &c.configuration.at(ij));

      // 1. CONTROLLO LIMITI DEI GIUNTI (OUT OF BOUNDS)
      if (!state.satisfiesBounds(state.getJointModelGroup(group_name_)))
      {
        r.feasibility = 0.0;
        r.out_of_bound = true;

        for (const std::string &jname : jmg_names)
        {
          if (!state.satisfiesPositionBounds(state.getJointModelGroup(group_name_)->getJointModel(jname)))
          {
            r.out_of_bound_links.push_back(jname);

            // Estraggo il valore effettivo del giunto per stamparlo
            const double *joint_value = state.getJointPositions(jname);
            RCLCPP_INFO(nh_->get_logger(),
                        "[Configurazione %zu Fallita] Giunto fuori dai limiti: '%s' = %.3f",
                        conf_idx, jname.c_str(), joint_value[0]);
          }
        }
      }
      else
      {
        r.out_of_bound = false;
        state.updateCollisionBodyTransforms();
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

        // 2. CONTROLLO DELLE COLLISIONI
        collision_detection::CollisionResult col_res;

        planning_scene_->checkCollision(col_req, col_res, copied_state, acm);

        r.feasibility = (col_res.collision) ? 0.0 : 1.0;

        if (col_res.collision)
        {
          RCLCPP_INFO(nh_->get_logger(), "[Configurazione %zu Fallita] Rilevata Collisione!", conf_idx);

          if (req->detailed)
          {
            // Se detailed è true, possiamo estrarre i nomi esatti degli oggetti/link che collidono
            for (const auto &contact : col_res.contacts)
            {
              RCLCPP_INFO(nh_->get_logger(), "    -> Contatto tra: '%s' e '%s'",
                          contact.first.first.c_str(),
                          contact.first.second.c_str());
            }
          }
          else
          {
            RCLCPP_INFO(nh_->get_logger(), "    -> (Imposta req->detailed = true nel client per vedere i link in collisione)");
          }
        }
        else
        {
          // Opzionale: stampa di successo
          // RCLCPP_INFO(nh_->get_logger(), "[Configurazione %zu] Valida e collision-free.", conf_idx);
        }

        // Popolamento dei dati se richiesto dal client
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

      // Add the collision result to the response
      res->result.push_back(r);
    }

    // Unlock the scene mutex
    scene_mtx_.unlock();
  }

  // This function updates the planning scene with the provided scene
  /**
   * @brief Updates the planning scene with the given moveit_msgs::PlanningScene.
   *
   * This function locks the scene mutex, sets the planning scene using the provided message,
   * and then unlocks the mutex. If the planning scene cannot be set, an error message is printed.
   *
   * @param scene The moveit_msgs::PlanningScene to update the planning scene with.
   */
  void MoveCollisionChecker::updatePlanningScene(const moveit_msgs::msg::PlanningScene &scene)
  {

    scene_mtx_.lock();
    // RCLCPP_INFO(nh_->get_logger(), "Updating Planning Scene with new data...");
    // RCLCPP_INFO(nh_->get_logger(), "Planning Scene Message Allowed Collision Matrix size: %zu entries", scene.allowed_collision_matrix.entry_names.size());
    if (!planning_scene_->setPlanningSceneDiffMsg(scene))
    {
      RCLCPP_ERROR(nh_->get_logger(), "Unable to update the Planning Scene");
    }
    // else
    // {
    //   RCLCPP_INFO(nh_->get_logger(), "Planning Scene updated successfully.");
    //   RCLCPP_INFO(nh_->get_logger(), "Planning Scene Allowed Collision Matrix size after update: %zu entries", planning_scene_->getAllowedCollisionMatrix().getSize());
    // }
    scene_mtx_.unlock();
  }

} // namespace ik_solver