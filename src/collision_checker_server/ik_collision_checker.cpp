#include <collision_checker_server/ik_collision_checker.h>

// This code is part of the ik_solver namespace
namespace ik_solver
{
// The MoveCollisionChecker class is defined here
MoveCollisionChecker::MoveCollisionChecker(const ros::NodeHandle& nh) : robot_model_loader_("robot_description")
{
  // The NodeHandle object is assigned to the nh_ member variable
  nh_ = nh;

  // The group_name_ member variable is assigned the value of the "group_name" parameter
  // If the parameter is not specified, an exception is thrown
  if (!nh_.getParam("group_name", group_name_))
  {
    throw std::invalid_argument(nh_.getNamespace() + "/group_name is not specified");
  }

  // The planning_scene_ member variable is assigned a new PlanningScene object
  // The PlanningScene object is initialized with the model obtained from the robot_model_loader_
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_loader_.getModel());

  // The current state of the robot is obtained from the planning_scene_
  moveit::core::RobotState state = planning_scene_->getCurrentState();

  // The names of the active joints in the group are obtained
  std::vector<std::string> jmg_names = state.getJointModelGroup(group_name_)->getActiveJointModelNames();

  // For each joint in the group, its name and index are logged
  for (const std::string& jname : jmg_names)
  {
    ROS_INFO("joint %s, index = %d", jname.c_str(),
             state.getJointModelGroup(group_name_)->getJointModel(jname)->getFirstVariableIndex());
  }

  // The names of all the joints in the group are obtained
  std::vector<std::string> full_jmg_names = state.getJointModelGroup(group_name_)->getJointModelNames();

  // For each joint in the group, its name, index, and the index of its mimic joint are logged
  for (const std::string& jname : full_jmg_names)
  {
    ROS_INFO("joint %s, index = %d, %d", jname.c_str(),
             state.getJointModelGroup(group_name_)->getJointModel(jname)->getFirstVariableIndex(),
             state.getJointModelGroup(group_name_)->getJointModel(jname)->getMimic()->getJointIndex());
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
  server_=nh_.advertiseService("collision_check",&MoveCollisionChecker::collisionCheck,this);
}

// This is the collisionCheck function
// It performs collision checking for a given set of joint configurations
bool MoveCollisionChecker::collisionCheck(ik_solver_msgs::CollisionChecking::Request &req, ik_solver_msgs::CollisionChecking::Response &res)
{
  // Update the planning scene with the provided scene
  updatePlanningScene(req.planning_scene);

  // Get the current state of the robot
  moveit::core::RobotState state = planning_scene_->getCurrentState();

  // Get the names of the active joints in the group
  std::vector<std::string> jmg_names = state.getJointModelGroup(group_name_)->getActiveJointModelNames();

  // Check if the number of joint names match the number of active joints in the group
  if (jmg_names.size() != req.joint_names.size())
  {
    ROS_WARN("Joint names do not match");
    ROS_WARN("Request:");
    for (const std::string& s : req.joint_names)
      ROS_WARN("- %s", s.c_str());
    ROS_WARN("Joint model group %s:", group_name_.c_str());
    for (const std::string& s : jmg_names)
      ROS_WARN("- %s", s.c_str());
    ROS_WARN("RESORTING IS NOT IMPLEMENTED YET");
    return false;
  }

  // Set up collision request parameters
  collision_detection::CollisionRequest col_req;
  col_req.contacts = req.detailed;
  col_req.distance = req.detailed;
  col_req.max_contacts = req.detailed ? 10 : 0;
  col_req.group_name = group_name_;

  // Lock the scene mutex to ensure thread safety
  scene_mtx_.lock();

  // Iterate over each provided joint configuration
  for (ik_solver_msgs::Configuration& c : req.solutions)
  {
    ik_solver_msgs::CollisionResult r;
    r.solution = c;

    // Set the joint positions in the robot state
    for (size_t ij = 0; ij < req.joint_names.size(); ij++)
      state.setJointPositions(req.joint_names.at(ij), &c.configuration.at(ij));

    // Check if the joint positions satisfy the joint bounds
    if (!state.satisfiesBounds(state.getJointModelGroup(group_name_)))
    {
      // If any joint is out of bounds, add it to the out_of_bound_links list
      for (const std::string& jname : jmg_names)
      {
        if (!state.satisfiesPositionBounds(state.getJointModelGroup(group_name_)->getJointModel(jname)))
        {
          r.out_of_bound_links.push_back(jname);
          ROS_DEBUG("joint %s is out of bound. value =%f", jname.c_str());
        }
      }
      r.feasibility = 0.0;
      r.out_of_bound = true;
    }
    else
    {
      r.out_of_bound = false;
      state.updateCollisionBodyTransforms();

      // Perform collision checking
      collision_detection::CollisionResult col_res;
      planning_scene_->checkCollision(col_req, col_res, state);

      // Set the feasibility based on collision result
      r.feasibility = (col_res.collision) ? 0.0 : 1.0;

      // If detailed collision information is requested, populate the result
      if (req.detailed)
      {
        r.distance = col_res.distance;
        for (const std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact>>& contact : col_res.contacts)
        {
          r.colliding_obj_first.push_back(contact.first.first);
          r.colliding_obj_second.push_back(contact.first.second);
        }
      }
    }

    // Add the collision result to the response
    res.result.push_back(r);
  }

  // Unlock the scene mutex
  scene_mtx_.unlock();

  return true;
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
void MoveCollisionChecker::updatePlanningScene(const moveit_msgs::PlanningScene& scene)
{

  scene_mtx_.lock();
  if (!planning_scene_->setPlanningSceneMsg(scene))
  {
    ROS_ERROR("Unable to update the Planning Scene");
  }
  scene_mtx_.unlock();
}

}  // namespace ik_solver
