/*
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <collision_checker_server/ik_collision_checker.h>

namespace ik_solver
{

MoveCollisionChecker::MoveCollisionChecker(const ros::NodeHandle& nh)
{
  nh_=nh;

  if (!nh_.getParam("group_name",group_name_))
  {
    throw std::invalid_argument(nh_.getNamespace()+"/group_name is not specified");
  }
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  planning_scene_=std::make_shared<planning_scene::PlanningScene>(robot_model_loader.getModel());

  moveit::core::RobotState state=planning_scene_->getCurrentState();
  std::vector<std::string> jmg_names=state.getJointModelGroup(group_name_)->getActiveJointModelNames();

  for (const std::string& jname: jmg_names)
  {
    ROS_INFO("joint %s, index = %d",jname.c_str(),state.getJointModelGroup(group_name_)->getJointModel(jname)->getFirstVariableIndex());
  }

  std::vector<std::string> full_jmg_names=state.getJointModelGroup(group_name_)->getJointModelNames();
  for (const std::string& jname: full_jmg_names)
  {
    ROS_INFO("joint %s, index = %d, %d",jname.c_str(),
             state.getJointModelGroup(group_name_)->getJointModel(jname)->getFirstVariableIndex(),
             state.getJointModelGroup(group_name_)->getJointModel(jname)->getMimic()
             );
  }
}
void MoveCollisionChecker::init()
{
  server_=nh_.advertiseService("collision_check",&MoveCollisionChecker::collisionCheck,this);
}

bool MoveCollisionChecker::collisionCheck(ik_solver_msgs::CollisionChecking::Request &req, ik_solver_msgs::CollisionChecking::Response &res)
{
  updatePlanningScene(req.planning_scene);
  moveit::core::RobotState state=planning_scene_->getCurrentState();
  std::vector<std::string> jmg_names=state.getJointModelGroup(group_name_)->getActiveJointModelNames();
  if (jmg_names.size()!=req.joint_names.size())
  {
    ROS_WARN("Joint names do not match");
    ROS_WARN("Request:");
    for (const std::string& s: req.joint_names)
      ROS_WARN("- %s",s.c_str());
    ROS_WARN("Joint model group %s:",group_name_.c_str());
    for (const std::string& s: jmg_names)
      ROS_WARN("- %s",s.c_str());

    ROS_WARN("RESORTING IS NOT IMPLEMENTED YET");
    return false;
  }


  collision_detection::CollisionRequest col_req;
  col_req.contacts = req.detailed;
  col_req.distance = req.detailed;
  col_req.max_contacts = req.detailed? 10: 0;
  col_req.group_name=group_name_;

  scene_mtx_.lock();
  for (ik_solver_msgs::Configuration& c: req.solutions)
  {
    ik_solver_msgs::CollisionResult r;
    r.solution=c;

    for (size_t ij=0;ij<req.joint_names.size();ij++)
      state.setJointPositions(req.joint_names.at(ij),&c.configuration.at(ij));

    if (!state.satisfiesBounds(state.getJointModelGroup(group_name_)))
    {
      for (const std::string& jname: jmg_names)
      {
        if (!state.satisfiesPositionBounds(state.getJointModelGroup(group_name_)->getJointModel(jname)))
        {
          r.out_of_bound_links.push_back(jname);
          ROS_DEBUG("joint %s is out of bound. value =%f",jname.c_str());
        }
      }
      r.feasibility=0.0;
      r.out_of_bound=true;
    }
    else
    {
      r.out_of_bound=false;
      state.updateCollisionBodyTransforms();


      collision_detection::CollisionResult col_res;
      planning_scene_->checkCollision(col_req,col_res,state);

      r.feasibility=(col_res.collision)?0.0:1.0;

      if (req.detailed)
      {
        r.distance= col_res.distance;
        for (const  std::pair<std::pair<std::string, std::string>, std::vector<collision_detection::Contact> >& contact: col_res.contacts)
        {
          r.colliding_obj_first.push_back(contact.first.first);
          r.colliding_obj_second.push_back(contact.first.second);
        }
      }
    }


    res.result.push_back(r);
  }
  scene_mtx_.unlock();

  return true;
}

void MoveCollisionChecker::updatePlanningScene(const moveit_msgs::PlanningScene& scene)
{
  scene_mtx_.lock();
  if (!planning_scene_->setPlanningSceneMsg(scene))
    ROS_ERROR("unable to update planning scene");
  scene_mtx_.unlock();
}




}  // end namespace ik_solver
