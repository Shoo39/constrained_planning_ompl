#pragma once

#include <memory>

#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>

#include <ros/ros.h>

#include "kin_constr_planning_context.h"

namespace kin_constr
{

// Macro that forward declares a class and defines the respective smartpointers through MOVEIT_DECLARE_PTR.
MOVEIT_CLASS_FORWARD(KinConstrOMPLInterface);

class KinConstrOMPLInterface
{
public:
  KinConstrOMPLInterface(const moveit::core::RobotModelConstPtr& robot_model, const ros::NodeHandle& nh = ros::NodeHandle("~"));

  bool loadPlannerConfiguration(const std::string& group_name, const std::string& planner_id,
                                const std::map<std::string, std::string>& group_params,
                                planning_interface::PlannerConfigurationSettings& planner_config);
  
  void loadPlannerConfigurations();

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig);

  const planning_interface::PlannerConfigurationMap& getPlannerConfigurations() const;

  KinConstrPlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::MotionPlanRequest& req,
                                                  moveit_msgs::MoveItErrorCodes& error_code) const;

private:
  ros::NodeHandle nh_;

  robot_model::RobotModelConstPtr robot_model_;

  planning_interface::PlannerConfigurationMap planner_configs_;

};
}  // namespace kin_constr
