#pragma once

#include <ompl/base/goals/GoalLazySamples.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>

#include <eigen_conversions/eigen_msg.h>

#include <utility>
#include <random>

#include "kin_constr_planning_context.h"
#include "kin_constraint.h"

namespace kin_constr
{
namespace ob = ompl::base;
namespace og = ompl::geometric;

class KinConstrPlanningContext;

class KinConstrainedGoalSampler : public ob::GoalStates
{
public:
  KinConstrainedGoalSampler(const KinConstrPlanningContext* pc,
                            const moveit_msgs::Constraints& goal_constraint, 
                            const moveit_msgs::Constraints& path_constraints, 
                            KinConstraintPtr constraints, bool atlas_used,
                            const moveit::core::JointModelGroup* joint_model_group,
                            int max_goal_samples = 10);
private:
  void sampleStates();

  const moveit_msgs::Constraints goal_constraint_;
  const moveit_msgs::Constraints path_constraints_;
  const KinConstrPlanningContext* planning_context_;
  robot_state::RobotState robot_state_;
  std::shared_ptr<KinConstraint> constraints_;
  bool atlas_used_;
  int max_goal_samples_;
  const moveit::core::JointModelGroup* joint_model_group_;
};

typedef std::shared_ptr<KinConstrainedGoalSampler> KinConstrainedGoalSamplerPtr;
}  // namespace kin_constr