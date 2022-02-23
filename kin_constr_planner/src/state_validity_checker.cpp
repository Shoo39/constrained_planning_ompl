#include "state_validity_checker.h"

namespace kin_constr
{
MoveItStateValidityChecker::MoveItStateValidityChecker(const ob::SpaceInformationPtr& si,
                                               robot_model::RobotModelConstPtr& robot_model, const std::string& group,
                                               const planning_scene::PlanningSceneConstPtr& ps)
  : ob::StateValidityChecker(si), group_name_(group), tss_(robot_model), ps_(ps)
{
  collision_request_simple_.group_name = group_name_;
}

bool MoveItStateValidityChecker::isValid(const ob::State* state) const
{
  auto&& q = *state->as<ompl::base::ConstrainedStateSpace::StateType>();
  collision_detection::CollisionResult res;

  robot_state::RobotState robot_state = ps_->getCurrentState();
  robot_state.setJointGroupPositions(group_name_, q);
  robot_state.update();
  ps_->checkCollision(collision_request_simple_, res, robot_state);
  return (!res.collision && robot_state.satisfiesBounds());

  // moveit::core::RobotState* robot_state = tss_.getStateStorage();
  // robot_state->setJointGroupPositions(group_name_, q);
  // robot_state->update();
  // ps_->checkCollision(collision_request_simple_, res, *robot_state);
  // return (!res.collision && robot_state->satisfiesBounds());
}
}  // namespace kin_constr