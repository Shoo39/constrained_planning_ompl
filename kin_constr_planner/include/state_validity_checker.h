#pragma once

#include <string>

#include <threadsafe_state_storage.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

namespace kin_constr
{
namespace ob = ompl::base;

/** \brief Interface to collision checking in planning scene as an OMPL State Validity Checker
 * */
class MoveItStateValidityChecker : public ob::StateValidityChecker
{
public:
  MoveItStateValidityChecker(const ob::SpaceInformationPtr& si, robot_model::RobotModelConstPtr& robot_model,
                         const std::string& group, const planning_scene::PlanningSceneConstPtr& ps);
  /// Actual state validation function 
  virtual bool isValid(const ob::State* state) const;

private:
  std::string group_name_;
  TSStateStorage tss_;      /// OMPL requires the StateValidityChecker to be thread-safe.
  const planning_scene::PlanningSceneConstPtr& ps_;
  collision_detection::CollisionRequest collision_request_simple_;
};
}  // namespace kin_constr