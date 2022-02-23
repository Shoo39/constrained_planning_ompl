#pragma once

#include <memory>

#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/lexical_casts.h>
#include <moveit/ompl_interface/detail/constrained_sampler.h>
#include <moveit/ompl_interface/detail/goal_union.h>
#include <moveit/robot_model/joint_model.h>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/Planner.h>
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>

#include <ompl/base/goals/GoalStates.h>

#include "state_validity_checker.h"
#include "kin_constraint.h"
#include "kin_constr_goal_sampler.h"

namespace kin_constr
{
namespace ob = ompl::base;
namespace og = ompl::geometric;

// Macro that forward declares a class and defines the respective smartpointers through MOVEIT_DECLARE_PTR.
MOVEIT_CLASS_FORWARD(KinConstrPlanningContext);

// Subclass of PlanningContext, which is given by MoveIt library
class KinConstrPlanningContext : public planning_interface::PlanningContext
{
public:
  KinConstrPlanningContext(const std::string& name, const std::string& group, moveit::core::RobotModelConstPtr robot_model);

  ~KinConstrPlanningContext() = default;

  /* Parent functions which have to be overriden */
  void clear() override;

  bool solve(planning_interface::MotionPlanResponse& res) override;

  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;

  /** \brief Creation of OMPL state space, simple setup and planner based on request.
   * **/
  bool preSolve(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                const planning_scene::PlanningSceneConstPtr& ps, planning_interface::MotionPlanRequest request,
                planning_interface::MotionPlanDetailedResponse& response);

  /** \brief Simplify solution and convert from OMPL to generic format.
   **/
  bool postSolve(planning_interface::MotionPlanDetailedResponse& res);

  /** \brief Check if the constraints are satisfied along the path
   * `solution_path_`. **/
  bool checkSolution();

  /** \brief Convert the vector of joint values to a MoveIt RobotTrajectory.
   * **/
  robot_trajectory::RobotTrajectoryPtr createRobotTrajectoryFromSolution(std::vector<Eigen::VectorXd> path);

  void setConfig(const planning_interface::PlannerConfigurationSettings& config);

  void applyConfig(std::map<std::string, std::string>& cfg);

  bool setGoalConstraints(const std::vector<moveit_msgs::Constraints>& goal_constraints, const moveit_msgs::Constraints& path_constraints, bool atlas_used);

  og::SimpleSetupPtr& getOMPLSimpleSetup();

  const og::SimpleSetupPtr& getOMPLSimpleSetup() const;

  void copyToOMPLState(ob::State* state, const moveit::core::RobotState& rstate) const;

  void copyToRobotState(ob::State* state, moveit::core::RobotState& rstate) const;

  const moveit::core::RobotState& getInitialRobotState() const
  {
    return initial_state_;
  }

private:
  /** \brief Create OMPL planner based on the name of the planner (planner_id).
   * **/
  ob::PlannerPtr selectAndCreatePlanner(const std::string& planner_id,
                                        ob::ConstrainedSpaceInformationPtr space_info) const;

  std::shared_ptr<ob::RealVectorStateSpace> state_space_;
  std::shared_ptr<KinConstraint> constraints_;

  std::shared_ptr<ob::ConstrainedStateSpace> constrained_state_space_;
  std::shared_ptr<ob::ConstrainedSpaceInformation> constrained_state_space_info_;
  std::shared_ptr<og::SimpleSetup> simple_setup_;
  std::shared_ptr<ob::Planner> planner_;

  std::vector<Eigen::VectorXd> solution_path_;
  int constraint_function_type_;
  bool anchoring_needed_;
  int constraint_adherence_method_;
  std::string type_;

  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_; 
  std::size_t num_dofs_;
  double delta_;

  planning_interface::PlannerConfigurationSettings config_;

  robot_state::RobotState initial_state_;
};
}  // namespace kin_constr_plugin