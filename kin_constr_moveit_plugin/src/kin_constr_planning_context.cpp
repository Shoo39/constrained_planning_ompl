#include "kin_constr_planning_context.h"

namespace kin_constr
{
KinConstrPlanningContext::KinConstrPlanningContext(const std::string& name, const std::string& group,
                                           moveit::core::RobotModelConstPtr robot_model)
  : PlanningContext(name, group), robot_model_(robot_model), joint_model_group_(robot_model->getJointModelGroup(group)),
    delta_(0.05), initial_state_(robot_model)
{
  num_dofs_ = robot_model->getJointModelGroup(group)->getVariableCount();
  constraint_function_type_ = ErrorFunctionType::VECTOR;
  constraint_adherence_method_ = ConstraintAdherenceMethod::PROJECTED;
  initial_state_.update();
  anchoring_needed_ = false;
}

void KinConstrPlanningContext::clear()
{
}

//TODO: Get right behavior for invalid interpolation
bool KinConstrPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  planning_interface::MotionPlanDetailedResponse res_detailed;
  bool success = solve(res_detailed);

  res.error_code_ = res_detailed.error_code_;

  if(res_detailed.trajectory_.size()+res_detailed.processing_time_.size() > 0){
    res.trajectory_ = res_detailed.trajectory_[0];
    res.planning_time_ = res_detailed.processing_time_[0];
  }

  return success;
}

bool KinConstrPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  if(!preSolve(robot_model_, joint_model_group_->getName(), getPlanningScene(), request_, res)){
    return false;
  }

  double allowed_planning_time = request_.allowed_planning_time;
  if (allowed_planning_time == 0.0)
  {
    ROS_INFO_STREAM("Setting allowed planning time to default value of 5 seconds.");
    allowed_planning_time = 5.0;
  }
  
  // Try solving it
  simple_setup_->setup();
  ob::PlannerStatus stat = simple_setup_->solve(allowed_planning_time);
  if (stat == ob::PlannerStatus::EXACT_SOLUTION)
  {
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    bool post_processing = postSolve(res);

    res.trajectory_.resize(1);
    res.processing_time_.resize(1);
    res.trajectory_[0] = createRobotTrajectoryFromSolution(solution_path_);
    res.processing_time_[0] = simple_setup_->getLastPlanComputationTime();

    if (!post_processing){
      return false;
    }

    return true;
  }
  else if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION){
    OMPL_WARN("No exact solution found!");
    ROS_ERROR_STREAM("Error: " << stat);
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
    return false;
  }
  else{
    OMPL_WARN("Planning failed!");
    ROS_ERROR_STREAM("Error: " << stat);
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
}

bool KinConstrPlanningContext::terminate()
{
  return true;
}

bool KinConstrPlanningContext::preSolve(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                            const planning_scene::PlanningSceneConstPtr& ps,
                            planning_interface::MotionPlanRequest request,
                            planning_interface::MotionPlanDetailedResponse& response)
{
  const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(group); 
  num_dofs_ = joint_model_group->getVariableCount();

  state_space_ = std::make_shared<ob::RealVectorStateSpace>(num_dofs_);
  
  /* Set joint position limits as space bounds */
  ob::RealVectorBounds bounds(num_dofs_);
  moveit::core::JointBoundsVector joint_bounds =  joint_model_group->getActiveJointModelsBounds();
  moveit::core::JointBoundsVector::iterator b_it;
  
  int index = 0;
  for (b_it=joint_bounds.begin(); b_it!=joint_bounds.end(); b_it++){
    auto b_it2=(*b_it)->begin();

    bounds.setLow(index, b_it2->min_position_);
    bounds.setHigh(index, b_it2->max_position_);
    index++;
  }
  state_space_->setBounds(bounds);

  /* Applying configuration settings */
  std::map<std::string, std::string> cfg = config_.config;
  applyConfig(cfg);

  /* Create custom constraints in form of (F(q)=0) */
  constraints_ = createConstraint(robot_model, group, request.path_constraints, constraint_function_type_);
  if (!constraints_)
  {
    ROS_ERROR_STREAM("Failed to create constraints");
    response.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }

  /* Set start and goal joint configurations */
  Eigen::VectorXd start_joint_positions(num_dofs_);
  Eigen::VectorXd goal_joint_positions(num_dofs_);
  bool goal_joints_specified = false;

  robot_state::RobotState start_state(robot_model);

  // No initial start joint state is specified
  if (!request.start_state.is_diff && request.start_state.joint_state.name.empty()) {
    ROS_INFO_STREAM("Using current state as start state.");
    initial_state_ = ps->getCurrentState();
  }
  // Read start state from planning request
  else {
    moveit::core::robotStateMsgToRobotState(request.start_state, initial_state_);
  }

  // In case the initial start state deviates only slightly from a constraint satisfying state (e.g. due to bad position controllers),
  // try to correct the state and begin planning from the corrected start state. 
  Eigen::VectorXd initial_joint_positions;
  initial_state_.copyJointGroupPositions(joint_model_group, initial_joint_positions);
  ROS_INFO_STREAM("Initial start joint positions: " <<  initial_joint_positions.transpose());

  start_state = initial_state_;
  if (!constraints_->correctRobotState(start_state)){
    ROS_ERROR_STREAM("No constraint satisfying state could be found near the current robot state.");
    response.error_code_.val = moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS;
    return false;
  }

  start_state.copyJointGroupPositions(joint_model_group, start_joint_positions);
  ROS_INFO_STREAM("Start joint positions used for planning: " <<  start_joint_positions.transpose());

  Eigen::VectorXd diff = initial_joint_positions - start_joint_positions;
  double max_diff = diff.cwiseAbs().maxCoeff();

  if (max_diff > 0.02){
    ROS_ERROR_STREAM("The current robot state deviates too much from a constraint satisfying state. " << 
    "Move robot (close) to a constraint satisfying configuration first.");
    response.error_code_.val = moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS;
    return false;
  }

  // Goal joint state is given
  if (!request.goal_constraints[0].joint_constraints.empty()) {
    // Extract goal from planning request
    std::size_t joint_index{ 0 };
    for (auto& joint_constraint : request.goal_constraints[0].joint_constraints)
    {
      goal_joint_positions[joint_index] = joint_constraint.position;
      joint_index++;
    }
    goal_joints_specified = true;

    // Check if goal state satisfies constraints
    const double constraint_tolerance{ constraints_->getTolerance() };
    Eigen::VectorXd constraint_violation{ constraints_->getCoDimension() };

    constraints_->function(goal_joint_positions, constraint_violation);
    
    for (Eigen::Index dim{ 0 }; dim < constraint_violation.size(); ++dim)
    {
      if (std::abs(constraint_violation[dim]) > constraint_tolerance)
      {
        ROS_ERROR_STREAM("Goal constraints violated along the path for dimension " << dim << ". Value: " << constraint_violation[dim]);
        response.error_code_.val = moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS;
        return false;
      }
    }
  }
  
  /* Create appropriate constrained state space */
  if (constraint_adherence_method_ == ConstraintAdherenceMethod::ATLAS){
    ROS_INFO_STREAM("Using constrained planning method: Atlas State Space");
    constrained_state_space_ = std::make_shared<ob::AtlasStateSpace>(state_space_, constraints_);
    anchoring_needed_ = true;
    // std::shared_ptr<ob::AtlasStateSpace> css = std::make_shared<ob::AtlasStateSpace>(state_space_, constraints_);
    // std::cout << "Alpha: " << css->getAlpha() << ", Epsilon: " << css->getEpsilon() 
    //           << ", Rho: " << css->getRho() << std::endl;
    // constrained_state_space_ = css;
  }
  else if (constraint_adherence_method_ == ConstraintAdherenceMethod::TANGENT_BUNDLE){
    ROS_INFO_STREAM("Using constrained planning method: Tangent Bundle State Space");
    constrained_state_space_ = std::make_shared<ob::TangentBundleStateSpace>(state_space_, constraints_);

    anchoring_needed_ = true;
  }
  else{
    ROS_INFO_STREAM("Using default constrained planning method: Projected State Space");
    constrained_state_space_ = std::make_shared<ob::ProjectedStateSpace>(state_space_, constraints_);
  }

  // For Atlas and TangentBundle, the start and goal states have to be anchored. 
  ob::ScopedState<> start(constrained_state_space_);
  ob::ScopedState<> goal(constrained_state_space_);

  start->as<ob::ConstrainedStateSpace::StateType>()->copy(start_joint_positions);
  if (goal_joints_specified){
    goal->as<ob::ConstrainedStateSpace::StateType>()->copy(goal_joint_positions); 
  }
  if (anchoring_needed_) {
    constrained_state_space_->as<ob::AtlasStateSpace>()->anchorChart(start.get());
    if (goal_joints_specified){
      constrained_state_space_->as<ob::AtlasStateSpace>()->anchorChart(goal.get());
    }   
  }
  
  /* Set delta parameter */
  constrained_state_space_->setDelta(delta_);
  ROS_INFO_STREAM("Delta: " << constrained_state_space_->getDelta() << ", Lambda: " << constrained_state_space_->getLambda());

  /* Configure constrained state space */ 
  constrained_state_space_info_ = std::make_shared<ob::ConstrainedSpaceInformation>(constrained_state_space_);

  simple_setup_ = std::make_shared<og::SimpleSetup>(constrained_state_space_info_);
  simple_setup_->setStateValidityChecker(
      std::make_shared<MoveItStateValidityChecker>(constrained_state_space_info_, robot_model, group, ps));

  // Set planner
  planner_ = selectAndCreatePlanner(type_, constrained_state_space_info_);
  simple_setup_->setPlanner(planner_);
  ROS_INFO_STREAM("Planner configuration '" << getGroupName() << "' will use planner '" << type_ << "'. Additional configuration parameters will be set when planner is constructed.");
  
  if(!simple_setup_->getStateValidityChecker()->isValid(start.get())){
    ROS_ERROR_STREAM("Current start state is considered as invalid. Try to move to a valid start state again.");
    response.error_code_.val = moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS;
    return false;
  }
  simple_setup_->setStartState(start);

  // Rest of the parameters are for the planner itself
  simple_setup_->getSpaceInformation()->setup();
  simple_setup_->getSpaceInformation()->params().setParams(cfg, true);
  simple_setup_->getSpaceInformation()->setup();

  if (!goal_joints_specified){
    if(!setGoalConstraints(request.goal_constraints, request.path_constraints, anchoring_needed_)){
      response.error_code_.val = moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS;
      return false;
    }
  }
  else{
    simple_setup_->setGoalState(goal);
  }
  
  return true;
}

bool KinConstrPlanningContext::postSolve(planning_interface::MotionPlanDetailedResponse& res)
{
  //simple_setup_->simplifySolution(5.);
  auto path = simple_setup_->getSolutionPath();
  path.interpolate();

  ROS_INFO_STREAM("Writing path from OMPL to generic format.");

  // Write path to generic format indepenent from OMPL to pass it to ROS?
  solution_path_.clear();

  Eigen::VectorXd start_joint_positions;
  initial_state_.copyJointGroupPositions(joint_model_group_, start_joint_positions);
  solution_path_.push_back(start_joint_positions);

  for (auto& state : path.getStates())
  {
    const Eigen::Map<Eigen::VectorXd>& x = *state->as<ob::ConstrainedStateSpace::StateType>();

    Eigen::VectorXd joint_position(x);
    solution_path_.push_back(joint_position);
  }

  bool is_path_valid = checkSolution();
  if(!is_path_valid){
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
  }
  ROS_INFO_STREAM("Is OMPL interpolation valid? " << (is_path_valid ? "yes" : "no"));

  return is_path_valid;
}

bool KinConstrPlanningContext::checkSolution()
{
  std::cout << "Checking solution " << std::endl;
  
  if (solution_path_.size() < 2)
  {
    ROS_ERROR_STREAM("There is no solution path to check.");
    return false;
  }

  const double constraint_tolerance{ constraints_->getTolerance() };
  Eigen::VectorXd constraint_violation{ constraints_->getCoDimension() };

  int index = 0;
  bool result = true;

  collision_detection::CollisionResult res;
  moveit::core::RobotState robot_state(robot_model_);
  collision_detection::CollisionRequest collision_request_simple_;

  Eigen::VectorXd q;
  Eigen::VectorXd q_new = solution_path_[0];

  for (int i=1; i<solution_path_.size(); i++)
  {
    q = solution_path_[i];
    constraints_->function(q, constraint_violation);
    Eigen::VectorXd error_vec = constraints_->calcError(q);

    //ROS_INFO_STREAM("Index: " << i << ", Error: " << error_vec.transpose() << ", Joint diff: " << (q-q_new).norm());

    q_new = q;
  
    robot_state.setJointGroupPositions(joint_model_group_->getName(), q);
    robot_state.update();
    res.clear();
    getPlanningScene()->checkCollision(collision_request_simple_, res, robot_state);

    if (res.collision){
      ROS_ERROR_STREAM("Collision detected at index " << index << ". Contacts count: " << res.contact_count);
      result &= false;
    }
    if (!robot_state.satisfiesBounds(joint_model_group_)){
      // Joint bounds are violated due to a tiny numerical error
      if (robot_state.satisfiesBounds(joint_model_group_, 1e-10)){
        robot_state.enforceBounds();
        robot_state.copyJointGroupPositions(joint_model_group_, solution_path_[i]);
      }
      else{
        ROS_ERROR_STREAM("Bounds violation detected at index " << index);
        result &= false;
      }
    }
    
    for (Eigen::Index dim{ 0 }; dim < constraint_violation.size(); ++dim)
    {
      if (std::abs(constraint_violation[dim]) > constraint_tolerance)
      {
        ROS_ERROR_STREAM("Constraints violated along the path for dimension " << dim << " at index " << index << " at state: ");
        ROS_INFO_STREAM(constraint_violation.transpose());
        result &= false;
        break;
      }
    }
    index++;
  }
  ROS_INFO("Finished checking solution.");
  return result;
}

robot_trajectory::RobotTrajectoryPtr
KinConstrPlanningContext::createRobotTrajectoryFromSolution(std::vector<Eigen::VectorXd> path)
{
  if (path.size() == 0)
  {
    ROS_ERROR_STREAM("Cannot create robot trajectory from empty path.");
  }

  auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, request_.group_name);

  for (std::size_t path_index = 0; path_index < path.size(); ++path_index)
  {
    size_t joint_index = 0;
    auto state = std::make_shared<moveit::core::RobotState>(planning_scene_->getCurrentState());
    for (const moveit::core::JointModel* jm : trajectory->getGroup()->getActiveJointModels())
    {
      assert(jm->getVariableCount() == 1);
      state->setVariablePosition(jm->getFirstVariableIndex(), path[path_index][joint_index++]);
    }
    trajectory->addSuffixWayPoint(state, 0.0);
  }
  return trajectory;
}

void KinConstrPlanningContext::setConfig(const planning_interface::PlannerConfigurationSettings& config)
{
  config_ = config;

  if (config_.config.empty()){
    ROS_WARN_STREAM("PlannerConfigurationSettings is empty!");
  }
  else{
    ROS_INFO_STREAM("Name: " << config_.name << ", group: " << config.group);
  }
}

void KinConstrPlanningContext::applyConfig(std::map<std::string, std::string>& cfg)
{
  /* Set constraint function type */
  auto it = cfg.find("constraint_function_type");

  if (it==cfg.end()){
    ROS_WARN_STREAM("Constraint function type not specified. Using default: Vector");
  }
  else{
    std::string type = it->second;
    cfg.erase(it);
    if (type.compare("ScalarWithTolLin")==0){
      constraint_function_type_ = ErrorFunctionType::SCALAR_WITH_TOL_LIN;
    }
    else if (type.compare("Scalar")==0){
      constraint_function_type_ = ErrorFunctionType::SCALAR;
    }
    else if (type.compare("VectorWithTolLin")==0){
      constraint_function_type_ = ErrorFunctionType::VECTOR_WITH_TOL_LIN;
    }
  }

  /* Check which constraint adherence method is chosen. For Atlas and TangentBundle, it is recommended to anchor start and goal states. */
  it = cfg.find("constraint_adherence_method");
  if (it == cfg.end()){
    ROS_WARN_STREAM("Constraint adherence method is not specified. Using default: ProjectedStateSpace");
  }
  else {
    std::string method = it->second;
    cfg.erase(it);

    if (method.compare("AtlasStateSpace")==0) {
      ROS_INFO_STREAM("Using constrained planning method: Atlas State Space");
      constraint_adherence_method_ = ConstraintAdherenceMethod::ATLAS;
    }
    else if (method.compare("TangentBundleStateSpace")==0) {
      ROS_INFO_STREAM("Using constrained planning method: Tangent Bundle State Space");
      constraint_adherence_method_ = ConstraintAdherenceMethod::TANGENT_BUNDLE;
    }
    else{
      ROS_INFO_STREAM("Using default constrained planning method: Projected State Space");
    }
  }

  /* Set the delta parameter */
  it = cfg.find("delta");
  if (it != cfg.end()){
    delta_ = moveit::core::toDouble(it->second);
    cfg.erase(it);
  }

  /* Select planner type*/
  it = cfg.find("type");
  if (it == cfg.end()){
    ROS_WARN_STREAM("Attribute 'type' not specified in planner configuration: " << getGroupName());
  }
  else{
    type_ = it->second;
    cfg.erase(it);
  }

  /* Set the distance between waypoints when interpolating and collision checking. */
  it = cfg.find("longest_valid_segment_fraction");
  if (it != cfg.end()){
    double longest_valid_segment_fraction_config = moveit::core::toDouble(it->second);
    cfg["longest_valid_segment_fraction"] = moveit::core::toString(longest_valid_segment_fraction_config);
  }

  /* Set optimizing objective */
  std::string optimizer;
  ompl::base::OptimizationObjectivePtr objective;
  it = cfg.find("optimization_objective");
  if (it != cfg.end())
  {
    optimizer = it->second;
    cfg.erase(it);

    if (optimizer == "PathLengthOptimizationObjective")
      objective.reset(new ompl::base::PathLengthOptimizationObjective(simple_setup_->getSpaceInformation()));
    else if (optimizer == "MinimaxObjective")
      objective.reset(new ompl::base::MinimaxObjective(simple_setup_->getSpaceInformation()));
    else if (optimizer == "StateCostIntegralObjective")
      objective.reset(new ompl::base::StateCostIntegralObjective(simple_setup_->getSpaceInformation()));
    else if (optimizer == "MechanicalWorkOptimizationObjective")
      objective.reset(new ompl::base::MechanicalWorkOptimizationObjective(simple_setup_->getSpaceInformation()));
    else if (optimizer == "MaximizeMinClearanceObjective")
      objective.reset(new ompl::base::MaximizeMinClearanceObjective(simple_setup_->getSpaceInformation()));
    else
      objective.reset(new ompl::base::PathLengthOptimizationObjective(simple_setup_->getSpaceInformation()));

    simple_setup_->setOptimizationObjective(objective);
  }
}

bool KinConstrPlanningContext::setGoalConstraints(const std::vector<moveit_msgs::Constraints>& goal_constraints, 
                                                  const moveit_msgs::Constraints& path_constraints, bool atlas_used)
{
  ob::GoalPtr final_goal;
  std::vector<ob::GoalPtr> goals;
  for (const moveit_msgs::Constraints& goal_constraint : goal_constraints){
    ob::GoalPtr goal = ob::GoalPtr(new KinConstrainedGoalSampler(this, goal_constraint, path_constraints, 
                                                                constraints_, atlas_used, joint_model_group_));
    if(goal->as<KinConstrainedGoalSampler>()->hasStates()){
      goals.push_back(goal);
    }                                                       
  }

  if (!goals.empty()){
    if (goals.size() == 1){
      final_goal = goals[0];
    }
    //TODO: Not sure if this option works
    else{
      final_goal = ob::GoalPtr(new ompl_interface::GoalSampleableRegionMux(goals));
    }
  }
  else{
    ROS_ERROR("Unable to construct goals");
    return false;
  }
  simple_setup_->setGoal(final_goal);

  return static_cast<bool>(final_goal);
}

og::SimpleSetupPtr& KinConstrPlanningContext::getOMPLSimpleSetup()
{
  return simple_setup_;
}

const og::SimpleSetupPtr& KinConstrPlanningContext::getOMPLSimpleSetup() const
{
  return simple_setup_;
}

void KinConstrPlanningContext::copyToOMPLState(ob::State* state, const moveit::core::RobotState& rstate) const
{
  Eigen::VectorXd&& q = *state->as<ob::ConstrainedStateSpace::StateType>();
  rstate.copyJointGroupPositions(joint_model_group_, q);

  state->as<ob::ConstrainedStateSpace::StateType>()->copy(q);
}

void KinConstrPlanningContext::copyToRobotState(ob::State* state, moveit::core::RobotState& rstate) const
{
  Eigen::VectorXd&& q = *state->as<ob::ConstrainedStateSpace::StateType>();
  rstate.setJointGroupPositions(joint_model_group_, q);
  rstate.update();
}

// Return the OMPL planner pointer corresponding to planner id string
ob::PlannerPtr KinConstrPlanningContext::selectAndCreatePlanner(const std::string& planner_id,
                                                    ob::ConstrainedSpaceInformationPtr space_info) const
{
  if (planner_id == "geometric::AnytimePathShortening")
    return std::make_shared<og::AnytimePathShortening>(space_info);

  else if (planner_id == "geometric::BFMT")
    return std::make_shared<og::BFMT>(space_info);

  else if (planner_id == "geometric::BiEST")
    return std::make_shared<og::BiEST>(space_info);

  else if (planner_id == "geometric::BiTRRT")
    return std::make_shared<og::BiTRRT>(space_info);

  else if (planner_id == "geometric::BKPIECE")
    return std::make_shared<og::BKPIECE1>(space_info);

  else if (planner_id == "geometric::EST")
    return std::make_shared<og::EST>(space_info);
  
  else if (planner_id == "geometric::FMT")
    return std::make_shared<og::FMT>(space_info);
  
  else if (planner_id == "geometric::KPIECE")
    return std::make_shared<og::KPIECE1>(space_info);
  
  else if (planner_id == "geometric::LazyPRM")
    return std::make_shared<og::LazyPRM>(space_info);
  
  else if (planner_id == "geometric::LazyPRMstar")
    return std::make_shared<og::LazyPRMstar>(space_info);
  
  else if (planner_id == "geometric::LazyRRT")
    return std::make_shared<og::LazyRRT>(space_info);
  
  else if (planner_id == "geometric::LBKPIECE")
    return std::make_shared<og::LBKPIECE1>(space_info);
  
  else if (planner_id == "geometric::LBTRRT")
    return std::make_shared<og::LBTRRT>(space_info);
  
  else if (planner_id == "geometric::PDST")
    return std::make_shared<og::PDST>(space_info);
  
  else if (planner_id == "geometric::PRM")
    return std::make_shared<og::PRM>(space_info);
  
  else if (planner_id == "geometric::PRMstar")
    return std::make_shared<og::PRMstar>(space_info);
  
  else if (planner_id == "geometric::ProjEST")
    return std::make_shared<og::ProjEST>(space_info);

  else if (planner_id == "geometric::RRT")
    return std::make_shared<og::RRT>(space_info);
  
  else if (planner_id == "geometric::RRTConnect")
    return std::make_shared<og::RRTConnect>(space_info);
  
  else if (planner_id == "geometric::RRTstar")
    return std::make_shared<og::RRTstar>(space_info);

  else if (planner_id == "geometric::SBL")
    return std::make_shared<og::SBL>(space_info);

  else if (planner_id == "geometric::SPARS")
    return std::make_shared<og::SPARS>(space_info);
  
  else if (planner_id == "geometric::SPARStwo")
    return std::make_shared<og::SPARStwo>(space_info);
  
  else if (planner_id == "geometric::STRIDE")
    return std::make_shared<og::STRIDE>(space_info);
  
  else if (planner_id == "geometric::TRRT")
    return std::make_shared<og::TRRT>(space_info);
    
  else
  {
    ROS_ERROR_STREAM("Unkown planner id: " << planner_id);
    return nullptr;
  }
}
}  // namespace kin_constr
