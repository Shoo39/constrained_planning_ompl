#include "kin_constr_goal_sampler.h"

namespace kin_constr
{
KinConstrainedGoalSampler::KinConstrainedGoalSampler(const KinConstrPlanningContext* pc,
                                                    const moveit_msgs::Constraints& goal_constraint, 
                                                    const moveit_msgs::Constraints& path_constraints, 
                                                    KinConstraintPtr constraints, bool atlas_used,
                                                    const moveit::core::JointModelGroup* joint_model_group,
                                                    int max_goal_samples)
    :   ob::GoalStates(pc->getOMPLSimpleSetup()->getSpaceInformation()),
        goal_constraint_(goal_constraint),
        path_constraints_(path_constraints),
        constraints_(constraints),
        planning_context_(pc),
        robot_state_(pc->getInitialRobotState()),
        atlas_used_(atlas_used),
        joint_model_group_(joint_model_group),
        max_goal_samples_(max_goal_samples)
{
    sampleStates();
}

void KinConstrainedGoalSampler::sampleStates()
{
    ob::ScopedState<> goal_state(si_->getStateSpace()); 
    
    // Get target goal position 
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = goal_constraint_.position_constraints[0].constraint_region.primitive_poses[0].position.x;
    goal_pose.position.y = goal_constraint_.position_constraints[0].constraint_region.primitive_poses[0].position.y;
    goal_pose.position.z = goal_constraint_.position_constraints[0].constraint_region.primitive_poses[0].position.z;

    goal_pose.orientation.w = goal_constraint_.orientation_constraints[0].orientation.w;
    goal_pose.orientation.x = goal_constraint_.orientation_constraints[0].orientation.x;
    goal_pose.orientation.y = goal_constraint_.orientation_constraints[0].orientation.y;
    goal_pose.orientation.z = goal_constraint_.orientation_constraints[0].orientation.z;

    // Get goal position tolerance
    Eigen::Vector3d position_tolerance;
    position_tolerance[0] = goal_constraint_.position_constraints[0].constraint_region.primitives[0].dimensions[0];
    position_tolerance[1] = goal_constraint_.position_constraints[0].constraint_region.primitives[0].dimensions[1];
    position_tolerance[2] = goal_constraint_.position_constraints[0].constraint_region.primitives[0].dimensions[2];

    Eigen::Vector3d orientation_tolerance;
    orientation_tolerance[0] = goal_constraint_.orientation_constraints[0].absolute_x_axis_tolerance;
    orientation_tolerance[1] = goal_constraint_.orientation_constraints[0].absolute_y_axis_tolerance;
    orientation_tolerance[2] = goal_constraint_.orientation_constraints[0].absolute_z_axis_tolerance;

    // Get offset transformation
    Eigen::Isometry3d T_offset = Eigen::Translation3d(  path_constraints_.position_constraints[0].target_point_offset.x,
                                                        path_constraints_.position_constraints[0].target_point_offset.y,
                                                        path_constraints_.position_constraints[0].target_point_offset.z ) *
                                Eigen::Quaterniond( path_constraints_.orientation_constraints[0].orientation.w,
                                                    path_constraints_.orientation_constraints[0].orientation.x,
                                                    path_constraints_.orientation_constraints[0].orientation.y,
                                                    path_constraints_.orientation_constraints[0].orientation.z );  
                                                    
    Eigen::Isometry3d T_target;
    tf::poseMsgToEigen(goal_pose, T_target);

    T_target = T_target * T_offset;

    // Check if goal state satisfies constraints
    if(!constraints_->validatePose(T_target)){
        ROS_ERROR("Given goal pose does not satisfy the constraints!");
        return;
    }            

    bool success = true;
    int goal_counter = 0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(-1.0, 1.0);

    for (int i=0; i < 100; i++){
        if (goal_counter >= max_goal_samples_)  
            break;

        robot_state_.setToRandomPositions(joint_model_group_);

        // Create random displacement 
        Eigen::Isometry3d T_random_displacement = Eigen::Isometry3d(Eigen::AngleAxis<double>(dist(gen) * orientation_tolerance[2], Eigen::Vector3d::UnitZ()) *
                                                    Eigen::AngleAxis<double>(dist(gen) * orientation_tolerance[1], Eigen::Vector3d::UnitY()) *
                                                    Eigen::AngleAxis<double>(dist(gen) * orientation_tolerance[0], Eigen::Vector3d::UnitX()));    
        T_random_displacement.translation() = Eigen::Vector3d( dist(gen) * position_tolerance[0], 
                                                               dist(gen) * position_tolerance[1],
                                                               dist(gen) * position_tolerance[2]);     

        Eigen::Isometry3d T_target_with_random_disp = T_target * T_random_displacement;
        T_target_with_random_disp = T_target_with_random_disp * T_offset.inverse();
        tf::poseEigenToMsg(T_target_with_random_disp, goal_pose);                                       

        success = robot_state_.setFromIK(joint_model_group_, goal_pose);
        
        if (success){
            robot_state_.update();
            planning_context_->copyToOMPLState(goal_state.get(), robot_state_);
            success &= si_->getStateValidityChecker()->isValid(goal_state.get());

            if (success){
                Eigen::VectorXd&& goal_joint_positions = *goal_state.get()->as<ob::ConstrainedStateSpace::StateType>();
                robot_state_.copyJointGroupPositions(joint_model_group_, goal_joint_positions);

                if (atlas_used_){
                    ob::ScopedState<> goal(si_->getStateSpace());
                    goal->as<ob::ConstrainedStateSpace::StateType>()->copy(goal_joint_positions);
                    si_->getStateSpace()->as<ob::AtlasStateSpace>()->anchorChart(goal.get());
                }

                ROS_INFO_STREAM("Adding goal configuration: " << goal_joint_positions.transpose());
                
                goal_state.get()->as<ob::ConstrainedStateSpace::StateType>()->copy(goal_joint_positions);
                addState(goal_state.get());
                goal_counter++;
            }
        }
    }
}
}