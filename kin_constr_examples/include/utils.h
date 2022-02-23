#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <cmath>

namespace rvt = rviz_visual_tools;

void set_start_configuration(XmlRpc::XmlRpcValue& start_configuration_list, planning_interface::MotionPlanRequest& req, 
                            const moveit::core::JointModelGroup* joint_model_group, robot_state::RobotState state, 
                            moveit_visual_tools::MoveItVisualTools& visual_tools, planning_scene_monitor::PlanningSceneMonitorPtr psm);

void set_goal_configuration(XmlRpc::XmlRpcValue& goal_configuration_list, planning_interface::MotionPlanRequest& req, 
                            const moveit::core::JointModelGroup* joint_model_group, robot_state::RobotState state, 
                            moveit_visual_tools::MoveItVisualTools& visual_tools, planning_scene_monitor::PlanningSceneMonitorPtr psm);

void get_collision_objects(XmlRpc::XmlRpcValue& obstacles_list, std::vector<moveit_msgs::CollisionObject>& collision_objects);

void get_attached_objects(XmlRpc::XmlRpcValue& attached_objects_list, std::vector<moveit_msgs::AttachedCollisionObject>& attached_objects);

robot_trajectory::RobotTrajectoryPtr repairTrajectory(robot_trajectory::RobotTrajectoryPtr rt);

robot_trajectory::RobotTrajectoryPtr createEquidistantTimeTrajectory(robot_trajectory::RobotTrajectoryPtr rt, double dt);

void downSamplingTimeTrajectory(robot_trajectory::RobotTrajectoryPtr rt, int factor);

bool getJacobianDerivative(moveit::core::RobotState& robot_state, const moveit::core::JointModelGroup* group, 
                        const moveit::core::LinkModel* link, const Eigen::Vector3d& reference_point_position, Eigen::MatrixXd& J_dot,
                        Eigen::VectorXd& q_dot);

Eigen::MatrixXd getJacobianDerivative(moveit::core::RobotState& robot_state, const moveit::core::JointModelGroup* group,
                                        const Eigen::Vector3d& reference_point_position, Eigen::VectorXd& q_dot);

Eigen::Matrix3d skewMatrix(const Eigen::Ref<const Eigen::Vector3d>& vec);

Eigen::VectorXd calcError(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::Isometry3d& FK_start,
                            const Eigen::Isometry3d& FK_goal);

Eigen::VectorXd calcErrorRPY(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::Isometry3d& FK_start,
                            const Eigen::Isometry3d& FK_goal);

Eigen::VectorXd calcErrorAngleAxis(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::Isometry3d& FK_start,
                            const Eigen::Isometry3d& FK_goal);

Eigen::MatrixXd quatJacobian(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::VectorXd &joint_values,
                            const Eigen::Isometry3d& FK_goal, const Eigen::Isometry3d& T_offset);

Eigen::MatrixXd analyticalJacobian(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::VectorXd &joint_values,
                            const Eigen::Isometry3d& FK_goal, const Eigen::Isometry3d& T_offset);

Eigen::MatrixXd angleAxisJacobian(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::VectorXd &joint_values,
                            const Eigen::Isometry3d& FK_goal, const Eigen::Isometry3d& T_offset);

void save_trajectory(moveit_msgs::RobotTrajectory trajectory_msg, std::string filepath);
