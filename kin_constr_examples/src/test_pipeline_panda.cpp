#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>

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
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "kin_constraint.h"
#include "utils.h"


int main(int argc, char** argv)
{
    /* Initialize */
    ros::init(argc, argv, "test_pipeline_panda");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    // Get some parameters from parameter server
    XmlRpc::XmlRpcValue config_params;
    node_handle.getParam("kin_constr_planning_config/parameters", config_params);

    // Get robot information and create planning scene to monitor
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("/robot_description"));

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    psm->startSceneMonitor("/move_group/monitored_planning_scene");
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

    std::string group_name = std::string(config_params["group"]);
    moveit::planning_interface::MoveGroupInterface move_group(group_name);

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(std::string(config_params["group"]));


    /* Use planning pipeline */
    planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));
    // Path validation is done by the ompl interface, such that this flag can be set to false. Otherwise an error is displayed which however can be ignored. 
    planning_pipeline->checkSolutionPaths(false); 

    /* Visualization */
    std::string frame = std::string(config_params["fixed_frame"]);
    moveit_visual_tools::MoveItVisualTools visual_tools(frame, "/rviz_visual_tools");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.deleteAllMarkers();  // clear all old markers
    ros::Duration(1.0).sleep();
    visual_tools.trigger();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.2;
    visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE, true);
    ros::Duration(1.0).sleep();     // Without waiting text won't be recognized and therefore not displayed
    visual_tools.trigger();

    /* Add obstacles */
    XmlRpc::XmlRpcValue obstacles_list;
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    if(node_handle.getParam("obstacles", obstacles_list)){
        get_collision_objects(obstacles_list, collision_objects);
        planning_scene_interface.addCollisionObjects(collision_objects);
    }
    else{
        ROS_INFO_STREAM("No obstacles specified");
    }

    /* Add attached objects */
    XmlRpc::XmlRpcValue attached_objects_list;
    std::vector<moveit_msgs::AttachedCollisionObject> attached_objects;

    if (node_handle.getParam("attached_objects", attached_objects_list)){
        get_attached_objects(attached_objects_list, attached_objects);
        planning_scene_interface.applyAttachedCollisionObjects(attached_objects);
    }
    else{
        ROS_INFO_STREAM("No attached objects specified");
    }
    ros::Duration(5.0).sleep();

    /* Start planning */
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = std::string(config_params["group"]);
    req.max_velocity_scaling_factor = config_params["velocity_factor"];
    req.max_acceleration_scaling_factor = config_params["acceleration_factor"];
    req.allowed_planning_time = config_params["allowed_planning_time"];
    req.planner_id = std::string(config_params["planner_id"]);

    /* Create constraints */
    XmlRpc::XmlRpcValue constraint_list;
    node_handle.getParam("kin_constr_planning_config/constraint", constraint_list);
    moveit_msgs::Constraints path_constraints = kin_constr::KinConstraint::createKinConstraintMsg(constraint_list);
    req.path_constraints = path_constraints;  

    /* Get start and joint configuration for planning */
    XmlRpc::XmlRpcValue configuration_list;
    if (node_handle.getParam("kin_constr_planning_config/start_configuration", configuration_list)){
        set_start_configuration(configuration_list, req, joint_model_group, *robot_state, visual_tools, psm);
    }

    if (node_handle.getParam("kin_constr_planning_config/goal_configuration", configuration_list)){
        set_goal_configuration(configuration_list, req, joint_model_group, *robot_state, visual_tools, psm);
    }
    else {
        ROS_ERROR("No goal information specified. Planning can't be executed. Please fill in 'kin_constr_planning_config/goal_configuration'");
        return -1;
    }

    // Start planning
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    if(!planning_pipeline->generatePlan(lscene, req, res)){
        ROS_ERROR("Pipeline returns FAILURE");
    }
    else{
        ROS_INFO("Pipeline returns SUCCESS");
    }

    // Retrieve result
    if(res.error_code_.val != res.error_code_.SUCCESS){
        ROS_ERROR_STREAM("Could not compute plan successfully");
    }
    else{
        ROS_INFO_STREAM("Path found with length: " << res.trajectory_->getWayPointCount());  

        ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        moveit_msgs::DisplayTrajectory display_trajectory;

        moveit_msgs::MotionPlanResponse response;
        res.getMessage(response);

        display_trajectory.trajectory_start = response.trajectory_start;
        display_trajectory.trajectory.push_back(response.trajectory);

        display_publisher.publish(display_trajectory);
        
        visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
        visual_tools.trigger();

        Eigen::Isometry3d T_offset(Eigen::AngleAxis<double>(constraint_list["offset_pose"][5], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxis<double>(constraint_list["offset_pose"][4], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxis<double>(constraint_list["offset_pose"][3], Eigen::Vector3d::UnitX()));
        T_offset.translation() = Eigen::Vector3d(constraint_list["offset_pose"][0],
                                                    constraint_list["offset_pose"][1],
                                                    constraint_list["offset_pose"][2]);

        geometry_msgs::Pose pose_msg;
        Eigen::VectorXd prev_joint_positions;
        Eigen::VectorXd current_joint_positions;

        std::ofstream myfile;
        // myfile.open("/home/isshu/data.dat", std::ios::app);

        for (int k=0; k<res.trajectory_->getWayPointCount(); k++){

            Eigen::Isometry3d end_effector_transform = res.trajectory_->getWayPoint(k).getGlobalLinkTransform(std::string(constraint_list["link"])) * T_offset;
            Eigen::Vector3d pos = end_effector_transform.translation();

            robot_state::RobotState rs(res.trajectory_->getWayPoint(k));

            Eigen::VectorXd q;
            rs.copyJointGroupPositions(joint_model_group, q);
            Eigen::VectorXd q_dot;
            rs.copyJointGroupVelocities(joint_model_group, q_dot);
            Eigen::VectorXd q_ddot;
            rs.copyJointGroupAccelerations(joint_model_group, q_ddot);
            

            Eigen::MatrixXd J = rs.getJacobian(joint_model_group, T_offset.translation());
            Eigen::MatrixXd J_dot = getJacobianDerivative(rs, joint_model_group, T_offset.translation(), q_dot);

            Eigen::Vector3d vel = J.block<3, 7>(0,0) * q_dot;
            Eigen::Vector3d acc = J_dot.block<3,7>(0, 0) * q_dot + J.block<3, 7>(0,0) * q_ddot;

            double time = res.trajectory_->getWayPointDurationFromStart(k);
            
            myfile << time << " " << pos.transpose() << " " << vel.transpose() << " " << acc.transpose() << "\n";

            pose_msg = tf2::toMsg(end_effector_transform);
            visual_tools.publishAxis(pose_msg);
        }
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' ");
    }

    std::vector<std::string> object_ids;
    for(int i = 0; i < collision_objects.size(); i++){
        object_ids.push_back(collision_objects[i].id);
    }
    for(int i = 0; i < attached_objects.size(); i++){
        object_ids.push_back(attached_objects[i].object.id);
    }
    planning_scene_interface.removeCollisionObjects(object_ids);

    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();

    node_handle.deleteParam("");

    return 0;
}