#include "utils.h"

void set_start_configuration(XmlRpc::XmlRpcValue& start_configuration_list, planning_interface::MotionPlanRequest& req, 
                            const moveit::core::JointModelGroup* joint_model_group, robot_state::RobotState state, 
                            moveit_visual_tools::MoveItVisualTools& visual_tools, planning_scene_monitor::PlanningSceneMonitorPtr psm)
{
    // Are any predefined poses given?
    std::string pose_id = std::string(start_configuration_list["start_predefined_pose"]);
    int num_joints_specified = start_configuration_list["start_joint_positions"].size();

    if (!pose_id.empty()){
        ROS_INFO_STREAM("Predefined start pose selected: " << pose_id);
        if(!state.setToDefaultValues(joint_model_group, pose_id)){
            ROS_WARN_STREAM("No predefined pose named " << pose_id << " found. Setting to default pose.");
            state.setToDefaultValues();
        }
        moveit::core::robotStateToRobotStateMsg(state, req.start_state);
    }
    // Are explicit joint values given?
    else if (num_joints_specified == joint_model_group->getVariableCount()){
        Eigen::VectorXd start_joint_positions(num_joints_specified, 1);
        for (int i=0; i<num_joints_specified; i++){
            start_joint_positions[i] = start_configuration_list["start_joint_positions"][i];
        }
        state.setJointGroupPositions(joint_model_group, start_joint_positions);
        moveit::core::robotStateToRobotStateMsg(state, req.start_state);

        ROS_INFO_STREAM("Start joint values specified: " << start_joint_positions.transpose());
    }
    // Is a link pose given?
    else if (start_configuration_list["start_pose_xyzrpy"].size() == 6){
        // Get target pose 
        Eigen::Isometry3d T_start(Eigen::AngleAxis<double>(start_configuration_list["start_pose_xyzrpy"][5], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxis<double>(start_configuration_list["start_pose_xyzrpy"][4], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxis<double>(start_configuration_list["start_pose_xyzrpy"][3], Eigen::Vector3d::UnitX()));
        T_start.translation() = Eigen::Vector3d(start_configuration_list["start_pose_xyzrpy"][0],
                                start_configuration_list["start_pose_xyzrpy"][1],
                                start_configuration_list["start_pose_xyzrpy"][2]);

        geometry_msgs::PoseStamped start_pose;
        start_pose.header.frame_id = std::string(start_configuration_list["start_frame_id"]);
        tf::poseEigenToMsg(T_start, start_pose.pose);

        visual_tools.publishAxis(start_pose.pose);
        visual_tools.publishText(start_pose.pose, "Start pose", rvt::WHITE, rvt::XLARGE, true);
        visual_tools.trigger();

        // Get offset from link 
        Eigen::Isometry3d T_offset;
        if (start_configuration_list["start_pose_offset"].size() == 6){
            T_offset = Eigen::Isometry3d(Eigen::AngleAxis<double>(start_configuration_list["start_pose_offset"][5], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxis<double>(start_configuration_list["start_pose_offset"][4], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxis<double>(start_configuration_list["start_pose_offset"][3], Eigen::Vector3d::UnitX()));
            T_offset.translation() = Eigen::Vector3d(start_configuration_list["start_pose_offset"][0],
                                                    start_configuration_list["start_pose_offset"][1],
                                                    start_configuration_list["start_pose_offset"][2]);
        }
        else{
            T_offset = Eigen::Isometry3d::Identity();
        }

        T_start = T_start * T_offset.inverse();
        tf::poseEigenToMsg(T_start, start_pose.pose);

        // Start IK to find a configuration which leads to the desired pose
        // Check for validity (pose reached, collision) 
        collision_detection::CollisionRequest collision_request_simple;
        collision_detection::CollisionResult result;
           
        bool success = true;
        int counter = 0;
        do {
            state.setToRandomPositions(joint_model_group);
            state.update();
            success = state.setFromIK(joint_model_group, start_pose.pose);

            result.clear();
            psm->getPlanningScene()->checkCollision(collision_request_simple, result, state);

            counter++;
            if (counter>100){
                ROS_ERROR("No configuration for specified start pose found");
                return;
            }
        } while (!(success && !result.collision)); 

        Eigen::VectorXd start_joint_positions; 
        state.copyJointGroupPositions(joint_model_group, start_joint_positions);
        moveit::core::robotStateToRobotStateMsg(state, req.start_state);

        ROS_INFO_STREAM("Start joint configuration found by IK: " << start_joint_positions.transpose());
    }
    else{
        return;
    }

    // Visualize initial and goal configurations
    visual_tools.publishRobotState(state, rviz_visual_tools::GREEN);
    visual_tools.trigger();
    ros::Duration(1.5).sleep();
}

void set_goal_configuration(XmlRpc::XmlRpcValue& goal_configuration_list, planning_interface::MotionPlanRequest& req, 
                            const moveit::core::JointModelGroup* joint_model_group, robot_state::RobotState state, 
                            moveit_visual_tools::MoveItVisualTools& visual_tools, planning_scene_monitor::PlanningSceneMonitorPtr psm)
{
    // Are any predefined poses given?
    std::string pose_id = std::string(goal_configuration_list["goal_predefined_pose"]);
    int num_joints_specified = goal_configuration_list["goal_joint_positions"].size();

    if (!pose_id.empty()){
        ROS_INFO_STREAM("Predefined goal pose selected: " << pose_id);
        if(!state.setToDefaultValues(joint_model_group, pose_id)){
            ROS_WARN_STREAM("No predefined pose named " << pose_id << " found. Setting to default pose.");
            state.setToDefaultValues();
        }
        req.goal_constraints.clear();
        req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(state, joint_model_group)); 

        // Visualize initial and goal configurations
        visual_tools.publishRobotState(state, rviz_visual_tools::ORANGE);
        visual_tools.trigger();
    }
    // Are explicit joint values given?
    else if (num_joints_specified == joint_model_group->getVariableCount()){
        Eigen::VectorXd goal_joint_positions(num_joints_specified, 1);
        for (int i=0; i<num_joints_specified; i++){
            goal_joint_positions[i] = goal_configuration_list["goal_joint_positions"][i];
        }
        state.setJointGroupPositions(joint_model_group, goal_joint_positions.transpose());
        
        req.goal_constraints.clear();
        req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(state, joint_model_group));

        // Visualize initial and goal configurations
        visual_tools.publishRobotState(state, rviz_visual_tools::ORANGE);
        visual_tools.trigger();

        ROS_INFO_STREAM("Goal joint values specified: " << goal_joint_positions);
    }
    // Is a link pose given?
    else if (goal_configuration_list["goal_pose_xyzrpy"].size() == 6){
        // Get target pose 
        Eigen::Isometry3d T_goal(Eigen::AngleAxis<double>(goal_configuration_list["goal_pose_xyzrpy"][5], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxis<double>(goal_configuration_list["goal_pose_xyzrpy"][4], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxis<double>(goal_configuration_list["goal_pose_xyzrpy"][3], Eigen::Vector3d::UnitX()));
        T_goal.translation() = Eigen::Vector3d(goal_configuration_list["goal_pose_xyzrpy"][0],
                                                goal_configuration_list["goal_pose_xyzrpy"][1],
                                                goal_configuration_list["goal_pose_xyzrpy"][2]);

        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = std::string(goal_configuration_list["goal_frame_id"]);
        tf::poseEigenToMsg(T_goal, goal_pose.pose);

        // visual_tools.publishAxis(goal_pose.pose);
        // visual_tools.publishText(goal_pose.pose, "Goal pose", rvt::WHITE, rvt::XLARGE, true);
        // visual_tools.trigger();

        // Get offset from link 
        Eigen::Isometry3d T_offset;
        if (goal_configuration_list["goal_pose_offset"].size() == 6){
            T_offset = Eigen::Isometry3d(Eigen::AngleAxis<double>(goal_configuration_list["goal_pose_offset"][5], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxis<double>(goal_configuration_list["goal_pose_offset"][4], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxis<double>(goal_configuration_list["goal_pose_offset"][3], Eigen::Vector3d::UnitX()));
            T_offset.translation() = Eigen::Vector3d(goal_configuration_list["goal_pose_offset"][0],
                                                    goal_configuration_list["goal_pose_offset"][1],
                                                    goal_configuration_list["goal_pose_offset"][2]);
        }
        else{
            T_offset = Eigen::Isometry3d::Identity();
        }

        T_goal = T_goal * T_offset.inverse();
        tf::poseEigenToMsg(T_goal, goal_pose.pose);

        std::vector<double> tolerance_pos(3);
        std::vector<double> tolerance_angle(3);

        tolerance_pos[0] = goal_configuration_list["goal_pose_tolerance"][0];
        tolerance_pos[1] = goal_configuration_list["goal_pose_tolerance"][1];
        tolerance_pos[2] = goal_configuration_list["goal_pose_tolerance"][2];

        tolerance_angle[0] = goal_configuration_list["goal_pose_tolerance"][3];
        tolerance_angle[1] = goal_configuration_list["goal_pose_tolerance"][4];
        tolerance_angle[2] = goal_configuration_list["goal_pose_tolerance"][5];

        req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(std::string(goal_configuration_list["goal_link"]), 
                                                                                        goal_pose, tolerance_pos, tolerance_angle));

        ROS_INFO("Goal pose set. Configurations will be sampled later.");
    }
}

void get_collision_objects(XmlRpc::XmlRpcValue& obstacles_list, std::vector<moveit_msgs::CollisionObject>& collision_objects)
{
    collision_objects.clear();

    for(int i = 0; i < obstacles_list.size(); i++){
        moveit_msgs::CollisionObject collision_object;
        collision_object.id = std::string(obstacles_list[i]["name"]);
        collision_object.header.frame_id = std::string(obstacles_list[i]["frame_id"]);
        collision_object.operation = collision_object.ADD;

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        std::vector<double> dim{obstacles_list[i]["dimension"][0], 
                                obstacles_list[i]["dimension"][1], 
                                obstacles_list[i]["dimension"][2]};
        primitive.dimensions = dim;

        geometry_msgs::Pose pose;
        pose.position.x = obstacles_list[i]["position"][0];
        pose.position.y = obstacles_list[i]["position"][1];
        pose.position.z = obstacles_list[i]["position"][2];

        tf2::Quaternion quat;
        quat.setRPY(obstacles_list[i]["orientation"][0], 
                    obstacles_list[i]["orientation"][1], 
                    obstacles_list[i]["orientation"][2]);
        pose.orientation = tf2::toMsg(quat);

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);        

        collision_objects.push_back(collision_object);
    }
}

void get_attached_objects(XmlRpc::XmlRpcValue& attached_objects_list, std::vector<moveit_msgs::AttachedCollisionObject>& attached_objects)
{
    for(int i=0; i < attached_objects_list.size(); i++){

        moveit_msgs::AttachedCollisionObject attached_object;

        attached_object.object.id = std::string(attached_objects_list[i]["name"]);
        attached_object.link_name = std::string(attached_objects_list[i]["frame_id"]);
        attached_object.object.header.frame_id = std::string(attached_objects_list[i]["frame_id"]);

        geometry_msgs::Pose pose;
        pose.position.x = attached_objects_list[i]["position"][0];
        pose.position.y = attached_objects_list[i]["position"][1];
        pose.position.z = attached_objects_list[i]["position"][2];
        tf2::Quaternion quat;
        quat.setRPY(attached_objects_list[i]["orientation"][0],
                    attached_objects_list[i]["orientation"][1],
                    attached_objects_list[i]["orientation"][2]);
        pose.orientation = tf2::toMsg(quat);

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = attached_objects_list[i]["dimension"][0];
        primitive.dimensions[1] = attached_objects_list[i]["dimension"][1];
        primitive.dimensions[2] = attached_objects_list[i]["dimension"][2];

        attached_object.object.primitives.push_back(primitive);
        attached_object.object.primitive_poses.push_back(pose);
        attached_object.object.operation = attached_object.object.ADD;

        std::vector<std::string> touch_links;
        for (int j=0; j<attached_objects_list[i]["adjacent_links"].size(); j++){
            touch_links.push_back(std::string(attached_objects_list[i]["adjacent_links"][j]));
        }
        attached_object.touch_links = touch_links;
        attached_objects.push_back(attached_object);
    }
}

// Post processing often assigns the same timestamp to different waypoints. 
void repairTrajectory(moveit_msgs::RobotTrajectory& trajectory)
{
    ros::Duration last_element;

    for (auto it = trajectory.joint_trajectory.points.begin(); it!=trajectory.joint_trajectory.points.end(); ){
        
        if (it==trajectory.joint_trajectory.points.begin()){
            last_element = it->time_from_start;
            it++;
        }
        else{
            if (it->time_from_start==last_element){
                it = trajectory.joint_trajectory.points.erase(it);
                ROS_INFO_STREAM("Fault detected! Will be removed.");
            }
            else{
                last_element = it->time_from_start;
                it++;
            }
        }
    }
}

robot_trajectory::RobotTrajectoryPtr repairTrajectory(robot_trajectory::RobotTrajectoryPtr rt)
{
    double last_time = -1;
    double length = rt->getWayPointCount();

    auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(rt->getRobotModel(), rt->getGroupName());

    for (int i=0; i<length; i++){

        if (rt->getWayPointDurationFromStart(i) > (last_time+0.0001)){
            last_time = rt->getWayPointDurationFromStart(i); 
            double dt = rt->getWayPointDurationFromPrevious(i); 
            trajectory->addSuffixWayPoint(rt->getWayPoint(i), dt);
        }
    }
    return trajectory;
}

robot_trajectory::RobotTrajectoryPtr createEquidistantTimeTrajectory(robot_trajectory::RobotTrajectoryPtr rt, double dt)
{
    std::string group = rt->getGroupName();

    auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(rt->getRobotModel(), group);

    int num_points = rt->getWayPointCount();
    int num_joints = rt->getWayPoint(0).getVariableCount();

    auto jmg = rt->getWayPoint(0).getJointModelGroup(group);

    double last_time = -1;

    for (int i=1; i<num_points; i++){
        robot_state::RobotState rs1(rt->getWayPoint(i-1));
        robot_state::RobotState rs2(rt->getWayPoint(i));
        
        double t1 = rt->getWayPointDurationFromStart(i-1);
        double t2 = rt->getWayPointDurationFromStart(i);

        Eigen::VectorXd joint_pos1;
        Eigen::VectorXd joint_vel1;
        Eigen::VectorXd joint_acc1;
        rs1.copyJointGroupPositions(jmg, joint_pos1);
        rs1.copyJointGroupVelocities(jmg, joint_vel1);
        rs1.copyJointGroupAccelerations(jmg, joint_acc1);

        Eigen::VectorXd joint_pos2;
        Eigen::VectorXd joint_vel2;
        Eigen::VectorXd joint_acc2;
        rs2.copyJointGroupPositions(jmg, joint_pos2);
        rs2.copyJointGroupVelocities(jmg, joint_vel2);
        rs2.copyJointGroupAccelerations(jmg, joint_acc2);

        if ((i==1) && (t2/dt<1)){
            robot_state::RobotStatePtr rs = std::make_shared<moveit::core::RobotState>(rs1);
            trajectory->addSuffixWayPoint(rs, 0);
            last_time = 0;
        } 

        for (int j=t1/dt; j<t2/dt; j++){
            Eigen::VectorXd joint_pos(num_joints);
            Eigen::VectorXd joint_vel(num_joints);
            Eigen::VectorXd joint_acc(num_joints);
            double time = j * dt;

            if (time == last_time)
                continue;
            else{
                last_time = time;
            }

            for (int k=0; k<num_joints; k++){
                double a, b, c, d;  // f(t) = a*t^3+b*t^2+c*t+d

                Eigen::MatrixXd M(6, 4);
                M <<    pow(t1, 3), pow(t1, 2), t1, 1,
                        pow(t2, 3), pow(t2, 2), t2, 1,
                        3*pow(t1, 2), 2*t1, 1, 0,
                        3*pow(t2, 2), 2*t2, 1, 0,
                        6*t1, 2, 0, 0,
                        6*t2, 2, 0, 0;

                Eigen::VectorXd y(6);
                y << joint_pos1(k), joint_pos2(k), joint_vel1(k), joint_vel2(k), joint_acc1(k), joint_acc2(k);

                Eigen::VectorXd v = Eigen::VectorXd::Ones(6);
                Eigen::MatrixXd W = v.asDiagonal();
                // W(1,1) = 10;
                // W(2,2) = 10;

                Eigen::VectorXd x(4);
                x = (M.transpose() * W * M).inverse() * M.transpose() * W * y;
                a = x(0);
                b = x(1);
                c = x(2);
                d = x(3);
                
                // Eigen::MatrixXd A(2,2);
                // A << 6*t1, 2, 6*t2, 2;

                // Eigen::VectorXd y(2);
                // y << joint_acc1(k), joint_acc2(k);

                // Eigen::VectorXd x = A.inverse() * y;
                // a = x(0);
                // b = x(1);

                // c = joint_vel1(k) - 3 * a * pow(t1, 2)  - 2 * b * t1;

                // d = joint_pos1(k) - a * pow(t1, 3) - b * pow(t1, 2) - c * t1;

                joint_pos(k) = a * pow(time, 3) + b * pow(time, 2) + c * time + d;
                joint_vel(k) = 3 * a * pow(time, 2) + 2 * b * time + c;
                joint_acc(k) = 6 * a * time + 2 * b;
            }

            robot_state::RobotStatePtr state = std::make_shared<moveit::core::RobotState>(rs1);
            state->setJointGroupPositions(jmg, joint_pos);
            state->setJointGroupVelocities(jmg, joint_vel);
            state->setJointGroupAccelerations(jmg, joint_acc);
            state->update();

            trajectory->addSuffixWayPoint(state, dt);
        }
    }
    return trajectory;
}

void downSamplingTimeTrajectory(robot_trajectory::RobotTrajectoryPtr rt, int factor)
{
    std::string group = rt->getGroupName();

    auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(rt->getRobotModel(), group);

    int num_points = rt->getWayPointCount();

    auto jmg = rt->getWayPoint(0).getJointModelGroup(group);

    double dt = rt->getWayPointDurationFromStart(1);

    for (int i=0; i<num_points; i=i+factor){
        robot_state::RobotState rs(rt->getWayPoint(i));
        robot_state::RobotStatePtr state = std::make_shared<moveit::core::RobotState>(rs);

        Eigen::VectorXd joint_pos;
        Eigen::VectorXd joint_vel;
        Eigen::VectorXd joint_acc;
        
        state->setJointGroupPositions(jmg, joint_pos);
        state->setJointGroupVelocities(jmg, joint_vel);
        state->setJointGroupAccelerations(jmg, joint_acc);
        state->update();

        trajectory->addSuffixWayPoint(state, i*dt);
    }
    rt->clear();
    rt = trajectory;
}

bool getJacobianDerivative(moveit::core::RobotState& robot_state, const moveit::core::JointModelGroup* group, 
                        const moveit::core::LinkModel* link, const Eigen::Vector3d& reference_point_position, Eigen::MatrixXd& J_dot,
                        Eigen::VectorXd& q_dot)
{
    if (!group->isChain())
    {
        ROS_ERROR_STREAM("The group " << group->getName() << " is not a chain. Cannot compute Jacobian derivative.");
        return false;
    }
    if (!group->isLinkUpdated(link->getName()))
    {
        return false;
    }

    // Get Jacobian
    Eigen::MatrixXd J;
    robot_state.getJacobian(group, link, reference_point_position, J, false);

    const moveit::core::JointModel* root_joint_model = group->getJointModels()[0];
    const moveit::core::LinkModel* root_link_model = root_joint_model->getParentLinkModel();
    Eigen::Isometry3d reference_transform = root_link_model ? robot_state.getGlobalLinkTransform(root_link_model).inverse() : Eigen::Isometry3d::Identity();

    int rows = 6;
    int columns = group->getVariableCount();
    J_dot = Eigen::MatrixXd::Zero(rows, columns);   // 6 x n

    Eigen::Isometry3d link_transform = reference_transform * robot_state.getGlobalLinkTransform(link);
    Eigen::Vector3d point_transform = link_transform * reference_point_position;

    Eigen::Vector3d joint_axis;
    Eigen::Isometry3d joint_transform;

    // omega0_0 ...omega0_n-1
    Eigen::MatrixXd angular_velocities = Eigen::MatrixXd::Zero(3, columns); 
    Eigen::MatrixXd joint_axes = Eigen::MatrixXd::Zero(3, columns); 
    joint_axes.block<3, 1>(0, 0) = Eigen::Vector3d(0, 0, 1);

    // First loop to calculate angular velocities for each link.
    const moveit::core::LinkModel* current_link = link;
    while(current_link){
        const moveit::core::JointModel* pjm = current_link->getParentJointModel();
        if (pjm->getVariableCount() > 0){
            if (!group->hasJointModel(pjm->getName())){
                current_link = pjm->getParentLinkModel();
                continue;
            }
            unsigned int joint_index = group->getVariableGroupIndex(pjm->getName());
            joint_transform = reference_transform * robot_state.getGlobalLinkTransform(current_link);
            // z0_i-1 
            joint_axis = joint_transform.linear() * static_cast<const moveit::core::RevoluteJointModel*>(pjm)->getAxis();
            joint_axes.block<3, 1>(0, joint_index) = joint_axis;

            if (joint_index == (columns-1)){
                current_link = pjm->getParentLinkModel();
                continue;
            }
            else{
                if(pjm->getType() == moveit::core::JointModel::REVOLUTE){
                    Eigen::Vector3d delta_omega = joint_axis * q_dot(joint_index);
                    for (int i=joint_index+1; i<columns; i++){
                        angular_velocities.block<3, 1>(0, i) = angular_velocities.block<3, 1>(0, i) + delta_omega;
                    }
                }
            } 
        }
        if (pjm == root_joint_model)
            break;
        current_link = pjm->getParentLinkModel();
    }

    // Second loop to calculate Jacobian derivative
    current_link = link;
    Eigen::Vector3d beta = Eigen::Vector3d::Zero();

    bool correct_starting_joint_found = false;

    while(current_link){
        const moveit::core::JointModel* pjm = current_link->getParentJointModel();
        if (pjm->getVariableCount() > 0){
            if (!group->hasJointModel(pjm->getName())){
                current_link = pjm->getParentLinkModel();
                correct_starting_joint_found = false;
                continue;
            }
            else {
                unsigned int joint_index = group->getVariableGroupIndex(pjm->getName());

                if (joint_index==0){
                    if(pjm->getType() == moveit::core::JointModel::REVOLUTE)
                        J_dot.block<3,1>(0,0) = joint_axes.block<3, 1>(0,0).cross(beta);
                    break;
                }
                
                if (!correct_starting_joint_found){
                    beta = J.block<3,1>(0, joint_index) * q_dot(joint_index);
                    correct_starting_joint_found = true;
                }
                joint_transform = reference_transform * robot_state.getGlobalLinkTransform(current_link);
                joint_axis = joint_transform.linear() * static_cast<const moveit::core::RevoluteJointModel*>(pjm)->getAxis(); // z0_i-1 

                Eigen::Vector3d joint_axis_dot = angular_velocities.block<3, 1>(0, joint_index).cross(joint_axis); // d(z0_i-1)/dt

                Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
                for (int index = 0; index <= joint_index - 1; index++){
                    alpha += joint_axes.block<3, 1>(0, index).cross(point_transform - joint_transform.translation()) * q_dot(index);
                }

                if(pjm->getType() == moveit::core::JointModel::REVOLUTE){
                    J_dot.block<3, 1>(0, joint_index) = joint_axis_dot.cross(point_transform - joint_transform.translation()) + joint_axis.cross(alpha + beta);
                    J_dot.block<3, 1>(3, joint_index) = joint_axis_dot;
                }
                else if(pjm->getType() == moveit::core::JointModel::PRISMATIC){
                    J_dot.block<3, 1>(0, joint_index) = joint_axis_dot;
                    J_dot.block<3, 1>(3, joint_index) = Eigen::Vector3d::Zero();
                }
                else {
                    ROS_ERROR("Unknown type of joint in Jacobian computation");
                }

                beta += J.block<3,1>(0, joint_index-1) * q_dot(joint_index-1);
            }
        }
        if (pjm == root_joint_model)
            break;
        current_link = pjm->getParentLinkModel();
    }

    return true;
}

Eigen::MatrixXd getJacobianDerivative(moveit::core::RobotState& robot_state, const moveit::core::JointModelGroup* group,
                                        const Eigen::Vector3d& reference_point_position, Eigen::VectorXd& q_dot) 
{
  Eigen::MatrixXd result;
  if (!getJacobianDerivative(robot_state, group, group->getLinkModels().back(), reference_point_position, result, q_dot))
    throw moveit::Exception("Unable to compute Jacobian Derivative");
  return result;
}

Eigen::Matrix3d skewMatrix(const Eigen::Ref<const Eigen::Vector3d>& vec)
{
    Eigen::Matrix3d S;
    S <<    0,      -vec(2),    vec(1), 
            vec(2), 0,          -vec(0), 
            -vec(1), vec(0),    0;
    return S;
}

Eigen::Vector3d extractRPY(const Eigen::Ref<const Eigen::MatrixXd>& R){
    // Naming might be confusing, but since rotation angles around fixed x, y and z axis is needed,
    // the actual order is yaw, pitch, roll.
    Eigen::Vector3d rpy;    
    double yaw = atan2(R(2,1), R(2,2));
    double pitch = -asin(R(2,0));
    double roll = atan2(R(1,0), R(0,0));
    rpy << yaw, pitch, roll; 

    return rpy;
}

Eigen::VectorXd calcError(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::Isometry3d& FK_start,
                            const Eigen::Isometry3d& FK_goal)
{
    Eigen::VectorXd error(6,1);

    Eigen::Isometry3d Td_e = FK_goal.inverse() * FK_start;
    Eigen::Vector3d error_trans = Td_e.translation();
    // Eigen::Vector3d error_trans = FK_goal.translation()-FK_start.translation();

    Eigen::Quaterniond Q = Eigen::Quaterniond(Td_e.rotation());
    // Eigen::Quaterniond Q_e = Eigen::Quaterniond(FK_start.rotation());
    // Eigen::Quaterniond Q_d = Eigen::Quaterniond(FK_goal.rotation());

    Eigen::Vector3d error_rot = Q.vec();
    // Eigen::Vector3d error_rot = Q_e.w() * Q_d.vec() - Q_d.w() * Q_e.vec() - skewMatrix(Q_d.vec()) * Q_e.vec();
    error << error_trans, error_rot;
    return error;
} 

Eigen::VectorXd calcErrorRPY(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::Isometry3d& FK_start,
                            const Eigen::Isometry3d& FK_goal)
{
    Eigen::VectorXd error(6,1);

    Eigen::Isometry3d Td_e = FK_goal.inverse() * FK_start;

    Eigen::Vector3d error_trans = Td_e.translation();
    Eigen::Vector3d error_rot = extractRPY(Td_e.rotation());
    error << error_trans, error_rot;

    return error;
} 

Eigen::VectorXd calcErrorAngleAxis(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::Isometry3d& FK_start,
                            const Eigen::Isometry3d& FK_goal)
{
    Eigen::Vector3d error_trans = FK_goal.translation()-FK_start.translation();
    // Axis-Angle error
    Eigen::Matrix3d Rt = FK_goal.rotation();
    Eigen::Matrix3d Re = FK_start.rotation();

    Eigen::VectorXd error(6, 1);
    error << error_trans, 0.5*(Re.col(0).cross(Rt.col(0)) + Re.col(1).cross(Rt.col(1)) + Re.col(2).cross(Rt.col(2)));
    return error;
}

Eigen::MatrixXd quatJacobian(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::VectorXd &joint_values,
                            const Eigen::Isometry3d& FK_goal, const Eigen::Isometry3d& T_offset)
{
    std::string link_name = joint_model_group->getLinkModelNames().back();
    Eigen::VectorXd error(6,1);

    robot_state::RobotState start_state(robot_model);
    start_state.setJointGroupPositions(joint_model_group, joint_values);

    Eigen::Vector3d offset_translation = T_offset.translation();

    Eigen::Isometry3d FK_start = start_state.getGlobalLinkTransform(link_name) * T_offset;

    Eigen::Isometry3d Td_e = FK_goal.inverse() * FK_start;
    Eigen::Quaterniond Q = Eigen::Quaterniond(Td_e.rotation());

    Eigen::Matrix3d Rt_0 = FK_goal.rotation().transpose();

    // Transformation matrix to transform Jacobian from world to task frame:
    //  |Rt_0    0|
    //  |0    Rt_0|
    Eigen::MatrixXd M(2*Rt_0.rows(), 2*Rt_0.cols());
    M << Rt_0, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Rt_0;

    // Task frame Jacobian
    Eigen::MatrixXd Jt = M * start_state.getJacobian(joint_model_group, offset_translation); 

    Eigen::MatrixXd T(6,6);
    T << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), 0.5*(Q.w()*Eigen::Matrix3d::Identity()-skewMatrix(Q.vec())); 

    return T * Jt;

    // Eigen::Matrix3d Re_0 = FK_start.rotation().transpose();
    // Eigen::Quaterniond Q_e = Eigen::Quaterniond(FK_start.rotation());
    // Eigen::Matrix3d Rt_0 = FK_goal.rotation().transpose();
    // Eigen::Quaterniond Q_d = Eigen::Quaterniond(FK_goal.rotation());
    
    // Eigen::MatrixXd T(6,6);
    // T << -1*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), 
    //     -0.5 * Q_d.vec() * Q_e.vec().transpose() -Q_d.w() * 0.5*(Q_e.w()*Eigen::Matrix3d::Identity()-skewMatrix(Q_e.vec())) - skewMatrix(Q_d.vec()) * 0.5*(Q_e.w()*Eigen::Matrix3d::Identity()-skewMatrix(Q_e.vec())); 
    // return T * start_state.getJacobian(joint_model_group, offset_translation); 
}

Eigen::MatrixXd analyticalJacobian(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::VectorXd &joint_values,
                            const Eigen::Isometry3d& FK_goal, const Eigen::Isometry3d& T_offset)
{
    robot_state::RobotState start_state(robot_model);
    start_state.setJointGroupPositions(joint_model_group, joint_values);

    std::string link_name = joint_model_group->getLinkModelNames().back();
    Eigen::Isometry3d FK_start = start_state.getGlobalLinkTransform(link_name) * T_offset;

    Eigen::MatrixXd E_rpy(6, 6);
    Eigen::Vector3d rpy = extractRPY(FK_start.rotation());

    Eigen::Matrix3d R_trafo = Eigen::Matrix3d::Zero();
    R_trafo(0, 0) = cos(rpy[2]) / cos(rpy(1));
    R_trafo(0, 1) = sin(rpy[2]) / cos(rpy(1));
    
    R_trafo(1, 0) = -sin(rpy[2]);
    R_trafo(1, 1) = cos(rpy[2]);

    R_trafo(2, 0) = cos(rpy[2])*sin(rpy[1])/cos(rpy[1]);
    R_trafo(2, 1) = sin(rpy[2])*sin(rpy[1])/cos(rpy[1]);
    R_trafo(2, 2) = 1;

    E_rpy << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), R_trafo;

    Eigen::Matrix3d Rt_0 = FK_goal.rotation().transpose();

    // Transformation matrix to transform Jacobian from world to task frame:
    //  |Rt_0    0|
    //  |0    Rt_0|
    Eigen::MatrixXd M(2*Rt_0.rows(), 2*Rt_0.cols());
    M << Rt_0, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Rt_0;

    // Task frame Jacobian
    Eigen::Vector3d offset_translation = T_offset.translation();
    Eigen::MatrixXd Jt = M * start_state.getJacobian(joint_model_group, offset_translation); 

    // Analytical task frame Jacobian
    Eigen::MatrixXd J_a = E_rpy * Jt;

    return J_a;
}

Eigen::MatrixXd angleAxisJacobian(const robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup* joint_model_group,
                            const Eigen::VectorXd &joint_values,
                            const Eigen::Isometry3d& FK_goal, const Eigen::Isometry3d& T_offset)
{
    std::string link_name = joint_model_group->getLinkModelNames().back();
    robot_state::RobotState start_state(robot_model);
    start_state.setJointGroupPositions(joint_model_group, joint_values);

    Eigen::Isometry3d FK_start = start_state.getGlobalLinkTransform(link_name) * T_offset;

    Eigen::Matrix3d Rd = FK_goal.rotation();
    Eigen::Matrix3d Re = FK_start.rotation();

    Eigen::Matrix3d L = -0.5*(skewMatrix(Rd.col(0))*skewMatrix(Re.col(0)) + 
                                skewMatrix(Rd.col(1))*skewMatrix(Re.col(1))+ 
                                skewMatrix(Rd.col(2))*skewMatrix(Re.col(2)));
    Eigen::MatrixXd E(6, 6);
    E << -1*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), -1*L;
    return E * start_state.getJacobian(joint_model_group, T_offset.translation());
}

void save_trajectory(moveit_msgs::RobotTrajectory trajectory_msg, std::string filepath)
{
    std::string full_filepath = filepath + ".bag";
    int last_slash = filepath.find_last_of("/");
    std::string filename = filepath.substr(last_slash+1);

    rosbag::Bag bag;

// <<<<<<< HEAD
    if (!boost::filesystem::exists(full_filepath)){
        bag.open(full_filepath, rosbag::bagmode::Write);
        bag.write(filename + "_0", ros::Time::now(), trajectory_msg);
    }
    else{
        bag.open(full_filepath, rosbag::bagmode::Append);
        bag.write(filename + "_1", ros::Time::now(), trajectory_msg);
    }
// =======
    // if (!boost::filesystem::exists(full_filename)){
    //     bag.open(full_filename, rosbag::bagmode::Write);
    //     bag.write(traj_id + "_0", ros::Time::now(), trajectory_msg);
    // }
    // else{
    //     bag.open(full_filename, rosbag::bagmode::Append);
    //     bag.write(traj_id + "_1", ros::Time::now(), trajectory_msg);
    // }

    bag.close();

    ROS_INFO_STREAM("Saved trajectory to file: " << full_filepath);
}