#include "kin_constraint.h"


namespace kin_constr
{
KinConstraint::KinConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs, 
                                moveit_msgs::Constraints constraints, const Eigen::VectorXd motion_constraint_vector,
                                const int error_fcn_type, const bool fixed, const unsigned int num_constr, double tolerance) :
    ob::Constraint(num_dofs, num_constr, tolerance), num_dofs_(num_dofs), robot_model_(robot_model), error_fcn_type_(error_fcn_type)
{
    robot_state_.reset(new robot_state::RobotState(robot_model_));
    robot_state_->setToDefaultValues();
    joint_model_group_ = robot_state_->getJointModelGroup(group);

    ROS_INFO_STREAM("Tolerance: " << tolerance);

    ROS_INFO_STREAM("Creating constraints of shape (" << num_dofs << ", " << getCoDimension() << ")");

    if ((error_fcn_type_==ErrorFunctionType::SCALAR_WITH_TOL_LIN) || 
        (error_fcn_type_==ErrorFunctionType::VECTOR_WITH_TOL_LIN)){
        ROS_INFO_STREAM("Tolerances are considered.");
    }

    //TODO: Enable check without error message
    /*++ Get link frame which is subject to the constraint ++*/
    std::string link = constraints.position_constraints[0].link_name;
    if (joint_model_group_->getLinkModel(link)){
        link_name_ = link;
    }
    else{
        link_name_ = joint_model_group_->getLinkModelNames().back();
        ROS_INFO_STREAM("Error above can be ignored");
    }
    ROS_INFO_STREAM("Creating OMPL constraints for link: " << link_name_ );

    /*++ Check whether task frame is fixed or not (parametrized) ++*/
    if (fixed){
        task_frame_type_ = TaskFrameType::FIXED;
        ROS_INFO_STREAM("Using fixed task frame");
    }
    else {
        task_frame_type_ = TaskFrameType::PARAMETRIZED;
        ROS_INFO_STREAM("Using parametrized task frame");
    }

    /*++ Get motion constraint vector ++*/
    // Store vector as diagonal matrix to simplify math. operations
    C_ = motion_constraint_vector.asDiagonal();

    ROS_INFO_STREAM("Using Motion Constraint Vector: " << motion_constraint_vector.transpose());

    /*++ Define task frame ++*/
    T0_t_ = Eigen::Translation3d( constraints.position_constraints[0].constraint_region.primitive_poses[0].position.x,
                                constraints.position_constraints[0].constraint_region.primitive_poses[0].position.y,
                                constraints.position_constraints[0].constraint_region.primitive_poses[0].position.z ) *
            Eigen::Quaterniond( constraints.position_constraints[0].constraint_region.primitive_poses[0].orientation.w,
                                constraints.position_constraints[0].constraint_region.primitive_poses[0].orientation.x,
                                constraints.position_constraints[0].constraint_region.primitive_poses[0].orientation.y,
                                constraints.position_constraints[0].constraint_region.primitive_poses[0].orientation.z ); 

    T_offset_ = Eigen::Translation3d( constraints.position_constraints[0].target_point_offset.x,
                                        constraints.position_constraints[0].target_point_offset.y,
                                        constraints.position_constraints[0].target_point_offset.z ) *
                Eigen::Quaterniond( constraints.orientation_constraints[0].orientation.w,
                                        constraints.orientation_constraints[0].orientation.x,
                                        constraints.orientation_constraints[0].orientation.y,
                                        constraints.orientation_constraints[0].orientation.z );                                            

    /*++ Set representation type of orientation error ++*/
    if (constraints.orientation_constraints[0].parameterization == moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES){
        orientation_type_ = OrientationType::RPY;
       ROS_INFO_STREAM("Using rpy representation");
    }
    else{
        orientation_type_ = OrientationType::QUAT;
        ROS_INFO_STREAM("Using quaternion representation");
    }

    // /*++ Extract tolerances ++*/
    // //TODO: Enable quaternion tolerance description
    position_tolerances_ << constraints.position_constraints[0].constraint_region.primitives[0].dimensions[0],
                            constraints.position_constraints[0].constraint_region.primitives[0].dimensions[1],
                            constraints.position_constraints[0].constraint_region.primitives[0].dimensions[2];
    orientation_tolerances_ <<  constraints.orientation_constraints[0].absolute_x_axis_tolerance,
                                constraints.orientation_constraints[0].absolute_y_axis_tolerance,
                                constraints.orientation_constraints[0].absolute_z_axis_tolerance;
    tolerances_ = Eigen::VectorXd(6, 1);
    tolerances_ << position_tolerances_, orientation_tolerances_;
    ROS_INFO_STREAM("Tolerance vector: " << tolerances_.transpose());
}

Eigen::Isometry3d KinConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values, const std::string& link_name) const
{
    // Using robot_state_ directly is not safe
    robot_state::RobotState rs(*robot_state_);
    rs.setJointGroupPositions(joint_model_group_, joint_values);
    rs.update();
    return rs.getGlobalLinkTransform(link_name) * T_offset_;
}

Eigen::MatrixXd KinConstraint::geometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values, const std::string& link_name, 
                                    const Eigen::Ref<const Eigen::Vector3d>& offset) const
{
    Eigen::MatrixXd J;
    // Using robot_state_ directly is not safe
    robot_state::RobotState rs(*robot_state_);
    rs.setJointGroupPositions(joint_model_group_, joint_values);
    rs.update();
    rs.getJacobian(joint_model_group_, rs.getLinkModel(link_name), offset, J);
    return J;
}

Eigen::MatrixXd KinConstraint::taskFrameJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
    Eigen::Matrix3d Rt_0 = T0_t_.rotation().transpose();

    // Transformation matrix to transform Jacobian from world to task frame:
    //  |Rt_0    0|
    //  |0    Rt_0|
    Eigen::MatrixXd M(2*Rt_0.rows(), 2*Rt_0.cols());
    M << Rt_0, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Rt_0;

    // Task frame Jacobian
    Eigen::MatrixXd Jt = M * geometricJacobian(joint_values, link_name_, Eigen::Vector3d(T_offset_.translation()));
    return Jt;
}

Eigen::MatrixXd KinConstraint::analyticalTaskFrameJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
    Eigen::Vector3d rpy = extractRPY((T0_t_.inverse() * forwardKinematics(joint_values, link_name_)).rotation());

    Eigen::Matrix3d R_trafo = Eigen::Matrix3d::Zero();
    R_trafo(0, 0) = cos(rpy[2]) / cos(rpy(1));
    R_trafo(0, 1) = sin(rpy[2]) / cos(rpy(1));
    
    R_trafo(1, 0) = -sin(rpy[2]);
    R_trafo(1, 1) = cos(rpy[2]);

    R_trafo(2, 0) = cos(rpy[2])*sin(rpy[1])/cos(rpy[1]);
    R_trafo(2, 1) = sin(rpy[2])*sin(rpy[1])/cos(rpy[1]);
    R_trafo(2, 2) = 1;

    Eigen::MatrixXd E_rpy(6, 6);
    E_rpy << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), R_trafo;

    // Analytical task frame Jacobian
    Eigen::MatrixXd Jt_a = E_rpy * taskFrameJacobian(joint_values);
    return Jt_a;
}

Eigen::MatrixXd KinConstraint::quatTaskFrameJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
    Eigen::Isometry3d Tt_e = T0_t_.inverse() * forwardKinematics(joint_values, link_name_);
    Eigen::Quaterniond Q = Eigen::Quaterniond(Tt_e.rotation());

    Eigen::Matrix3d M = 0.5*(Q.w()*Eigen::Matrix3d::Identity() - skewMatrix(Q.vec()));

    Eigen::MatrixXd K(6,6);
    K << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), M;
    
    return K * taskFrameJacobian(joint_values);
}

Eigen::MatrixXd KinConstraint::angleAxisTaskFrameJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
    Eigen::Isometry3d Tt_e = T0_t_.inverse() * forwardKinematics(joint_values, link_name_);
    
    Eigen::Matrix3d Rt_e = Tt_e.rotation();
    // Eigen::Matrix3d L = -0.5 * (skewMatrix(Eigen::Vector3d(1,0,0)) * skewMatrix(Rt_e.col(0)) + 
    //                             skewMatrix(Eigen::Vector3d(0,1,0)) * skewMatrix(Rt_e.col(1)) + 
    //                             skewMatrix(Eigen::Vector3d(0,0,1)) * skewMatrix(Rt_e.col(2)));
    // Eigen::MatrixXd E(6,6);
    // E << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), L;

    Eigen::Matrix3d L = -0.5 * (skewMatrix(Rt_e.col(0)) * skewMatrix(Eigen::Vector3d(1,0,0)) + 
                                skewMatrix(Rt_e.col(1)) * skewMatrix(Eigen::Vector3d(0,1,0)) + 
                                skewMatrix(Rt_e.col(2)) * skewMatrix(Eigen::Vector3d(0,0,1)));
    Eigen::MatrixXd E(6,6);
    E << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), -L;
    

    return E * taskFrameJacobian(joint_values);
}

void KinConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const
{
    // Scalar case
    if (error_fcn_type_==ErrorFunctionType::SCALAR){
        out = 0.5*calcError(x).transpose() * C_ * calcError(x);
    }
    else if (error_fcn_type_==ErrorFunctionType::SCALAR_WITH_TOL_LIN){
        out = 0.5*calcErrorWithTol(x).transpose() * C_ * calcErrorWithTol(x);
    }
    // Vector case
    else{
        out = Eigen::VectorXd(getCoDimension(), 1);
        Eigen::VectorXd error;

        if (error_fcn_type_==ErrorFunctionType::VECTOR){
            error = calcError(x);
        }
        else if (error_fcn_type_==ErrorFunctionType::VECTOR_WITH_TOL_LIN){
            error = calcErrorWithTol(x);
        }
        else{
            ROS_ERROR("Unknown error function type was specified.");
        }

        int counter = 0;
        for(int i=0; i<6; i++){
            if(C_(i,i)==1){
                out(counter) = error(i);
                counter++;
            }
        }
    }
}

void KinConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const
{
    Eigen::MatrixXd J;

    if (orientation_type_ == OrientationType::RPY){
        if ((error_fcn_type_==ErrorFunctionType::SCALAR) || (error_fcn_type_==ErrorFunctionType::VECTOR)){
            J = analyticalTaskFrameJacobian(x);
        }
        else {
            J = calcError(x).cwiseSign().asDiagonal() * analyticalTaskFrameJacobian(x);  // Works also with Atlas and TangentBundle
            // J = calcTolMask(calcError(x)).asDiagonal() * analyticalTaskFrameJacobian(x);
        }
    }
    else{
        if ((error_fcn_type_==ErrorFunctionType::SCALAR) || (error_fcn_type_==ErrorFunctionType::VECTOR)){
            J = quatTaskFrameJacobian(x);
            // J = angleAxisTaskFrameJacobian(x);
        }
        else {
            J = calcError(x).cwiseSign().asDiagonal() * quatTaskFrameJacobian(x);   // Works also with Atlas and TangentBundle (kind of)
            // J = calcError(x).cwiseSign().asDiagonal() * angleAxisTaskFrameJacobian(x); 
            // J = calcTolMask(calcError(x)).asDiagonal() * quatTaskFrameJacobian(x);
        }
    }

    if (error_fcn_type_==ErrorFunctionType::SCALAR){
        out = calcError(x).transpose() * C_ * J;
    }
    else if (error_fcn_type_==ErrorFunctionType::SCALAR_WITH_TOL_LIN){
        out = calcErrorWithTol(x).transpose() * C_ * J;
    }
    else{
        out = Eigen::MatrixXd(getCoDimension(), num_dofs_);
    
        int counter = 0;
        for(int i=0; i<6; i++){
            if(C_(i,i)==1){
                out.row(counter) = J.row(i);
                counter++;
            }
        }
    }
}

Eigen::VectorXd KinConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
    Eigen::VectorXd error(6, 1);
    
    if (orientation_type_ == OrientationType::RPY){
        Eigen::Isometry3d Tt_e = T0_t_.inverse() * forwardKinematics(joint_values, link_name_);
    
        Eigen::Vector3d translation_error(Tt_e.translation());
        Eigen::Vector3d orientation_error_rpy = extractRPY(Tt_e.rotation());

        error << translation_error, orientation_error_rpy;
    }
    else{
        // Quaternion error
        Eigen::Isometry3d Tt_e = T0_t_.inverse() * forwardKinematics(joint_values, link_name_);
        Eigen::Vector3d translation_error(Tt_e.translation());

        Eigen::Quaterniond Q = Eigen::Quaterniond(Tt_e.rotation());
        error << translation_error, Q.vec();

        // // Axis-Angle error
        // Eigen::Isometry3d Tt_e = T0_t_.inverse() * forwardKinematics(joint_values, link_name_);
        // Eigen::Matrix3d Rt_e = Tt_e.rotation();

        // Eigen::Vector3d translation_error(Tt_e.translation());
        // // Eigen::Vector3d orientation_error = 0.5 * ( skewMatrix(Eigen::Vector3d(1,0,0))*Rt_e.col(0) + 
        // //                                             skewMatrix(Eigen::Vector3d(0,1,0))*Rt_e.col(1) +
        // //                                             skewMatrix(Eigen::Vector3d(0,0,1))*Rt_e.col(2) );
        // Eigen::Vector3d orientation_error = 0.5 * ( skewMatrix(Rt_e.col(0))*Eigen::Vector3d(1,0,0) + 
        //                                             skewMatrix(Rt_e.col(1))*Eigen::Vector3d(0,1,0) +
        //                                             skewMatrix(Rt_e.col(2))*Eigen::Vector3d(0,0,1) );

        // error << translation_error, orientation_error;
    }

    return error;
}

Eigen::VectorXd KinConstraint::calcErrorWithTol(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
    Eigen::VectorXd error = calcError(joint_values);

    if ((error_fcn_type_==ErrorFunctionType::SCALAR_WITH_TOL_LIN) || (error_fcn_type_==ErrorFunctionType::VECTOR_WITH_TOL_LIN)){
        Eigen::VectorXd mask = calcTolMask(error);
        Eigen::VectorXd masked_error(6, 1);
        masked_error = mask.cwiseProduct(error) - tolerances_.cwiseProduct(mask.cwiseAbs());
        return masked_error;
    }
    else {
        return Eigen::VectorXd::Zero(6);
    }
}

bool KinConstraint::validatePose(const Eigen::Isometry3d& T) 
{
    Eigen::VectorXd error(6, 1);
    bool satisfies_constraints = true;

    if (orientation_type_ == OrientationType::RPY){
        Eigen::Isometry3d Tt_e = T0_t_.inverse() * T;
    
        Eigen::Vector3d translation_error(Tt_e.translation());
        Eigen::Vector3d rotation_error_rpy = extractRPY(Tt_e.rotation());

        error << translation_error, rotation_error_rpy;
    }
    else{
        // Quaternion error
        Eigen::Isometry3d Tt_e = T0_t_.inverse() * T;
        Eigen::Vector3d translation_error(Tt_e.translation());

        Eigen::Quaterniond Q = Eigen::Quaterniond(Tt_e.rotation());
        error << translation_error, Q.vec();
    }

    if ((error_fcn_type_==ErrorFunctionType::SCALAR_WITH_TOL_LIN) || (error_fcn_type_==ErrorFunctionType::VECTOR_WITH_TOL_LIN)){
        Eigen::VectorXd mask = calcTolMask(error);
        Eigen::VectorXd masked_error(6, 1);
        masked_error = mask.cwiseProduct(error) - tolerances_.cwiseProduct(mask.cwiseAbs());
        error = masked_error;
    }

    error = C_ * error;

    for (int i=0; i<error.size(); i++){
        if (error[i] > getTolerance()){
            satisfies_constraints &= false;
            ROS_WARN_STREAM("Given pose violates constraints at coordinate: " << i << ". Value: " << error[i]);
        }
    }

    return satisfies_constraints;
}

Eigen::VectorXd KinConstraint::calcTolMask(const Eigen::Ref<const Eigen::VectorXd>& error) const
{
    Eigen::VectorXd mask(6, 1);
    for (int i=0; i<6; i++){
        if(error(i)<-tolerances_(i)){
            mask(i) = -1;
        }
        else if(error(i)>tolerances_(i)){
            mask(i) = 1;
        }
        else{
            mask(i) = 0;
        }
    }
    return mask;
}

Eigen::Vector3d KinConstraint::extractRPY(const Eigen::Ref<const Eigen::MatrixXd>& R) const
{
    // Naming might be confusing, but since rotation angles around fixed x, y and z axis is needed,
    // the actual order is yaw, pitch, roll.
    Eigen::Vector3d rpy;    
    double yaw = atan2(R(2,1), R(2,2));
    double pitch = -asin(R(2,0));
    double roll = atan2(R(1,0), R(0,0));
    rpy << yaw, pitch, roll; 

    return rpy;
}

Eigen::Matrix3d KinConstraint::skewMatrix(const Eigen::Ref<const Eigen::Vector3d>& vec) const
{
    Eigen::Matrix3d S;
    S <<    0,      -vec(2),    vec(1), 
            vec(2), 0,          -vec(0), 
            -vec(1), vec(0),    0;
    return S;
}

moveit_msgs::Constraints KinConstraint::createKinConstraintMsg(XmlRpc::XmlRpcValue& constraint_list){
    ROS_INFO_STREAM("Creation of kinematic constraint message");
    moveit_msgs::Constraints path_constraints;

    /* Important note: 
        Here the moveit_msgs::Constraints message is used in order to store values needed for the creation of kinematic constraints defined above.
        This is because of the fixed motion planning pipeline structure, which forces one to use this message class.
        However, the filled in values do not make sense to the constraint definition given by MoveIt. Therefore using MoveIts functions 
        for state validity check with these messages will fail (for example using 'default_planner_request_adapters/FixStartStatePathConstraints'
        for postprocessing the trajectory will return failures.) 
    */

    try{
        std::string prefix = "fixed_";
        // if (constraint_list["task_frame_fixed"]){
        //     prefix = "fixed_";
        // }
        // Since I couldn't find any appropriate vector member inside moveit_msgs::Constraints 
        // which could be used in order to save the motion constraint vector this information
        // will be encoded inside the 'name'
        unsigned int encoded_vector = 0;
        const int vector_size = constraint_list["motion_constraint_vector"].size();
        int num = 0;

        for (int i=0; i<vector_size; i++){
            num = constraint_list["motion_constraint_vector"][i];
            if (num){
                prefix += "1";
            }
            else{
                prefix += "0";
            }
        }
        prefix += "_";
        path_constraints.name = prefix + std::string(constraint_list["name"]);
        

        /* --- Set position constraint --- */ 
        moveit_msgs::PositionConstraint position_constraint;
        position_constraint.header.frame_id = std::string(constraint_list["frame_id"]);
        position_constraint.link_name = std::string(constraint_list["link"]);

        // Position offset to target point w.r.t link
        if (constraint_list.hasMember("offset_pose")){
            position_constraint.target_point_offset.x = constraint_list["offset_pose"][0];
            position_constraint.target_point_offset.y = constraint_list["offset_pose"][1];
            position_constraint.target_point_offset.z = constraint_list["offset_pose"][2];
        }
        else{
            position_constraint.target_point_offset.x = 0;
            position_constraint.target_point_offset.y = 0;
            position_constraint.target_point_offset.z = 0;
        }

        // Define box shaped constraints for x, y, z
        shape_msgs::SolidPrimitive box;
        box.type = shape_msgs::SolidPrimitive::BOX;
        box.dimensions.push_back(constraint_list["tolerance_vector"][0]);
        box.dimensions.push_back(constraint_list["tolerance_vector"][1]);
        box.dimensions.push_back(constraint_list["tolerance_vector"][2]);

        position_constraint.constraint_region.primitives.push_back(box);

        // Define task frame position:
        geometry_msgs::Pose tf_pose;
        tf_pose.position.x = constraint_list["pose_xyzrpy"][0];
        tf_pose.position.y = constraint_list["pose_xyzrpy"][1];
        tf_pose.position.z = constraint_list["pose_xyzrpy"][2];

        tf2::Quaternion quat;
        quat.setRPY(constraint_list["pose_xyzrpy"][3], constraint_list["pose_xyzrpy"][4], constraint_list["pose_xyzrpy"][5]);
        tf_pose.orientation = tf2::toMsg(quat);

        position_constraint.constraint_region.primitive_poses.push_back(tf_pose);

        // Set tolerance (epsilon) for constraint satisfaction |f(q)| = epsilon 
        position_constraint.weight = constraint_list["epsilon"];
            
        path_constraints.position_constraints.push_back(position_constraint);


        /* --- Set orientation constraint --- */
        moveit_msgs::OrientationConstraint orientation_constraint;
        orientation_constraint.header.frame_id = std::string(constraint_list["frame_id"]);
        orientation_constraint.link_name = std::string(constraint_list["link"]);

        // Difference in orientation between link and regarded frame
        if (constraint_list.hasMember("offset_pose")){
            quat.setRPY(constraint_list["offset_pose"][3], 
                    constraint_list["offset_pose"][4], 
                    constraint_list["offset_pose"][5]);
        }
        else{
            quat.setRPY(0, 0, 0);
        }
        orientation_constraint.orientation = tf2::toMsg(quat);

        // Specify representation type of orientation error (rpy or rotation vector/ quaternion)
        if (constraint_list["orientation_error_type_rpy"]){
            orientation_constraint.parameterization = moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES;
        }
        else{
            orientation_constraint.parameterization = moveit_msgs::OrientationConstraint::ROTATION_VECTOR;
        }

        // Specify tolerance for rpy angles (since these are intuitive) 
        orientation_constraint.absolute_x_axis_tolerance = constraint_list["tolerance_vector"][3];
        orientation_constraint.absolute_y_axis_tolerance = constraint_list["tolerance_vector"][4];
        orientation_constraint.absolute_z_axis_tolerance = constraint_list["tolerance_vector"][5];

        path_constraints.orientation_constraints.push_back(orientation_constraint);

    }
    catch(const XmlRpc::XmlRpcException& ex){
        ROS_ERROR_STREAM("Error: '" << ex.getMessage() << "' has occured. Please make sure, that values inside ROS parameter server have the right type.");
        return path_constraints;
    }

    return path_constraints;
}

bool KinConstraint::correctRobotState(robot_state::RobotState& rs)
{
    Eigen::VectorXd joint_positions;
    rs.copyJointGroupPositions(joint_model_group_, joint_positions);

    Eigen::VectorXd error{getCoDimension()};
    function(joint_positions, error);

    double max_error = error.cwiseAbs().maxCoeff();

    ROS_INFO_STREAM("Initial error: " << max_error);

    Eigen::MatrixXd J(getCoDimension(), num_dofs_);
    Eigen::MatrixXd pInv;

    int max_iter = 100;
    for (int i=0; i<max_iter; i++){
        if (max_error < getTolerance()){
            // rs.setJointGroupPositions(joint_model_group_, joint_positions);
            // rs.update();
            ROS_INFO_STREAM("Error after correction: " << max_error);
            return true;
        }
        jacobian(joint_positions, J);

        pInv = J.completeOrthogonalDecomposition().pseudoInverse();

        joint_positions = joint_positions - pInv * error;
        rs.setJointGroupPositions(joint_model_group_, joint_positions);
        rs.enforceBounds();
        rs.copyJointGroupPositions(joint_model_group_, joint_positions);
        rs.update();

        function(joint_positions, error);
        max_error = error.cwiseAbs().maxCoeff();
    }
    ROS_WARN_STREAM("Given state could not be corrected to a constraint satisfying state.");
    return false;
}


KinConstraintPtr createConstraint(robot_model::RobotModelConstPtr robot_model, 
                                    const std::string& group, moveit_msgs::Constraints constraints, int error_fcn_type)
{
    std::size_t num_dofs = robot_model->getJointModelGroup(group)->getVariableCount();

    // Check whether task frame is fixed or not (parametrized)
    int index_under_score = constraints.name.find("_");
    std::string prefix = constraints.name.substr(0, index_under_score);
    std::string substring = constraints.name.substr(index_under_score+1, constraints.name.size() - index_under_score);

    bool fixed = false;

    if (prefix.compare("fixed") == 0){
        prefix = substring;
        fixed = true;
    }

    /* Extract motion constraint vector */ 
    std::string vector_string = prefix.substr(0, prefix.find("_"));

    int counter = 0;

    Eigen::VectorXd motion_constraint_vector(6, 1);
    for (int i=0; i<vector_string.size(); i++){
        if (vector_string[i] == '0'){
            motion_constraint_vector(i) = 0;
        }
        else{
            motion_constraint_vector(i) = 1;
            counter++;
        }
    }

    if (counter == 0){
        ROS_WARN_STREAM("Zero vector specified for motion constrained vector. If ProjectedStateSpace was selected, ignore this warning. However, Atlas and TangentBundleStateSpace will fail.");
        error_fcn_type=ErrorFunctionType::SCALAR;
    }

    if ((error_fcn_type==ErrorFunctionType::SCALAR) || 
        (error_fcn_type==ErrorFunctionType::SCALAR_WITH_TOL_LIN)){
        counter = 1;
    }

    double tolerance = constraints.position_constraints[0].weight;
    
    if (num_dofs > 0){
        auto kinematic_constraint = std::make_shared<KinConstraint>(robot_model, group, num_dofs, 
                                        constraints, motion_constraint_vector, error_fcn_type, fixed, counter, tolerance);
        return kinematic_constraint;
    }
    else    
        return nullptr;
}
}
