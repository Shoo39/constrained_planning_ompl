#pragma once

#include <memory>
#include <bitset>
#include <cmath>
#include <XmlRpcException.h>
#include <ompl/base/Constraint.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <Eigen/Geometry>
#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace kin_constr
{
namespace ob = ompl::base;

// Enumeration defining available options
enum TaskFrameType{
    FIXED,
    PARAMETRIZED
};
enum OrientationType{
    RPY,
    QUAT
};
enum ConstraintAdherenceMethod{
    PROJECTED,
    ATLAS,
    TANGENT_BUNDLE
};
enum ErrorFunctionType{
    SCALAR,
    SCALAR_WITH_TOL_LIN,
    VECTOR,
    VECTOR_WITH_TOL_LIN
};

class KinConstraint : public ob::Constraint
{
public:
    KinConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs, 
                    moveit_msgs::Constraints constraints, const Eigen::VectorXd motion_constraint_vector,
                    const int error_fcn_type, const bool fixed=true, const unsigned int num_constr=1, double tolerance=1e-4);

    /** \brief Function F(q) passed to OMPL which describes the constraint in form of F(q)=0 **/
    void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;

    /** \brief Jacobian of the constraint function: J(q)=(dF(q)/d(q))^T **/
    void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override;

    // Helper functions

    /** \brief Compute FK for the current joint configuration for the specified link w.r.t the world frame **/
    Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values, const std::string& link_name) const;

    /** \brief Compute Geometric Jacobian for the current joint configuration w.r.t the world frame **/
    Eigen::MatrixXd geometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values, const std::string& link_name, 
                                        const Eigen::Ref<const Eigen::Vector3d>& offset) const;
                                        
    /** \brief Compute the Geometric Jacobian w.r.t. the task frame **/                                    
    Eigen::MatrixXd taskFrameJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

    /** \brief Compute the Analytical Jacobian (with derivatives of RPY) w.r.t. the task frame **/
    Eigen::MatrixXd analyticalTaskFrameJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;
    
    /** \brief Compute the Analytical Jacobian (with derivatives of quaternion) w.r.t. the task frame **/
    Eigen::MatrixXd quatTaskFrameJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

    Eigen::MatrixXd angleAxisTaskFrameJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

    /** \brief Calculate the value of the parameter that is being constrained **/
    Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

    /** \brief Calculate the value of the parameter that is being constrained w.r.t tolerances **/
    Eigen::VectorXd calcErrorWithTol(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

    /** **/
    Eigen::VectorXd calcTolMask(const Eigen::Ref<const Eigen::VectorXd>& error) const;

    bool validatePose(const Eigen::Isometry3d& T);

    /** \brief Get roll, pitch, yaw angles from rotation matrix **/
    Eigen::Vector3d extractRPY(const Eigen::Ref<const Eigen::MatrixXd>& R) const;

    /** \brief Computes the skew matrix for the passed vector**/
    Eigen::Matrix3d skewMatrix(const Eigen::Ref<const Eigen::Vector3d>& vec) const;

    bool correctRobotState(robot_state::RobotState& rs);

    /** \brief Helper function for filling message **/
    static moveit_msgs::Constraints createKinConstraintMsg(XmlRpc::XmlRpcValue& constraint_list);

private:
    const std::size_t num_dofs_;   

    // Motion constraint vector (as diagonal matrix)
    Eigen::MatrixXd C_;

    // Representation of orientation (see enum at top)
    uint8_t orientation_type_;

    // Type of task frame (see enum at top)
    uint8_t task_frame_type_; 

    // Type of error function (see enum at top) 
    uint8_t error_fcn_type_;

    // Task frame
    Eigen::Isometry3d T0_t_;

    Eigen::Isometry3d T_offset_;

    // Tolerances for each value of the Cartesian error, described in the task frame
    Eigen::Vector3d position_tolerances_;
    Eigen::Vector3d orientation_tolerances_;
    Eigen::VectorXd tolerances_;

    robot_model::RobotModelConstPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;
    const robot_state::JointModelGroup* joint_model_group_;
    // Link for which the constraints are definied (usually the end-effector)
    std::string link_name_;     
};

typedef std::shared_ptr<KinConstraint> KinConstraintPtr;

// Helper function to create a KinConstraint instance.
KinConstraintPtr createConstraint(robot_model::RobotModelConstPtr robot_model, 
                                    const std::string& group, moveit_msgs::Constraints constraints, int error_fcn_type);

}