#include <string>
#include <vector>

#include <class_loader/class_loader.hpp>

#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include "kin_constr_planning_context.h"
#include "kin_constr_ompl_interface.h"

namespace kin_constr
{
/** \brief Class that is registered as a planning plugin by CLASS_LOADER_REGISTER_CLASS
 * */
class KinConstrPlannerManager : public planning_interface::PlannerManager
{
public:
  KinConstrPlannerManager() : planning_interface::PlannerManager(), nh_("~")
  {
  }

  bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns) override
  {
    if (!ns.empty()){
      nh_ = ros::NodeHandle(ns);
    }

    ROS_INFO_STREAM("Namespace of KinConstrPlannerManager: " << nh_.getNamespace());

    kc_ompl_interface_.reset(new KinConstrOMPLInterface(model, nh_));

    return true;
  }

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const override
  {
    return true;
  }

  std::string getDescription() const override
  {
    return "KinConstrPlanningOMPL";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("ProjectedStateSpace");
    algs.push_back("AtlasStateSpace");
    algs.push_back("TangentBundleStateSpace");
  }

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const moveit_msgs::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const override

  {
    if (req.group_name.empty())
    {
      ROS_ERROR("No group specified to plan for");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    if (!planning_scene)
    {
      ROS_ERROR("No planning scene supplied as input");
      error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    return kc_ompl_interface_->getPlanningContext(planning_scene, req, error_code);
  }

  /* Set some configurations (probably misused, but it works for now)*/
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override
  {
    // Add a few more configs than passed in (add defaults)
    kc_ompl_interface_->setPlannerConfigurations(pcs);

    PlannerManager::setPlannerConfigurations(kc_ompl_interface_->getPlannerConfigurations());
  }

private:
  ros::NodeHandle nh_;
  std::shared_ptr<KinConstrOMPLInterface> kc_ompl_interface_;
};
}  // namespace kin_constr

CLASS_LOADER_REGISTER_CLASS(kin_constr::KinConstrPlannerManager, planning_interface::PlannerManager);