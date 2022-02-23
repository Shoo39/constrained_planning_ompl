#include "kin_constr_ompl_interface.h"

namespace kin_constr
{
KinConstrOMPLInterface::KinConstrOMPLInterface(const moveit::core::RobotModelConstPtr& robot_model, const ros::NodeHandle& nh):
  nh_(nh), robot_model_(robot_model)
{
  loadPlannerConfigurations();
}

bool KinConstrOMPLInterface::loadPlannerConfiguration(
    const std::string& group_name, const std::string& planner_id,
    const std::map<std::string, std::string>& group_params,
    planning_interface::PlannerConfigurationSettings& planner_config)
{
  XmlRpc::XmlRpcValue xml_config;
  if (!nh_.getParam("planner_configs/" + planner_id, xml_config))
  {
    ROS_ERROR_STREAM("Could not find the planner configuration '" << planner_id << "' on the param server");
    return false;
  }

  if (xml_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_STREAM("A planning configuration should be of type XmlRpc Struct type (for configuration '" << planner_id << "')");
    return false;
  }

  planner_config.name = group_name + "[" + planner_id + "]";
  planner_config.group = group_name;

  // default to specified parameters of the group (overridden by configuration specific parameters)
  planner_config.config = group_params;

  // read parameters specific for this configuration
  for (std::pair<const std::string, XmlRpc::XmlRpcValue>& element : xml_config)
  {
    if (element.second.getType() == XmlRpc::XmlRpcValue::TypeString)
      planner_config.config[element.first] = static_cast<std::string>(element.second);
    else if (element.second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      planner_config.config[element.first] = moveit::core::toString(static_cast<double>(element.second));
    else if (element.second.getType() == XmlRpc::XmlRpcValue::TypeInt)
      planner_config.config[element.first] = std::to_string(static_cast<int>(element.second));
    else if (element.second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      planner_config.config[element.first] = std::to_string(static_cast<bool>(element.second));
  }

  return true;
}

void KinConstrOMPLInterface::loadPlannerConfigurations()
{
  // read the planning configuration for each group
  planning_interface::PlannerConfigurationMap pconfig;
  pconfig.clear();

  for (const std::string& group_name : robot_model_->getJointModelGroupNames())
  {
    // the set of planning parameters that can be specific for the group (inherited by configurations of that group)
    static const std::string KNOWN_GROUP_PARAMS[] = { "projection_evaluator", "longest_valid_segment_fraction",
                                                      "enforce_joint_model_state_space", "constraint_function_type",
                                                      "constraint_adherence_method", "delta" };

    // get parameters specific for the robot planning group
    std::map<std::string, std::string> specific_group_params;
    for (const std::string& k : KNOWN_GROUP_PARAMS)
    {
      std::string param_name{ group_name };
      param_name += "/";
      param_name += k;

      if (nh_.hasParam(param_name))
      {
        std::string value;
        if (nh_.getParam(param_name, value))
        {
          if (!value.empty())
            specific_group_params[k] = value;
          continue;
        }

        double value_d;
        if (nh_.getParam(param_name, value_d))
        {
          // convert to string using no locale
          specific_group_params[k] = moveit::core::toString(value_d);
          continue;
        }

        int value_i;
        if (nh_.getParam(param_name, value_i))
        {
          specific_group_params[k] = std::to_string(value_i);
          continue;
        }

        bool value_b;
        if (nh_.getParam(param_name, value_b))
        {
          specific_group_params[k] = std::to_string(value_b);
          continue;
        }
      }
    }

    // add default planner configuration
    planning_interface::PlannerConfigurationSettings default_pc;
    std::string default_planner_id;
    if (nh_.getParam(group_name + "/default_planner_config", default_planner_id))
    {
      if (!loadPlannerConfiguration(group_name, default_planner_id, specific_group_params, default_pc))
        default_planner_id = "";
    }

    if (default_planner_id.empty())
    {
      default_pc.group = group_name;
      default_pc.config = specific_group_params;
      default_pc.config["type"] = "geometric::RRTConnect";
    }

    default_pc.name = group_name;  // this is the name of the default config
    pconfig[default_pc.name] = default_pc;

    // get parameters specific to each planner type
    XmlRpc::XmlRpcValue config_names;
    if (nh_.getParam(group_name + "/planner_configs", config_names))
    {
      if (config_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR_STREAM("The planner_configs argument of a group configuration "
                        "should be an array of strings (for group '" << group_name<< "')");
        continue;
      }

      for (int j = 0; j < config_names.size(); ++j)  // NOLINT(modernize-loop-convert)
      {
        if (config_names[j].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_ERROR_STREAM("Planner configuration names must be of type string (for group '" << group_name << "')");
          continue;
        }

        const std::string planner_id = static_cast<std::string>(config_names[j]);

        planning_interface::PlannerConfigurationSettings pc;
        if (loadPlannerConfiguration(group_name, planner_id, specific_group_params, pc))
          pconfig[pc.name] = pc;
      }
    }
  }

  for (const std::pair<const std::string, planning_interface::PlannerConfigurationSettings>& config : pconfig)
  {
    ROS_DEBUG_STREAM("Parameters for configuration '" << config.first << "'");

    for (const std::pair<const std::string, std::string>& parameters : config.second.config)
      ROS_DEBUG_STREAM(" - " << parameters.first << " = " << parameters.second);
  }

  setPlannerConfigurations(pconfig);
}

void KinConstrOMPLInterface::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig)
{
  planning_interface::PlannerConfigurationMap pconfig2 = pconfig;

  // construct default configurations for planning groups that don't have configs already passed in
  for (const moveit::core::JointModelGroup* group : robot_model_->getJointModelGroups())
  {
    if (pconfig.find(group->getName()) == pconfig.end())
    {
      planning_interface::PlannerConfigurationSettings empty;
      empty.name = empty.group = group->getName();
      pconfig2[empty.name] = empty;
    }
  }

  planner_configs_ = pconfig2;
}

const planning_interface::PlannerConfigurationMap& KinConstrOMPLInterface::getPlannerConfigurations() const
{
  return planner_configs_;
}

KinConstrPlanningContextPtr KinConstrOMPLInterface::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene, 
                                          const moveit_msgs::MotionPlanRequest& req, moveit_msgs::MoveItErrorCodes& error_code) const
{
  if (req.group_name.empty())
  {
    ROS_ERROR_STREAM("No group specified to plan for");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return KinConstrPlanningContextPtr();
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  if (!planning_scene)
  {
    ROS_ERROR_STREAM("No planning scene supplied as input");
    return KinConstrPlanningContextPtr();
  }

  // Identify correct planning configuration
  auto pc = planner_configs_.end();
  if (!req.planner_id.empty())
  {
    pc = planner_configs_.find(req.planner_id.find(req.group_name) == std::string::npos ? 
                                req.group_name + "[" + req.planner_id + "]" :
                                req.planner_id);
    if (pc == planner_configs_.end()){
      ROS_WARN_STREAM("Cannot find planning configuration for group " << req.group_name << " using planner " << req.planner_id << ". Will use defaults instead.");
    }
  }

  if (pc == planner_configs_.end())
  {
    // Default config
    pc = planner_configs_.find(req.group_name);
    if (pc == planner_configs_.end())
    {
      ROS_ERROR_STREAM("Cannot find planning configuration for group " << req.group_name);
      return KinConstrPlanningContextPtr();
    }
  }

  //pc->second contains group specific configurations
  KinConstrPlanningContextPtr context = KinConstrPlanningContextPtr(new KinConstrPlanningContext("kin_constr_planning_context", req.group_name, robot_model_));
  context->setConfig(pc->second);
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);

  //TODO: Set Complete Initial State, Planning Volume, Goal Constraints, max. Goal Samples, ...
  return context;

}

}  // namespace kin_constr

