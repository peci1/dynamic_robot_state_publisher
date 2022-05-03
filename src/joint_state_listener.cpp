#include "dynamic_robot_state_publisher/joint_state_listener.h"

#include <kdl_parser/kdl_parser.hpp>

using namespace robot_state_publisher;

bool robot_state_publisher::DynamicJointStateListener::loadModel(KDL::Tree &tree,
    MimicMap &mimic_map, urdf::Model &model, const std::string &urdf)
{
  // gets the location of the robot description on the parameter server
  bool found;
  if (urdf.empty())
  {
    found = model.initParam("robot_description");
    if (!found)
    {
      ROS_ERROR("Could not read urdf_model from parameter server");
      return false;
    }
  }
  else
  {
    found = model.initString(urdf);
    if (!found)
    {
      ROS_ERROR("Could not read urdf_model from string: %s", urdf.c_str());
      return false;
    }
  }

  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return false;
  }

  mimic_map.clear();
  for (const auto &joint : model.joints_)
  {
    if (joint.second->mimic)
    {
      mimic_map.insert(make_pair(joint.first, joint.second->mimic));
    }
  }

  return true;
}

robot_state_publisher::DynamicJointStateListener::DynamicJointStateListener(
  const KDL::Tree &tree, const MimicMap &m, const urdf::Model &model)
#if ROS_VERSION_MINIMUM(1, 15, 0)
	: JointStateListener(std::make_shared<DynamicRobotStatePublisher>(tree, model), m)
#else
  : JointStateListener(tree, m, model), dynamicPublisher(&state_publisher_)
#endif
{
  ROS_INFO("Robot model loaded, it has %lu moving joints and "
           "%lu fixed joints.", this->getDynamicPublisher().getNumMovingJoints(),
           this->getDynamicPublisher().getNumFixedJoints());

  dynparamServer.setCallback(boost::bind(
    &robot_state_publisher::DynamicJointStateListener::configChangedCb,
    this, _1, _2));
	
	std::string fullParam;
	ros::NodeHandle nh;
	if (nh.searchParam("robot_description", fullParam))
	{
		DynamicRobotStateConfig config;
		nh.getParam(fullParam, config.robot_description);
		dynparamServer.updateConfig(config);
	}
}


bool robot_state_publisher::DynamicJointStateListener::reloadRobotModel(
  const std::string &urdf)
{
	boost::recursive_mutex::scoped_lock lock(updateOngoing);

  ROS_INFO("Reloading robot model.");

  timer_.stop();  /// make sure that state_publisher is not currently publishing
  publish_interval_.sleep();       /// allow publishFixedTransforms to end

  KDL::Tree tree;
  urdf::Model model;
  mimic_.clear();

  if (!loadModel(tree, mimic_, model, urdf))
    return false;

  this->getDynamicPublisher().updateTree(tree);
  timer_.start();        /// fixed transforms are published again

  ROS_INFO("Robot model reload complete, it has %lu moving joints and "
           "%lu fixed joints.", this->getDynamicPublisher().getNumMovingJoints(),
           this->getDynamicPublisher().getNumFixedJoints());

  return true;
}

void robot_state_publisher::DynamicJointStateListener::callbackJointState(
  const JointStateConstPtr &state)
{
  if (updateOngoing.try_lock())
  {
    JointStateListener::callbackJointState(state);
    updateOngoing.unlock();
  }
  else
  {
    ROS_WARN_THROTTLE(1, "Skipping joint state while robot model is being "
                         "updated.");
    return;
  }
}

void robot_state_publisher::DynamicJointStateListener::configChangedCb(
  DynamicRobotStateConfig &config, uint32_t level)
{
  if (level == 0)
    reloadRobotModel(config.robot_description);
}

DynamicRobotStatePublisher& robot_state_publisher::DynamicJointStateListener::getDynamicPublisher()
{
#if ROS_VERSION_MINIMUM(1, 15, 0)
	auto dynamicPublisher =
	  std::dynamic_pointer_cast<DynamicRobotStatePublisher>(this->state_publisher_);
	if (dynamicPublisher == nullptr)
		throw std::runtime_error("state publisher is not an instance of DynamicRobotStatePubliher!");
	return *dynamicPublisher;
#else
	return this->dynamicPublisher;
#endif
}
