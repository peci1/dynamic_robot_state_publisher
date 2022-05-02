#include <ros/common.h>

// HACK In melodic, we cannot substitute the RobotStatePublisher in JointStateListener, so we hack it like this
#if !ROS_VERSION_MINIMUM(1, 15, 0)
#define protected public
#endif
#include <robot_state_publisher/robot_state_publisher.h>
#if !ROS_VERSION_MINIMUM(1, 15, 0)
#undef protected
#endif

#include "dynamic_robot_state_publisher/robot_state_publisher.h"

using namespace robot_state_publisher;

std::string stripSlash(const std::string & in)
{
  if (!in.empty() && in[0] == '/')
  {
    return in.substr(1);
  }
  return in;
}

void robot_state_publisher::DynamicRobotStatePublisher::updateTree(const KDL::Tree &tree)
{
  /// function is only called from JointStateListener::reload_robot_model
  /// where the update mutex is acquired

  auto old_segments_fixed = this->getFixedSegments();

  this->getSegments().clear();
  this->getFixedSegments().clear();
  this->addChildren(tree.getRootSegment());

  // find out which segments have disappeared and publish a special TF message
  // that disconnects them from the main TF tree

  for (const auto& seg : this->getFixedSegments())
  {
    old_segments_fixed.erase(seg.first);
  }
  for (const auto& seg : this->getSegments())
  {
    old_segments_fixed.erase(seg.first);
  }

  std::vector<geometry_msgs::TransformStamped> deleteTfs;
  for (const auto& seg : old_segments_fixed)
  {
    geometry_msgs::TransformStamped tf;
    tf.child_frame_id = stripSlash(seg.second.tip);
    tf.header.frame_id = DynamicRobotStatePublisher::DELETED_STATIC_TFS_FRAME;
    tf.header.stamp = ros::Time::now();
    tf.transform.rotation.w = 1.0;
    deleteTfs.push_back(tf);
  }
  this->getStaticTfBroadcaster().sendTransform(deleteTfs);
}

#if ROS_VERSION_MINIMUM(1, 15, 0)
robot_state_publisher::DynamicRobotStatePublisher::DynamicRobotStatePublisher(
	const KDL::Tree& tree, const urdf::Model& model) : robot_state_publisher::RobotStatePublisher(tree, model)
#else
robot_state_publisher::DynamicRobotStatePublisher::DynamicRobotStatePublisher(
  robot_state_publisher::RobotStatePublisher *publisher) : publisher(publisher)
#endif
{
}

size_t robot_state_publisher::DynamicRobotStatePublisher::getNumMovingJoints() const
{
  return this->getSegments().size();
}

size_t robot_state_publisher::DynamicRobotStatePublisher::getNumFixedJoints() const
{
  return this->getFixedSegments().size();
}

std::map<std::string, SegmentPair>& robot_state_publisher::DynamicRobotStatePublisher::getSegments()
{
#if ROS_VERSION_MINIMUM(1, 15, 0)
	return this->segments_;
#else
	return publisher->segments_;
#endif
}

std::map<std::string, SegmentPair>& robot_state_publisher::DynamicRobotStatePublisher::getFixedSegments()
{
#if ROS_VERSION_MINIMUM(1, 15, 0)
	return this->segments_fixed_;
#else
	return publisher->segments_fixed_;
#endif
}

const std::map<std::string, SegmentPair>& robot_state_publisher::DynamicRobotStatePublisher::getSegments() const
{
#if ROS_VERSION_MINIMUM(1, 15, 0)
	return this->segments_;
#else
	return publisher->segments_;
#endif
}

const std::map<std::string, SegmentPair>& robot_state_publisher::DynamicRobotStatePublisher::getFixedSegments() const
{
#if ROS_VERSION_MINIMUM(1, 15, 0)
	return this->segments_fixed_;
#else
	return publisher->segments_fixed_;
#endif
}

tf2_ros::StaticTransformBroadcaster& robot_state_publisher::DynamicRobotStatePublisher::getStaticTfBroadcaster()
{
#if ROS_VERSION_MINIMUM(1, 15, 0)
	return this->static_tf_broadcaster_;
#else
	return publisher->static_tf_broadcaster_;
#endif
}

#if !ROS_VERSION_MINIMUM(1, 15, 0)
void robot_state_publisher::DynamicRobotStatePublisher::addChildren(const KDL::SegmentMap::const_iterator segment)
{
	publisher->addChildren(segment);
}
#endif
