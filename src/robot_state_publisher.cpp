// HACK we cannot substitute the RobotStatePublisher in JointStateListener, so we hack it like this
#define protected public
#include <robot_state_publisher/robot_state_publisher.h>
#undef protected

#include "dynamic_robot_state_publisher/robot_state_publisher.h"
void robot_state_publisher::DynamicRobotStatePublisher::updateTree(const KDL::Tree &tree)
{
  /// function is only called from JointStateListener::reload_robot_model
  /// where the update mutex is acquired

  publisher->segments_.clear();
  publisher->segments_fixed_.clear();
  publisher->addChildren(tree.getRootSegment());
}

robot_state_publisher::DynamicRobotStatePublisher::DynamicRobotStatePublisher(
  robot_state_publisher::RobotStatePublisher *publisher) : publisher(publisher)
{}

size_t robot_state_publisher::DynamicRobotStatePublisher::getNumMovingJoints() const
{
  return publisher->segments_.size();
}

size_t robot_state_publisher::DynamicRobotStatePublisher::getNumFixedJoints() const
{
  return publisher->segments_fixed_.size();
}
