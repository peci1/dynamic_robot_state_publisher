#ifndef DYNAMIC_ROBOT_STATE_PUBLISHER_ROBOT_STATE_PUBLISHER_H
#define DYNAMIC_ROBOT_STATE_PUBLISHER_ROBOT_STATE_PUBLISHER_H

#include <robot_state_publisher/robot_state_publisher.h>

namespace robot_state_publisher
{
/**
 * \brief An alternative RobotStatePublisher with update option.
 */
class DynamicRobotStatePublisher
#if ROS_VERSION_MINIMUM(1, 15, 0)
	: public robot_state_publisher::RobotStatePublisher
#else
	// In Melodic, we cannot subclass RobotStatePublisher as there is no way to plug it in JointStateListener.
#endif
{
public:
  /** \brief The TF frame name of the virtual frame that's parent of all
   * deleted static TF frames. */
  const std::string DELETED_STATIC_TFS_FRAME = "__deleted_static_tfs__";

#if ROS_VERSION_MINIMUM(1, 15, 0)
	/**
	 * \param tree The kinematic model of a robot, represented by a KDL Tree
	 */
	explicit DynamicRobotStatePublisher(const KDL::Tree& tree, const urdf::Model& model = urdf::Model());
#else
  /**
   * \brief Create the publisher.
   * \param [in] publisher The underlying RobotStatePublisher that is hijacked.
   */
  explicit DynamicRobotStatePublisher(RobotStatePublisher *publisher);
#endif

  /**
   * \brief Sets the robot model.
   * \param tree The kinematic model of a robot, represented by a KDL Tree.
   */
  virtual void updateTree(const KDL::Tree& tree);

  /**
   * \brief Return the number of moving joints in the currently represented model.
   * \return The number of moving joints in the currently represented model.
   */
  virtual size_t getNumMovingJoints() const;

  /**
   * \brief Return the number of fixed joints in the currently represented model.
   * \return The number of fixed joints in the currently represented model.
   */
  virtual size_t getNumFixedJoints() const;

protected:

	std::map<std::string, SegmentPair>& getSegments();
	std::map<std::string, SegmentPair>& getFixedSegments();

	const std::map<std::string, SegmentPair>& getSegments() const;
	const std::map<std::string, SegmentPair>& getFixedSegments() const;

	tf2_ros::StaticTransformBroadcaster& getStaticTfBroadcaster();

#if !ROS_VERSION_MINIMUM(1, 15, 0)
	virtual void addChildren(const KDL::SegmentMap::const_iterator segment);
	
  /** \brief The underlying (hacked) publisher. */
  RobotStatePublisher *publisher;
#endif
};
}

#endif //DYNAMIC_ROBOT_STATE_PUBLISHER_ROBOT_STATE_PUBLISHER_H
