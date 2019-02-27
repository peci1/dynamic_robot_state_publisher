#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <dynamic_robot_state_publisher/joint_state_listener.h>

int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "robot_state_publisher");
  NodeHandle node;

  // gets the location of the robot description on the parameter server
  urdf::Model model;
  if (!model.initParam("robot_description"))
    return 1;

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return 1;
  }

  MimicMap mimic;

  for(const auto& joint : model.joints_) {
    if(joint.second->mimic) {
      mimic.insert(make_pair(joint.first, joint.second->mimic));
    }
  }

  robot_state_publisher::DynamicJointStateListener state_publisher(tree, mimic, model);
  ros::spin();

  return 0;
}
