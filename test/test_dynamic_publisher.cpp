/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2022, Czech Technical University in Prague
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* Code inspired by test_two_links_moving_joint.cpp from robot_state_publisher. */
/* Author: Martin Pecka */

#include <string>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/JointState.h>

#include <dynamic_reconfigure/client.h>
#include <dynamic_robot_state_publisher/DynamicRobotStateConfig.h>

using namespace ros;
using namespace tf2_ros;
using namespace robot_state_publisher;

#define EPS 0.01

TEST(DynamicPublisherTest, Base)
{
	ros::NodeHandle nh, pnh("~");
	std::string baseRobotDescription, moreRobotDescription;
	ASSERT_TRUE(nh.getParam("robot_description", baseRobotDescription));
	ASSERT_TRUE(nh.getParam("more_robot_description", moreRobotDescription));

	ROS_INFO("Creating tf listener");
	Buffer buffer;
	TransformListener listener(buffer);
	
	dynamic_reconfigure::Client<DynamicRobotStateConfig> reconfigClient("test", NodeHandle(nh, "dynamic_pub"));
	DynamicRobotStateConfig config;
	auto spin = [](size_t num = 10, double duration = 0.1) {
		for (size_t i = 0; i < num; ++i) {
			ros::spinOnce();
			ros::WallDuration(duration).sleep();
		}
	};
	spin();
	ASSERT_TRUE(reconfigClient.getCurrentConfiguration(config, ros::Duration(1.0)));
	ASSERT_EQ(baseRobotDescription, config.robot_description);

	ROS_INFO("Publishing joint state to robot state publisher");
	ros::Publisher js_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
	ros::WallDuration(0.5).sleep();

	sensor_msgs::JointState js_msg;
	js_msg.name.push_back("joint1");
	js_msg.position.push_back(M_PI);
	
	auto publishJointStates = [&]() {
		for (unsigned int i = 0; i < 10; i++) {
			js_msg.header.stamp = ros::Time::now();
			js_pub.publish(js_msg);
			ros::WallDuration(0.1).sleep();
		}
	};
	publishJointStates();

	auto spinUntilFrame = [&](const std::string& frame, size_t num = 10, double duration = 0.1) {
		for (size_t i = 0; i < num && !buffer.canTransform("link1", frame, Time()); ++i) {
			ros::spinOnce();
			ros::WallDuration(duration).sleep();
		}
	};
	spinUntilFrame("link2");
	EXPECT_FALSE(buffer.canTransform("link1", "link3", Time()));
	ASSERT_TRUE(buffer.canTransform("link1", "link2", Time()));

	auto t = buffer.lookupTransform("link1", "link2", Time());
	EXPECT_NEAR(t.transform.translation.x, 5.0, EPS);
	EXPECT_NEAR(t.transform.translation.y, 0.0, EPS);
	EXPECT_NEAR(t.transform.translation.z, 0.0, EPS);
	
	// Now load the moreRobotDescription and check that link3 is reachable

	buffer.clear();
	
	config.robot_description = moreRobotDescription;
	ASSERT_TRUE(reconfigClient.setConfiguration(config));
	spin(10, 0.01);
	ASSERT_TRUE(reconfigClient.getCurrentConfiguration(config, ros::Duration(1.0)));

	publishJointStates();
	spinUntilFrame("link3");
	ASSERT_TRUE(buffer.canTransform("link1", "link3", Time()));
	ASSERT_TRUE(buffer.canTransform("link1", "link2", Time()));

	EXPECT_EQ(moreRobotDescription, config.robot_description);

	t = buffer.lookupTransform("link1", "link2", Time());
	EXPECT_NEAR(t.transform.translation.x, 5.0, EPS);
	EXPECT_NEAR(t.transform.translation.y, 0.0, EPS);
	EXPECT_NEAR(t.transform.translation.z, 0.0, EPS);

	t = buffer.lookupTransform("link1", "link3", Time());
	EXPECT_NEAR(t.transform.translation.x, 5.0, EPS);
	EXPECT_NEAR(t.transform.translation.y, 5.0, EPS);
	EXPECT_NEAR(t.transform.translation.z, 0.0, EPS);
	
	// And load the base description back, check that link3 is no longer reachable.
	
	buffer.clear();

	config.robot_description = baseRobotDescription;
	ASSERT_TRUE(reconfigClient.setConfiguration(config));
	spin(10, 0.01);
	ASSERT_TRUE(reconfigClient.getCurrentConfiguration(config, ros::Duration(1.0)));

	publishJointStates();
	spinUntilFrame("link2");
	EXPECT_FALSE(buffer.canTransform("link1", "link3", Time()));
	ASSERT_TRUE(buffer.canTransform("link1", "link2", Time()));

	EXPECT_EQ(baseRobotDescription, config.robot_description);
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_dynamic_publisher");
	return RUN_ALL_TESTS();
}
