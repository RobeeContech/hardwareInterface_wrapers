// Copyright 2022 RobeeRobotics LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>
#include "testable_joint_system_interface.cpp"
#include "hardware_interface/component_parser.hpp"
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using testing::SizeIs;


// class FriendTestableJointSystem : public test_hardware_interface_wrappers::TestableJointSystem
// {
//   FRIEND_TEST(TestableJointSystemTest, Load);

// };
// class TestTestableJointSystem: public ::testing::Test
// {
//    public:

//     static void SetUpTestSuite()    { rclcpp::init(0, nullptr); }
//     static void TearDownTestSuite() { rclcpp::shutdown(); }

//     void virtual SetUp() 
//     {
//     }
//     void virtual TearDown() { 
//     }
// }

TEST(TestableJointSystemTest, Load)
{
  // Parse transmission info
  std::string urdf_to_test = R"(
    <?xml version="1.0"?>
    <robot name="robot" xmlns="http://www.ros.org">
      <ros2_control name="FullSpec" type="system">
        <hardware>
            <plugin>test_hardware_interface_wrappers/TestableJointSystem</plugin>
        </hardware>
        <joint name="joint1">
          <command_interface name="velocity">
            <param name="min">-0.5</param>
            <param name="max">0.5</param>
          </command_interface>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
          </command_interface>
          <state_interface name="position"/>
        </joint>
      </ros2_control>
    </robot>
    )";

  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);


  test_hardware_interface_wrappers::TestableJointSystem hi;
  ASSERT_EQ(hi.on_init(infos[0]), CallbackReturn::SUCCESS);
}
