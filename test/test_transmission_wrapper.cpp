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
#include "hardware_interface_wrappers/transmission_wrapper.hpp"
#include "hardware_interface/component_parser.hpp"

using testing::SizeIs;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

const rclcpp::Logger logger = rclcpp::get_logger("TransmissionWrapperTest");

TEST(TransmissionWrapperTest, FullSpec)
{
  // Parse transmission info
  std::string urdf_to_test = R"(
    <?xml version="1.0"?>
    <robot name="robot" xmlns="http://www.ros.org">
      <ros2_control name="FullSpec" type="system">
        <hardware>
            <plugin>hardware_interface_wrappers/TransmissionWrapper</plugin>
            <param name="wrapped_interface">test_hardware_interface_wrappers/TestableJointSystem</param>
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
        <transmission name="transmission1">
          <plugin>robee_transmission_interface/ScaraTransmission</plugin>
          <actuator name="joint1_motor" role="actuator1">
            <mechanical_reduction>50</mechanical_reduction>
          </actuator>
          <actuator name="joint2_motor" role="actuator2">
            <mechanical_reduction>-50</mechanical_reduction>
          </actuator>
          <joint name="joint1" role="joint1">
            <offset>0.5</offset>
            <mechanical_reduction>2.0</mechanical_reduction>
          </joint>
          <joint name="joint2" role="joint2">
            <offset>-0.5</offset>
            <mechanical_reduction>-2.0</mechanical_reduction>
          </joint>
        </transmission>
      </ros2_control>
    </robot>
    )";

  std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf_to_test);

  hardware_interface_wrappers::TransmissionWrapper hi;
  ASSERT_EQ(hi.on_init(infos[0]),CallbackReturn::SUCCESS);

  // auto si = hi.export_state_interfaces();
  // auto ci = hi.export_command_interfaces();
  // rclcpp::Time t;
  // rclcpp::Duration d(0,0);
  
  // //set cmd
  // hi.write(t,d);
  // hi.read(t,d);

  // //check that stae == cmd
  

  RCLCPP_INFO_STREAM(logger,"Done test of Transmission Wrapper");
}

