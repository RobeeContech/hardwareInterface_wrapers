// Copyright 2023 RobeeRobotics LTD
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
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

const rclcpp::Logger logger = rclcpp::get_logger("TestableJointSystem");

namespace test_hardware_interface_wrappers
{
    class TestableJointSystem : public hardware_interface::SystemInterface
    {
        public:
            
            CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
            {
                RCLCPP_INFO_STREAM(logger,"Start init of TestableJointSystem");
                if (hardware_interface::SystemInterface::on_init(info) !=
                    hardware_interface::CallbackReturn::SUCCESS)
                {
                    RCLCPP_FATAL(logger,"Init Of System Interface Failed!");
                    return hardware_interface::CallbackReturn::ERROR;
                }

                states_postion_interface_index_.resize(info_.joints.size());
                hw_joint_states_.resize(info_.joints.size());
                for(uint j = 0; j < info_.joints.size(); j++)
                    hw_joint_states_[j].resize(info_.joints[j].state_interfaces.size(),
                        std::numeric_limits<double>::quiet_NaN());

                commands_postion_interface_index_.resize(info_.joints.size());
                hw_joint_commands_.resize(info_.joints.size());
                for(uint j = 0; j < info_.joints.size(); j++)
                    hw_joint_commands_[j].resize(info_.joints[j].command_interfaces.size(),
                std::numeric_limits<double>::quiet_NaN());
                return CallbackReturn::SUCCESS;
            }

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override
            {
                std::vector<hardware_interface::StateInterface> state_interfaces;
                states_postion_interface_index_.clear();
                int  found_pos_interface_index, found_vel_interface_index;
                // export joint state interface
                for(uint j = 0; j < info_.joints.size(); j++){
                    found_pos_interface_index = -1;
                    found_vel_interface_index = -1;
                    for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++){
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.joints[j].name, info_.joints[j].state_interfaces[i].name, &hw_joint_states_[j][i]));
                        if (info_.joints[j].state_interfaces[i].name == "position")
                        {
                            found_pos_interface_index = i;
                        }   
                        if (info_.joints[j].state_interfaces[i].name == "velocity")
                        {
                            found_vel_interface_index = i;
                        }      
                    }
                    if (found_pos_interface_index>=0)
                        states_postion_interface_index_.push_back(found_pos_interface_index);
                    else 
                        RCLCPP_ERROR_STREAM(logger,"not found states_postion_interface_index_ for joint =  "<<j << " with name " << info_.joints[j].name);

                    if (found_vel_interface_index>=0)
                        states_velocity_interface_index_.push_back(found_vel_interface_index);
                    else 
                        RCLCPP_ERROR_STREAM(logger,"not found states_velocity_interface_index_ for joint =  "<<j << " with name " << info_.joints[j].name);
                }
                return state_interfaces;
            }

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
            {
                std::vector<hardware_interface::CommandInterface> command_interfaces;
                commands_postion_interface_index_.clear();
                int  found_pos_interface_index,  found_vel_interface_index;
                // export joint command interface
                std::vector<double> test;
                for(uint j = 0; j < info_.joints.size(); j++){
                    found_pos_interface_index = -1;
                    found_vel_interface_index = -1;
                    for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++){
                        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                            info_.joints[j].name, info_.joints[j].command_interfaces[i].name, &hw_joint_commands_[j][i]));
                        if (info_.joints[j].command_interfaces[i].name == "position")
                            found_pos_interface_index = i;
                        else if (info_.joints[j].command_interfaces[i].name == "velocity")
                            found_vel_interface_index = i;
                    }
                    if (found_pos_interface_index >= 0 )
                         commands_postion_interface_index_.push_back(found_pos_interface_index);
                    else 
                        RCLCPP_ERROR_STREAM(logger,"not found commands_postion_interface_index_ for joint =  "<<j << " with name " << info_.joints[j].name);
                    if (found_vel_interface_index >= 0 )  
                        commands_velocity_interface_index_.push_back(found_vel_interface_index);                        
                    else 
                        RCLCPP_ERROR_STREAM(logger,"not found commands_velocity_interface_index_ for joint =  "<<j << " with name " << info_.joints[j].name);
                }
                return command_interfaces;
            }


            std::string get_name() const override { return "TestableJointSystem"; }

            hardware_interface::return_type read(
                const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
            {
                //copy joint pos command to joint pos state 
                for (long unsigned int j=0; j < hw_joint_states_.size(); j++) 
                {
                    if (states_postion_interface_index_[j]>=0 &&  commands_postion_interface_index_[j]>=0)
                    {
                        hw_joint_states_[j][states_postion_interface_index_[j]] = 
                                                hw_joint_commands_[j][commands_postion_interface_index_[j]];

                        hw_joint_states_[j][states_velocity_interface_index_[j]] = 
                                                hw_joint_commands_[j][commands_velocity_interface_index_[j]];

                    }
                }
                return hardware_interface::return_type::OK;
            }

            hardware_interface::return_type write(
                const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
            {
                return hardware_interface::return_type::OK;
            }

        private:
            std::vector<std::vector<double>>    hw_joint_commands_;
            std::vector<std::vector<double>>    hw_joint_states_;

            std::vector<int>                    states_postion_interface_index_;
            std::vector<int>                    commands_postion_interface_index_;

            std::vector<int>                    states_velocity_interface_index_;
            std::vector<int>                    commands_velocity_interface_index_;

            
    };
}  // namespace mooc_components



#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  test_hardware_interface_wrappers::TestableJointSystem, hardware_interface::SystemInterface)