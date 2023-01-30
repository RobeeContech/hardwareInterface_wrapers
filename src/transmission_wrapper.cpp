/*
 Copyright (c) 2023 RobeeContech, Inc.
 Author: Yoav Fekete
*/
#include "hardware_interface_wrappers/transmission_wrapper.hpp"

const rclcpp::Logger logger = rclcpp::get_logger("TransmissionWrapper");

namespace hardware_interface_wrappers
{
  
  hardware_interface::CallbackReturn TransmissionWrapper::on_init(
    const hardware_interface::HardwareInfo & info)
  {
    RCLCPP_INFO_STREAM(logger,"Start init of Transmission Wrapper");
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        RCLCPP_FATAL(logger,"Init Of System Interface Failed!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = info;
    
    RCLCPP_INFO_STREAM(logger,"Finish init TransmissionWrapper ");
    
    return hardware_interface::CallbackReturn::SUCCESS;
  }


  std::vector<hardware_interface::StateInterface> TransmissionWrapper::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
   
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> TransmissionWrapper::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
   
    return command_interfaces;
  }

  hardware_interface::CallbackReturn TransmissionWrapper::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger,"Activating");
    return hardware_interface::CallbackReturn::SUCCESS;
    
  }

  hardware_interface::CallbackReturn TransmissionWrapper::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger,"Diactivating");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type TransmissionWrapper::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type TransmissionWrapper::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
   
    return hardware_interface::return_type::OK;
  }

   
    hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> & stop_interfaces) 
    {
      return hardware_interface::return_type::OK;
    }
  
    
    hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> & stop_interfaces) 
    {
      return hardware_interface::return_type::OK;
    }


}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hardware_interface_wrappers::TransmissionWrapper, hardware_interface::SystemInterface)