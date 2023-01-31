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

    std::string wrapped_interface_name ="";
    for (auto x : info.hardware_parameters)
    {
       if (x.first == "wrapped_interface") wrapped_interface_name = x.second;

    }
    if (wrapped_interface_name == "")
    {
        RCLCPP_FATAL(logger,"Init Of transmission_wrapper Failed.  wrapped_interface paramater not specfied");
        return hardware_interface::CallbackReturn::ERROR;
    }

    loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::SystemInterface>>(
                "hardware_interface","hardware_interface::SystemInterface");
    try
    {    
      wrapped_interface_ = std::unique_ptr<hardware_interface::SystemInterface>(
          loader_->createUnmanagedInstance(wrapped_interface_name));
    }
    catch(const std::exception& e)
    {
      RCLCPP_FATAL_STREAM(logger,"Init Of transmission_wrapper Failed.  exception thrown while running load_hardware. "<<e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
   
    if (!wrapped_interface_ )
    {
       RCLCPP_FATAL(logger,"Init Of transmission_wrapper Failed.  wrapped_interface = null!!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    auto res = wrapped_interface_->on_init(info);

    RCLCPP_INFO_STREAM(logger,"Finish init TransmissionWrapper ");
    return res;
  }


  std::vector<hardware_interface::StateInterface> TransmissionWrapper::export_state_interfaces()
  {
    return wrapped_interface_->export_state_interfaces();
  }

  std::vector<hardware_interface::CommandInterface> TransmissionWrapper::export_command_interfaces()
  {
    return wrapped_interface_->export_command_interfaces();
  }

  hardware_interface::CallbackReturn TransmissionWrapper::on_activate(
    const rclcpp_lifecycle::State & previous_state)
  {
    return wrapped_interface_->on_activate(previous_state);
  }

  hardware_interface::CallbackReturn TransmissionWrapper::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
  {
    return wrapped_interface_->on_deactivate(previous_state);
  }

  hardware_interface::return_type TransmissionWrapper::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    return wrapped_interface_->read(time,period);
  }

  hardware_interface::return_type TransmissionWrapper::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    return wrapped_interface_->write(time,period);
  }

   
    hardware_interface::return_type TransmissionWrapper::prepare_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> & stop_interfaces) 
    {
      return wrapped_interface_->prepare_command_mode_switch(start_interfaces,stop_interfaces);
    }
  
    
    hardware_interface::return_type TransmissionWrapper::perform_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> & stop_interfaces) 
    {
      return wrapped_interface_->perform_command_mode_switch(start_interfaces,stop_interfaces);
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hardware_interface_wrappers::TransmissionWrapper, hardware_interface::SystemInterface)