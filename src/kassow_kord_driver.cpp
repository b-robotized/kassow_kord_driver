// Copyright (c) 2025 b»robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential
//
// Adapted for <Insert_Company_Name> that received unlimited, worldwide
// use and change right, except distributing this library separately
// of their product.

#include "kassow_kord_driver/kassow_kord_driver.hpp"
#include "kassow_kord_driver/kassow_kord_adapter.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * \file kassow_kord_driver.cpp
 * \brief Hardware interface for Kassow Kord robots using the kord-api.
 * params:
 *   - ip_address (string, required): IP address of the robot controller.
 *   - port (int, optional, default=7582): Port number for Kord connection.
 *   - session_id (int, optional, default=1): Kord session ID.
 *   - waitSync_timeout_ms (int, optional, default=500): Timeout for waitSync in milliseconds.
 */
namespace kassow_kord_driver
{  
hardware_interface::CallbackReturn KassowKordDriver::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto& hw_params = info_.hardware_parameters;
  // Get required parameter IP address
  if (hw_params.find("ip_address") == hw_params.end()) {
    RCLCPP_FATAL(get_logger(), 
                  "Parameter 'ip_address' is required but not provided");
    return hardware_interface::CallbackReturn::ERROR;
  }
  else
  {
    std::string ip_address = hw_params.at("ip_address");
  }
  
  // Get optional parameters with defaults
  int port = 7582;
  if (hw_params.find("port") != hw_params.end()) {
    port = std::stoi(hw_params.at("port"));
  }

  int session_id = 1;
  if (hw_params.find("session_id") != hw_params.end()) {
    session_id = std::stoi(hw_params.at("session_id"));
  }

  int waitSync_timeout_ms = 500;
  if (hw_params.find("waitSync_timeout_ms") != hw_params.end()) {
    waitSync_timeout_ms = std::stoi(hw_params.at("waitSync_timeout_ms"));
  }

  joint_names_.clear();
  if (info_.joints.size() != JOINT_COUNT)
  {
    RCLCPP_FATAL(
      get_logger(), "KassowKordDriver requires exactly 7 joints defined in the URDF/hardware config.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Validate we have one position command interface
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' command interface invalid. Expected exactly one position interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate we have three state interface
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' state interface invalid. Expected exactly three state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate state interfaces
    bool has_position = false;
    bool has_velocity = false;
    bool has_effort = false;
    
    for (const auto & state_interface : joint.state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION)
        has_position = true;
      else if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
        has_velocity = true;
      else if (state_interface.name == hardware_interface::HW_IF_EFFORT)
        has_effort = true;
    }
    
    if (!has_position || !has_velocity || !has_effort)
    {
      RCLCPP_FATAL(
          get_logger(), 
          "Joint '%s' missing required state interfaces. Expected position, velocity, and effort.",
          joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_names_.push_back(joint.name);
  }

  // init adapter with joint count
  kord_adapter_.init(ip_address, port, session_id, waitSync_timeout_ms);

  RCLCPP_INFO(get_logger(), "KassowKordDriver on_init completed for %zu joints", joint_cnt);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Establish connection to the robot using the adapter.
  if (!kord_adapter_.connect())
  {
    RCLCPP_FATAL(get_logger(), "Failed to connect to Kassow Kord robot via KordAdapter.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "KassowKordDriver configured and connected");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset robot errors and alarms as needed 
  try 
  {
    if (!kord_adapter_.reset_alarms())
    {
      RCLCPP_FATAL(get_logger(), "Failed to reset alarms to Kassow Kord robot via KordAdapter.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  catch (const std::exception& e) 
  {
    RCLCPP_ERROR(get_logger(), "Exception while resetting alarms: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial joint positions and set them as the initial command values
  try 
  {
    std::array<double, JOINT_COUNT> initial_positions{};
    kord_adapter_.readJointStates(initial_positions, nullptr, nullptr);
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      RCLCPP_INFO(get_logger(), "Initial position for joint %s: %f", joint_names_[i].c_str(), initial_positions[i]);
      set_command(joint_names_[i] + "/position", initial_positions[i]);
    }
  } 
  catch (const std::exception& e) 
  {
    RCLCPP_ERROR(get_logger(), "Failed to read initial state: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  kord_adapter_.disconnect();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

// read: fetch current joint states from the robot and populate ROS2 control state buffers.
hardware_interface::return_type KassowKordDriver::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!kord_adapter_.waitSync())
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "KordAdapter waitSync timeout");
    return hardware_interface::return_type::ERROR;
  }

  // TODO(yara): isnt there really a better way to do this without creating and copying arrays?
  std::array<double, JOINT_COUNT> position_states{};
  std::array<double, JOINT_COUNT> torque_states{};
  std::array<double, JOINT_COUNT> velocity_states{};
  if (!kord_adapter_.readJointStates(position_states, velocity_states, torque_states))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "KordAdapter failed to read states (using last known).");
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < JOINT_COUNT; ++i)
  {
    RCLCPP_DEBUG(
      get_logger(), "Read joint %zu: pos=%f, vel=%f, effort=%f", i,
      position_states[i], velocity_states[i], torque_states[i]);
    setState(joint_names_[i] + "/position", position_states[i]);
    setState(joint_names_[i] + "/velocity", velocity_states[i]);
    setState(joint_names_[i] + "/effort", torque_states[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KassowKordDriver::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::array<double, JOINT_COUNT> position_cmds{};
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    position_cmds[i] = get_command(joint_names_[i] + "/position");
    RCLCPP_DEBUG(
      get_logger(), "Command for joint %s: %f", joint_names_[i].c_str(), position_cmds[i]);
  }
  if (!kord_adapter_.writeJointPositions(position_cmds))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "KordAdapter failed to write joint positions");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace kassow_kord_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kassow_kord_driver::KassowKordDriver, hardware_interface::SystemInterface)
