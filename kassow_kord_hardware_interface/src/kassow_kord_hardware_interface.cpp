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

#include "kassow_kord_hardware_interface/kassow_kord_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * \file kassow_kord_hardware_interface.cpp
 * \brief Hardware interface for Kassow Kord robots using the kord-api.
 * params:
 *   - ip_address (string, required): IP address of the robot controller.
 *   - port (int, required): Port number for Kord connection.
 *   - session_id (int, required): Kord session ID.
 */
namespace kassow_kord_hardware_interface
{
hardware_interface::CallbackReturn KassowKordHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto & hw_params = info_.hardware_parameters;

  // Get required parameters
  if (hw_params.find("ip_address") != hw_params.end())
  {
    ip_address = hw_params.at("ip_address");
  }
  else
  {
    RCLCPP_FATAL(get_logger(), "Parameter 'ip_address' is required but not provided");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (hw_params.find("port") != hw_params.end())
  {
    port = std::stoi(hw_params.at("port"));
  }
  else
  {
    RCLCPP_FATAL(get_logger(), "Parameter 'port' is required but not provided");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (hw_params.find("session_id") != hw_params.end())
  {
    session_id = std::stoi(hw_params.at("session_id"));
  }
  else
  {
    RCLCPP_FATAL(get_logger(), "Parameter 'session_id' is required but not provided");
    return hardware_interface::CallbackReturn::ERROR;
  }

  waitSync_timeout_ms = static_cast<int>(std::ceil(1000.0 / params.hardware_info.rw_rate));

  if (info_.joints.size() != KORD_JOINT_COUNT)
  {
    RCLCPP_FATAL(
      get_logger(),
      "KassowKordHardwareInterface requires exactly 7 joints defined in the URDF/hardware config.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  size_t joint_index = 0;
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Validate we have one position command interface
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' command interface invalid. Expected exactly three command interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate state interfaces
    int required_interfaces = 0;

    for (const auto & command_interface : joint.command_interfaces)
    {
      if (
        command_interface.name == hardware_interface::HW_IF_POSITION ||
        command_interface.name == hardware_interface::HW_IF_VELOCITY ||
        command_interface.name == hardware_interface::HW_IF_ACCELERATION)
      {
        if (++required_interfaces == 3)
        {
          break;
        }
      }
    }

    if (required_interfaces != 3)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' missing required state interfaces. Expected position, velocity, and "
        "acceleration.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate we have four state interface
    if (joint.state_interfaces.size() != 4)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' state interface invalid. Expected exactly four state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate state interfaces
    required_interfaces = 0;

    for (const auto & state_interface : joint.state_interfaces)
    {
      if (
        state_interface.name == hardware_interface::HW_IF_POSITION ||
        state_interface.name == hardware_interface::HW_IF_VELOCITY ||
        state_interface.name == hardware_interface::HW_IF_ACCELERATION ||
        state_interface.name == hardware_interface::HW_IF_EFFORT)
      {
        if (++required_interfaces == 4)
        {
          break;
        }
      }
    }

    if (required_interfaces != 4)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' missing required state interfaces. Expected position, velocity, acceleration, "
        "and effort.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_position_itfs_[joint_index] = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    joint_velocity_itfs_[joint_index] = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    joint_acceleration_itfs_[joint_index] =
      joint.name + "/" + hardware_interface::HW_IF_ACCELERATION;
    joint_effort_itfs_[joint_index] = joint.name + "/" + hardware_interface::HW_IF_EFFORT;
    joint_index++;
  }

  kord_ = std::shared_ptr<kr2::kord::KordCore>(
    new kr2::kord::KordCore(ip_address, port, session_id, kr2::kord::UDP_CLIENT));

  // Initialize Control and Receiver Interfaces.
  ctl_iface_ = std::make_unique<kr2::kord::ControlInterface>(kord_);
  rcv_iface_ = std::make_unique<kr2::kord::ReceiverInterface>(kord_);

  RCLCPP_INFO(
    get_logger(), "KassowKordHardwareInterface on_init completed for %zu joints", KORD_JOINT_COUNT);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "cleanup KassowKordHardwareInterface...");

  if (!clean_alarms())
  {
    RCLCPP_DEBUG(
      get_logger(), "clean_alarms() returned false during deactivate (continuing cleanup)");
  }

  kord_->disconnect();

  RCLCPP_INFO(get_logger(), "Successfully cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Connecting to Kassow Kord robot at %s...", ip_address.c_str());
  if (!kord_->connect())
  {
    RCLCPP_FATAL(get_logger(), "Failed to connect to Kassow Kord robot.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Reset robot errors and alarms as needed
  try
  {
    if (!clean_alarms())
    {
      RCLCPP_FATAL(get_logger(), "Failed to reset alarms to Kassow Kord robot.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Exception while resetting alarms: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial joint positions and set them as the initial command values
  rcv_iface_->fetchData();
  position_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_ACTUAL_Q);
  velocity_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_ACTUAL_QD);
  acceleration_states =
    rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_SENSED_ACCELERATIONS);
  torque_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_SENSED_TRQ);
  
  for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
  {
    set_state(joint_position_itfs_[i], position_states[i]);
    set_state(joint_velocity_itfs_[i], velocity_states[i]);
    set_state(joint_acceleration_itfs_[i], acceleration_states[i]);
    set_state(joint_effort_itfs_[i], torque_states[i]);

    set_command(joint_position_itfs_[i], position_states[i]);
    set_command(joint_velocity_itfs_[i], velocity_states[i]);
    set_command(joint_acceleration_itfs_[i], acceleration_states[i]);
  }

  RCLCPP_INFO(get_logger(), "KassowKordHardwareInterface configured and connected");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

// read: fetch current joint states from the robot and populate ROS2 control state buffers.
hardware_interface::return_type KassowKordHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!kord_->waitSync(std::chrono::milliseconds(waitSync_timeout_ms)))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Kord waitSync timeout");
    return hardware_interface::return_type::ERROR;
  }

  rcv_iface_->fetchData();
  position_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_ACTUAL_Q);
  velocity_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_ACTUAL_QD);
  acceleration_states =
    rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_SENSED_ACCELERATIONS);
  torque_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_SENSED_TRQ);

  for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
  {
    set_state(joint_position_itfs_[i], position_states[i]);
    set_state(joint_velocity_itfs_[i], velocity_states[i]);
    set_state(joint_acceleration_itfs_[i], acceleration_states[i]);
    set_state(joint_effort_itfs_[i], torque_states[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KassowKordHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
  {
    position_cmds[i] = get_command(joint_position_itfs_[i]);
    velocity_cmds[i] = get_command(joint_velocity_itfs_[i]);
    acceleration_cmds[i] = get_command(joint_acceleration_itfs_[i]);
  }

  if (!ctl_iface_->directJControl(position_cmds, velocity_cmds, acceleration_cmds))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Kord failed to write joint positions");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// clean all alarms
bool KassowKordHardwareInterface::clean_alarms()
{
  std::map<kr2::kord::ControlInterface::EClearRequest, std::string> commands_mapped = {
    {kr2::kord::ControlInterface::EClearRequest::CLEAR_HALT, "CLEAR_HALT"},
    {kr2::kord::ControlInterface::EClearRequest::CBUN_EVENT, "CBUN_EVENT"},
    {kr2::kord::ControlInterface::EClearRequest::CONTINUE_INIT, "CONTINUE_INIT"},
    {kr2::kord::ControlInterface::EClearRequest::UNSUSPEND, "UNSUSPEND"}};

  for (const auto & kv : commands_mapped)
  {
    const auto command = kv.first;
    const auto & name = kv.second;
    int64_t token = ctl_iface_->clearAlarmRequest(command);

    RCLCPP_INFO(
      get_logger(), "%s command sent with token: %ld", name.c_str(), static_cast<int64_t>(token));

    // Poll for command status -- blocking
    while (rcv_iface_->getCommandStatus(token) == -1)
    {
      if (!kord_->waitSync(std::chrono::milliseconds(10), kr2::kord::F_SYNC_FULL_ROTATION))
      {
        RCLCPP_ERROR(get_logger(), "Sync wait timed out, exiting.");
        return false;
      }

      rcv_iface_->fetchData();
    }

    // Retrieve and log the command status
    auto status = rcv_iface_->getCommandStatus(token);
    if (status != -1)
    {
      RCLCPP_INFO(get_logger(), "%s command status: %d", name.c_str(), static_cast<int>(status));
    }
    else
    {
      RCLCPP_WARN(get_logger(), "%s command status remains unknown.", name.c_str());
      return false;
    }
  }

  return true;
}

}  // namespace kassow_kord_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kassow_kord_hardware_interface::KassowKordHardwareInterface, hardware_interface::SystemInterface)
