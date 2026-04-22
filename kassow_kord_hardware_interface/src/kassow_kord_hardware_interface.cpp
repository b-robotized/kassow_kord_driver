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
#include "kassow_kord_hardware_interface/bit_helpers.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * \file kassow_kord_hardware_interface.cpp
 * \brief Hardware interface for Kassow Kord robots using the kord-api.
 * params:
 *   - ip_address (string, required): IP address of the robot controller.
 *   - port (int, required): Port number for Kord connection.
 *   - session_id (int, required): Kord session ID.
 *   - waitSync_timeout_ms (int, required): Kord session ID.
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

  if (hw_params.find("waitSync_timeout_ms") != hw_params.end())
  {
    waitSync_timeout_ms = std::stoi(hw_params.at("waitSync_timeout_ms"));
  }
  else
  {
    RCLCPP_FATAL(get_logger(), "Parameter 'waitSync_timeout_ms' is required but not provided");
    return hardware_interface::CallbackReturn::ERROR;
  }

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

  // TODO(habartakh): Search for more exceptions/error scenarios
  //  Populate the GPIO interfaces vectors
  size_t bit_index;
  std::string io_type;
  RCLCPP_INFO(get_logger(), "Found %zu GPIO component(s) in hardware info", info_.gpios.size());
  for (const hardware_interface::ComponentInfo & gpio : info_.gpios)
  {
    RCLCPP_INFO(
      get_logger(), "GPIO component: '%s' — %zu state interface(s)", gpio.name.c_str(),
      gpio.state_interfaces.size());
    for (const auto & state_io : gpio.state_interfaces)
    {
      RCLCPP_INFO(get_logger(), "  state_interface name: '%s'", state_io.name.c_str());
      if (state_io.parameters.count("bit_index"))
      {
        bit_index = stoi(state_io.parameters.at("bit_index"));
        RCLCPP_INFO(get_logger(), "  bit_index: %zu", bit_index);

        if (state_io.parameters.count("io_type"))
        {
          io_type = state_io.parameters.at("io_type");
          RCLCPP_INFO(get_logger(), "  io_type: '%s'", io_type.c_str());
          if (io_type == "input")
          {
            if (bit_index >= KORD_INPUT_COUNT)
            {
              RCLCPP_ERROR(
                get_logger(),
                "state_interface '%s' has out-of-range bit_index %zu for input (max %zu)",
                state_io.name.c_str(), bit_index, KORD_INPUT_COUNT - 1);
              return hardware_interface::CallbackReturn::ERROR;
            }
            digital_inputs_itfs_[bit_index] = gpio.name + "/" + state_io.name;
            RCLCPP_INFO(
              get_logger(), "Current gpio state interface for input %lu is: %s", bit_index,
              digital_inputs_itfs_[bit_index].c_str());
          }
          else if (io_type == "output")
          {
            if (bit_index >= KORD_OUTPUT_COUNT)
            {
              RCLCPP_ERROR(
                get_logger(),
                "state_interface '%s' has out-of-range bit_index %zu for output (max %zu)",
                state_io.name.c_str(), bit_index, KORD_OUTPUT_COUNT - 1);
              return hardware_interface::CallbackReturn::ERROR;
            }
            digital_outputs_itfs_[bit_index] = gpio.name + "/" + state_io.name;
            RCLCPP_INFO(
              get_logger(), "Current gpio state interface for output %lu is: %s", bit_index,
              digital_outputs_itfs_[bit_index].c_str());
          }
          else
          {
            RCLCPP_ERROR(
              get_logger(), "Unknown io_type for state_interface '%s'", state_io.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
          }
        }
        else
        {
          RCLCPP_ERROR(
            get_logger(), "state_interface '%s' is missing required parameter 'io_type'",
            state_io.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
      else
      {
        RCLCPP_ERROR(
          get_logger(), "state_interface '%s' is missing required parameter 'bit_index'",
          state_io.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
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
  RCLCPP_INFO(
    get_logger(), "Connecting to Kassow Kord robot at ip %s | port %d | session id %d",
    ip_address.c_str(), port, session_id);
  if (!kord_->connect())
  {
    RCLCPP_FATAL(get_logger(), "Failed to connect to Kassow Kord robot.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "KassowKordHardwareInterface configured and connected");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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

  // Read initial Outputs and set them as the initial command values
  prev_io_cmd_sent = rcv_iface_->getDigitalOutput();
  RCLCPP_INFO(get_logger(), "Initial digital output state: 0x%016lX", prev_io_cmd_sent);

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

  if (rcv_iface_->systemAlarmState())
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Alarm detected, deactivating...");
    return hardware_interface::return_type::ERROR;
  }

  position_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_ACTUAL_Q);
  velocity_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_ACTUAL_QD);
  acceleration_states =
    rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_ACTUAL_QDD);
  torque_states = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_SENSED_TRQ);

  for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
  {
    set_state(joint_position_itfs_[i], position_states[i]);
    set_state(joint_velocity_itfs_[i], velocity_states[i]);
    set_state(joint_acceleration_itfs_[i], acceleration_states[i]);
    set_state(joint_effort_itfs_[i], torque_states[i]);
  }

  // Get the Digital Inputs
  // TODO(habartakh): How do we manage a scenario for two enable/disable requests
  const uint64_t digital_input = static_cast<uint64_t>(rcv_iface_->getDigitalInput());
  for (size_t i = 0; i < KORD_INPUT_COUNT; ++i)
  {
    if (digital_inputs_itfs_[i].empty())
    {
      continue;
    }
    const bool bit = bit_helpers::get_bit(digital_input, i);
    const double input_value = bit_helpers::bit_to_double(bit);
    set_state(digital_inputs_itfs_[i], input_value);
  }

  // Get the digital Outputs
  const uint64_t digital_output = static_cast<uint64_t>(rcv_iface_->getDigitalOutput());
  for (size_t i = 0; i < KORD_OUTPUT_COUNT; ++i)
  {
    if (digital_outputs_itfs_[i].empty())
    {
      continue;
    }
    const bool bit = bit_helpers::get_bit(digital_output, i);
    const double output_value = bit_helpers::bit_to_double(bit);
    set_state(digital_outputs_itfs_[i], output_value);
  }

  // Check if the IO write request is being processed or not
  if (ongoing_request_processing)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "Currently processing request with RID %ld ...",
      io_request.request_rid_);

    auto latest_response = rcv_iface_->getLatestRequest();

    // If the 1 second elapsed without confirming reception of the request
    auto time_elapsed = std::chrono::steady_clock::now() - init_time;
    if (time_elapsed > std::chrono::seconds(1))
    {
      RCLCPP_ERROR(get_logger(), "TIMEOUT: Request with RID %ld. ", latest_response.request_rid_);

      pending_enable_mask = 0;
      pending_disable_mask = 0;
      ongoing_request_processing = false;
      // return hardware_interface::return_type::ERROR; stop or not?
    }
    else  // Timeout not reached yet
    {
      // Check if the sent IO request is being evaluated
      if (io_request.request_rid_ != latest_response.request_rid_)
      {
        RCLCPP_INFO(get_logger(), "Latest request does not correspond to sent request...");
      }

      if (latest_response.request_status_ == kr2::kord::protocol::EControlCommandStatus::eSuccess)
      {
        RCLCPP_INFO(
          get_logger(), "SUCCESS: Request with RID %ld, transfer finished.",
          latest_response.request_rid_);

        // Apply only ONE mask on previous command
        bit_helpers::apply_enable(prev_io_cmd_sent, pending_enable_mask);
        bit_helpers::apply_disable(prev_io_cmd_sent, pending_disable_mask);
        pending_enable_mask = 0;
        pending_disable_mask = 0;
        ongoing_request_processing = false;
      }

      if (latest_response.request_status_ == kr2::kord::protocol::EControlCommandStatus::eFailure)
      {
        RCLCPP_ERROR(
          get_logger(), "ERROR: Request with RID %ld, transfer failed.",
          latest_response.request_rid_);

        pending_enable_mask = 0;
        pending_disable_mask = 0;
        ongoing_request_processing = false;

        // return hardware_interface::return_type::ERROR; return error or continue sending commands?
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KassowKordHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Joint Commands
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

  // IO commands

  // Build the desired mask from the command interfaces
  uint64_t desired_mask = 0;
  for (size_t i = 0; i < KORD_OUTPUT_COUNT; ++i)
  {
    if (digital_outputs_itfs_[i].empty())
    {
      continue;
    }
    const double cmd = get_command(digital_outputs_itfs_[i]);

    // if cmd == NaN, build_mask leaves the bit unset (NaN > 0.5 is false)
    desired_mask = bit_helpers::build_mask(desired_mask, i, cmd, prev_io_cmd_sent);
  }

  const bool command_changed = desired_mask != prev_io_cmd_sent;

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000, "desired_mask: 0x%016lX, command_changed: %s", desired_mask,
    command_changed ? "true" : "false");

  // Only send a command if something actually changed AND no request is currently being processed
  if (command_changed && !ongoing_request_processing)
  {
    ongoing_request_processing = true;

    const uint64_t changed = bit_helpers::changed_bits(desired_mask, prev_io_cmd_sent);
    const uint64_t enable_mask = bit_helpers::enable_bits(desired_mask, changed);
    const uint64_t disable_mask = bit_helpers::disable_bits(desired_mask, changed);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "changed: 0x%016lX, enable_mask: 0x%016lX, disable_mask: 0x%016lX", changed, enable_mask,
      disable_mask);

    if (enable_mask && disable_mask)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Both enable_mask (0x%016lX) and disable_mask (0x%016lX) are non-zero. "
        "Simultaneous enable and disable is not supported — skipping IO request.",
        enable_mask, disable_mask);
    }
    else if (enable_mask)  // Set bits to 1
    {
      io_request.asSetIODigitalOut().withEnabledPorts(
        enable_mask);  // Only send enable OR disable per request
      ctl_iface_->transmitRequest(io_request);

      RCLCPP_INFO(
        get_logger(),
        "Sent request with ID: %ld to enable ports corresponding to the following mask: 0x%016lX",
        io_request.request_rid_, enable_mask);

      pending_enable_mask = enable_mask;
      pending_disable_mask = 0;
    }
    else  // Set bits to 0
    {
      io_request.asSetIODigitalOut().withDisabledPorts(disable_mask);
      ctl_iface_->transmitRequest(io_request);

      RCLCPP_INFO(
        get_logger(),
        "Sent request with ID: %ld to disable ports corresponding to the following mask: 0x%016lX",
        io_request.request_rid_, disable_mask);

      pending_disable_mask = disable_mask;
      pending_enable_mask = 0;
    }

    pending_io_rid = io_request.request_rid_;      // track the request
    init_time = std::chrono::steady_clock::now();  // Start the timer
  }
  return hardware_interface::return_type::OK;
}

// clean all alarms
bool KassowKordHardwareInterface::clean_alarms()
{
  rcv_iface_->fetchData();

  if (rcv_iface_->systemAlarmState())
  {
    unsigned int motion_flags = rcv_iface_->getMotionFlags();
    unsigned int safety_flags = rcv_iface_->getRobotSafetyFlags();

    // If nothing to clear, early exit
    if (motion_flags == 0 && safety_flags == 0)
    {
      RCLCPP_INFO(get_logger(), "No errors.");
    }

    RCLCPP_INFO(get_logger(), "Motion flags: %d", motion_flags);
    RCLCPP_INFO(get_logger(), "Robot safety flags: %d", safety_flags);

    if (safety_flags & SafetyFlags::SAFETY_FLAG_USER_CONF_REQ)
    {
      RCLCPP_ERROR(get_logger(), "Errors cannot be cleared. User confirmation required.");
    }

    const bool is_halt = motion_flags & MotionFlags::MOTION_FLAG_HALT;
    const bool is_pstop = safety_flags & SafetyFlags::SAFETY_FLAG_PSTOP;

    if (is_halt && is_pstop)
    {
      RCLCPP_ERROR(get_logger(), "Halt Error.");
    }

    if (motion_flags & MotionFlags::MOTION_FLAG_SUSPENDED)
    {
      RCLCPP_ERROR(get_logger(), "Suspend Error.");
    }

    bool is_cbun = rcv_iface_->systemAlarmState() & kr2::kord::protocol::CAT_CBUN_EVENT;

    // Check for KORD event
    const auto system_events = rcv_iface_->getSystemEvents();
    for (auto & event : system_events)
    {
      if (event.event_group_ == kr2::kord::protocol::eKordEvent)
      {
        is_cbun = true;
        break;
      }
    }

    if (is_cbun)
    {
      RCLCPP_ERROR(get_logger(), "CBun Error.");
    }
  }

  // clear errors
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
