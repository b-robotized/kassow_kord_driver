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

#include <cmath>

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

std::string KassowKordHardwareInterface::motion_flags_to_string(unsigned int flags) const
{
  std::string s = "";
  if (flags & MOTION_FLAG_STANDBY)
  {
    s += "STANDBY ";
  }
  if (flags & MOTION_FLAG_TRACKING)
  {
    s += "TRACKING ";
  }
  if (flags & MOTION_FLAG_TERMINATED)
  {
    s += "TERMINATED ";
  }
  if (flags & MOTION_FLAG_HALT)
  {
    s += "HALT ";
  }
  if (flags & MOTION_FLAG_SYNC)
  {
    s += "SYNC ";
  }
  if (flags & MOTION_FLAG_SUSPENDED)
  {
    s += "SUSPENDED ";
  }
  if (flags & MOTION_FLAG_OFFLINE)
  {
    s += "OFFLINE ";
  }
  if (flags & MOTION_FLAG_INIT)
  {
    s += "INIT ";
  }
  if (flags & MOTION_FLAG_REINIT)
  {
    s += "REINIT ";
  }
  if (flags & MOTION_FLAG_BACKDRIVE)
  {
    s += "BACKDRIVE ";
  }
  if (flags & MOTION_FLAG_PAUSED)
  {
    s += "PAUSED ";
  }
  if (flags & MOTION_FLAG_MAINTENANCE)
  {
    s += "MAINTENANCE ";
  }
  if (flags & MOTION_FLAG_VELOCITYCTL)
  {
    s += "VELOCITYCTL ";
  }
  if (flags & MOTION_FLAG_ARTOACTIVE)
  {
    s += "ARTOACTIVE ";
  }
  return s.empty() ? "NONE" : s;
}

// bool KassowKordHardwareInterface::is_robot_stationary() const
// {
//   const double VELOCITY_THRESHOLD = 1e-4; // Adjust epsilon based on sensor noise
//   for (size_t i = 0; i < KORD_JOINT_COUNT; ++i) {
//     if (std::abs(velocity_states[i]) > VELOCITY_THRESHOLD) {
//       return false;
//     }
//   }
//   return true;
// }

double KassowKordHardwareInterface::get_max_joint_velocity() const
{
  double max_vel = 0.0;
  for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
  {
    double v = std::abs(velocity_states[i]);
    if (v > max_vel)
    {
      max_vel = v;
    }
  }
  return max_vel;
}

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

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Called from the RT loop: pause the services thread, but do not join it.
  if (ros_services_)
  {
    ros_services_->pause();
  }
  digital_output_service_.abort();
  digital_output_service_.reset();
  kord_->disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up KassowKordHardwareInterface...");

  if (!clean_alarms())
  {
    RCLCPP_DEBUG(get_logger(), "clean_alarms() returned false during cleanup");
  }

  if (ros_services_)
  {
    ros_services_->pause();
  }
  kord_->disconnect();

  RCLCPP_INFO(get_logger(), "Successfully cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down KassowKordHardwareInterface...");
  if (ros_services_)
  {
    ros_services_->stop();
    ros_services_.reset();
  }
  kord_->disconnect();
  RCLCPP_INFO(get_logger(), "Successfully shut down!");
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

  // The services thread is created once and runs until on_shutdown. It starts paused.
  if (!ros_services_)
  {
    update_rate_ = info_.rw_rate > 0 ? info_.rw_rate : update_rate_;
    ros_services_ = std::make_unique<KassowRosServices>(info_.name + "_services", update_rate_);
    ros_services_->start();
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

  unsigned int motion_flags_curr = rcv_iface_->getMotionFlags();
  if (motion_flags_curr != motion_flags_prev_)
  {
    RCLCPP_INFO(
      get_logger(), "Motion state: %s -> %s", motion_flags_to_string(motion_flags_prev_).c_str(),
      motion_flags_to_string(motion_flags_curr).c_str());
    motion_flags_prev_ = motion_flags_curr;
  }

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

  // Read the current outputs and use them as the initial output commands
  digital_outputs_sent_ = static_cast<uint64_t>(rcv_iface_->getDigitalOutput());
  digital_output_service_.reset();
  for (size_t i = 0; i < KORD_OUTPUT_COUNT; ++i)
  {
    if (!digital_outputs_itfs_[i].empty())
    {
      const double value =
        bit_helpers::bit_to_double(bit_helpers::is_set(digital_outputs_sent_, i));
      set_state(digital_outputs_itfs_[i], value);
      set_command(digital_outputs_itfs_[i], value);
    }
  }
  RCLCPP_INFO(get_logger(), "Initial digital output state: 0x%016lX", digital_outputs_sent_);

  skip_jcontrol_ = false;
  if (ros_services_)
  {
    ros_services_->resume();
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool KassowKordHardwareInterface::check_and_log_safety_violations()
{
  // 1. Fetch current status flags
  unsigned int motion_flags = rcv_iface_->getMotionFlags();
  unsigned int safety_flags = rcv_iface_->getRobotSafetyFlags();
  unsigned int button_flags = rcv_iface_->getButtonFlags();
  unsigned int hw_flags = rcv_iface_->getHWFlags();
  uint32_t alarm_state = rcv_iface_->systemAlarmState();

  if (ros_services_)
  {
    ros_services_->robot_state().motion_flags.store(motion_flags, std::memory_order_relaxed);
    ros_services_->robot_state().safety_flags.store(safety_flags, std::memory_order_relaxed);
    ros_services_->robot_state().button_flags.store(button_flags, std::memory_order_relaxed);
    ros_services_->robot_state().hw_flags.store(hw_flags, std::memory_order_relaxed);
    ros_services_->robot_state().alarm_state.store(alarm_state, std::memory_order_relaxed);
  }

  // 2. Evaluate individual triggers using the local enums
  bool m_halt = motion_flags & MotionFlags::MOTION_FLAG_HALT;
  bool m_susp = motion_flags & MotionFlags::MOTION_FLAG_SUSPENDED;
  bool s_estop = safety_flags & SafetyFlags::SAFETY_FLAG_ESTOP;
  bool s_pstop = safety_flags & SafetyFlags::SAFETY_FLAG_PSTOP;

  bool b_estop = button_flags & ButtonFlags::BUTTONS_FLAG_ESTOP;
  bool hw_estop = hw_flags & HWFlags::HW_FLAG_IOB_ESTOP_STALL;

  // Extract packed Category (bits 0-3) and Context (bits 4-7)
  uint8_t current_cat = alarm_state & 0x0F;
  uint8_t current_cntxt = (alarm_state >> 4) & 0x0F;

  bool a_ctx_estop = (current_cntxt == SystemAlarmContext::CNTXT_ESTOP);
  bool a_cat_safety = (current_cat == SystemAlarmCategory::CAT_SAFETY_EVENT);

  // 3. Trigger condition and format comprehensive log
  if (m_halt || m_susp || s_estop || s_pstop || b_estop || hw_estop || a_ctx_estop || a_cat_safety)
  {
    std::string log_msg = "Deactivating. FLags: ";

    if (m_halt || m_susp)
    {
      log_msg += "Motion: ";
      if (m_halt)
      {
        log_msg += "HALT";
      }
      if (m_halt && m_susp)
      {
        log_msg += ", ";
      }
      if (m_susp)
      {
        log_msg += "SUSPENDED";
      }
      log_msg += ". ";
    }

    if (s_estop || s_pstop)
    {
      log_msg += "SafetyFlags: ";
      if (s_estop)
      {
        log_msg += "ESTOP";
      }
      if (s_estop && s_pstop)
      {
        log_msg += ", ";
      }
      if (s_pstop)
      {
        log_msg += "PSTOP";
      }
      log_msg += ". ";
    }

    if (b_estop)
    {
      log_msg += "ButtonFlags: ESTOP. ";
    }
    if (hw_estop)
    {
      log_msg += "HWFlags: IOB_ESTOP_STALL. ";
    }

    if (a_ctx_estop || a_cat_safety)
    {
      log_msg += "SystemAlarm: ";
      if (a_ctx_estop)
      {
        log_msg += "CNTXT_ESTOP";
      }
      if (a_ctx_estop && a_cat_safety)
      {
        log_msg += ", ";
      }
      if (a_cat_safety)
      {
        log_msg += "CAT_SAFETY_EVENT";
      }
      log_msg += ". ";
    }

    RCLCPP_ERROR(get_logger(), "%s", log_msg.c_str());
    return true;  // Violation detected
  }

  return false;  // No violations
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (ros_services_)
  {
    ros_services_->pause();
  }
  digital_output_service_.abort();
  digital_output_service_.reset();

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

  unsigned int motion_flags_curr = rcv_iface_->getMotionFlags();
  if (motion_flags_curr != motion_flags_prev_)
  {
    RCLCPP_INFO(
      get_logger(), "Motion state: %s -> %s", motion_flags_to_string(motion_flags_prev_).c_str(),
      motion_flags_to_string(motion_flags_curr).c_str());
    motion_flags_prev_ = motion_flags_curr;
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

  read_digital_io();
  update_digital_output_request();

  if (rcv_iface_->systemAlarmState())
  {
    // write() deactivates the hardware on safety violations
    return hardware_interface::return_type::OK;
  }

  process_ros_services();
  return hardware_interface::return_type::OK;
}

void KassowKordHardwareInterface::read_digital_io()
{
  const auto digital_input = static_cast<uint64_t>(rcv_iface_->getDigitalInput());
  for (size_t i = 0; i < KORD_INPUT_COUNT; ++i)
  {
    if (!digital_inputs_itfs_[i].empty())
    {
      set_state(
        digital_inputs_itfs_[i], bit_helpers::bit_to_double(bit_helpers::is_set(digital_input, i)));
    }
  }

  const auto digital_output = static_cast<uint64_t>(rcv_iface_->getDigitalOutput());
  for (size_t i = 0; i < KORD_OUTPUT_COUNT; ++i)
  {
    if (!digital_outputs_itfs_[i].empty())
    {
      set_state(
        digital_outputs_itfs_[i],
        bit_helpers::bit_to_double(bit_helpers::is_set(digital_output, i)));
    }
  }
}

// IO request: monitored with the request ID. directJControl is not paused.
void KassowKordHardwareInterface::update_digital_output_request()
{
  if (digital_output_service_.get_state() == KordServiceState::DISPATCHED)
  {
    digital_output_service_.poll(*rcv_iface_);
  }
  if (!digital_output_service_.is_finished())
  {
    return;
  }

  const auto rid = digital_output_service_.get_request_rid();
  switch (digital_output_service_.release())
  {
    case KordServiceState::SUCCESS:
      digital_outputs_sent_ |= digital_output_service_.get_enable_mask();
      digital_outputs_sent_ &= ~digital_output_service_.get_disable_mask();
      RCLCPP_INFO(get_logger(), "Digital output request %ld finished.", rid);
      break;
    case KordServiceState::TIMEOUT:
      RCLCPP_ERROR(get_logger(), "Digital output request %ld timed out.", rid);
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Digital output request %ld failed.", rid);
      break;
  }
}

void KassowKordHardwareInterface::process_ros_services()
{
  skip_jcontrol_ = false;
  if (!ros_services_)
  {
    return;
  }

  // GetPayload: read from memory, updated by fetchData() in this cycle.
  ros_services_->get_payload().serve(*rcv_iface_);

  // SetLoad: monitored with the command token. directJControl is paused until it finishes.
  auto & set_load = ros_services_->set_load();
  const auto state = set_load.get_state();
  if (state == KordServiceState::REQUESTED)
  {
    const unsigned int flags = rcv_iface_->getMotionFlags();
    const bool is_standby = (flags & MotionFlags::MOTION_FLAG_STANDBY) != 0;
    const bool is_init = (flags & MotionFlags::MOTION_FLAG_INIT) != 0;
    const double max_v = get_max_joint_velocity();
    const double VELOCITY_THRESHOLD = 1e-4;

    if (max_v > VELOCITY_THRESHOLD)
    {
      RCLCPP_WARN(get_logger(), "SetLoad: robot is moving (max_vel: %e). Rejecting.", max_v);
      set_load.abort();
    }
    else if (is_standby || is_init)
    {
      skip_jcontrol_ = true;
      RCLCPP_INFO(
        get_logger(), "SetLoad: dispatching. Motion flags: [%s]",
        motion_flags_to_string(flags).c_str());
      set_load.dispatch(*ctl_iface_);
    }
    else
    {
      skip_jcontrol_ = true;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500, "SetLoad: waiting for STANDBY. Motion flags: [%s]",
        motion_flags_to_string(flags).c_str());
    }
  }
  else if (state == KordServiceState::DISPATCHED)
  {
    skip_jcontrol_ = true;
    set_load.poll(*rcv_iface_);
  }
}

hardware_interface::return_type KassowKordHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (check_and_log_safety_violations())
  {
    return hardware_interface::return_type::DEACTIVATE;
  }

  for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
  {
    position_cmds[i] = get_command(joint_position_itfs_[i]);
    velocity_cmds[i] = get_command(joint_velocity_itfs_[i]);
    acceleration_cmds[i] = get_command(joint_acceleration_itfs_[i]);
  }

  if (skip_jcontrol_)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500, "SetLoad in progress: skipping directJControl.");
  }
  else if (!ctl_iface_->directJControl(position_cmds, velocity_cmds, acceleration_cmds))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Kord failed to write joint positions");
    return hardware_interface::return_type::ERROR;
  }

  write_digital_outputs();
  return hardware_interface::return_type::OK;
}

void KassowKordHardwareInterface::write_digital_outputs()
{
  // One IO request at a time
  if (digital_output_service_.get_state() != KordServiceState::IDLE)
  {
    return;
  }

  uint64_t desired = digital_outputs_sent_;
  for (size_t i = 0; i < KORD_OUTPUT_COUNT; ++i)
  {
    if (digital_outputs_itfs_[i].empty())
    {
      continue;
    }
    const double cmd = get_command(digital_outputs_itfs_[i]);
    if (std::isnan(cmd))
    {
      continue;  // no command: keep the output as it is
    }
    desired = bit_helpers::double_to_bit(cmd) ? bit_helpers::set(desired, i)
                                              : bit_helpers::clear(desired, i);
  }

  const uint64_t enable_mask = bit_helpers::get_bits_to_enable(desired, digital_outputs_sent_);
  const uint64_t disable_mask = bit_helpers::get_bits_to_disable(desired, digital_outputs_sent_);
  if (!enable_mask && !disable_mask)
  {
    return;
  }

  // KORD takes either enabled or disabled ports per request. Enable first, disable in the next
  // request after this one is finished.
  const size_t timeout_ticks = update_rate_;  // 1 s
  if (enable_mask)
  {
    digital_output_service_.dispatch(*ctl_iface_, enable_mask, 0, timeout_ticks);
  }
  else
  {
    digital_output_service_.dispatch(*ctl_iface_, 0, disable_mask, timeout_ticks);
  }
  RCLCPP_INFO(
    get_logger(), "Digital output request %ld sent: enable 0x%016lX, disable 0x%016lX",
    digital_output_service_.get_request_rid(), digital_output_service_.get_enable_mask(),
    digital_output_service_.get_disable_mask());
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
