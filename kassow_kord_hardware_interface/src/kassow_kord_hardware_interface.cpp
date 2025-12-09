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
 *   - port (int, optional, default=7582): Port number for Kord connection.
 *   - session_id (int, optional, default=1): Kord session ID.
 *   - waitSync_timeout_ms (int, optional, default=500): Timeout for waitSync in milliseconds.
 */
namespace kassow_kord_hardware_interface
{
constexpr double TT_VALUE = 5.0;  // tracking time - TODO(yara): make configurable
constexpr double BT_VALUE = 3.0;  // blend time - TODO(yara): make configurable

// -----------------------------
// KordAdapter method implementations
// -----------------------------
// Constructors
KordAdapter::KordAdapter(
  const std::string & ip_address, int port, int session_id, int waitSync_timeout_ms)
: waitSync_timeout_ms_(waitSync_timeout_ms), connected_(false)
{
  try
  {
    kord_ = std::shared_ptr<kr2::kord::KordCore>(
      new kr2::kord::KordCore(ip_address, port, session_id, kr2::kord::UDP_CLIENT));

    // Initialize Control and Receiver Interfaces.
    ctl_iface_ = std::make_unique<kr2::kord::ControlInterface>(kord_);
    rcv_iface_ = std::make_unique<kr2::kord::ReceiverInterface>(kord_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("KassowKordAdapter"), "KordAdapter initialization failed");
  }

  RCLCPP_INFO(
    rclcpp::get_logger("KassowKordAdapter"),
    "KordAdapter initialized for IP %s:%d with session ID %d", ip_address.c_str(), port,
    session_id);
}

// TODO(yara): put in on cleanup
KordAdapter::~KordAdapter()
{
  // Best-effort disconnect in destructor; swallow exceptions.
  try
  {
    disconnect();
  }
  catch (...)
  {
  }
}

bool KordAdapter::connect()
{
  if (!kord_->connect())
  {
    connected_ = false;
    return connected_;
  }
  connected_ = true;
  return connected_;
}

// clean all alarms
bool KordAdapter::clean_alarms()
{
  if (!connected_)
  {
    return false;
  }

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
      rclcpp::get_logger("KassowKordAdapter"), "%s command sent with token: %lld", name.c_str(),
      static_cast<int64_t>(token));

    // Poll for command status -- blocking
    while (rcv_iface_->getCommandStatus(token) == -1)
    {
      if (!kord_->waitSync(std::chrono::milliseconds(10), kr2::kord::F_SYNC_FULL_ROTATION))
      {
        RCLCPP_ERROR(rclcpp::get_logger("KassowKordAdapter"), "Sync wait timed out, exiting.");
        return false;
      }

      rcv_iface_->fetchData();
    }

    // Retrieve and log the command status
    auto status = rcv_iface_->getCommandStatus(token);
    if (status != -1)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("KassowKordAdapter"), "%s command status: %d", name.c_str(),
        static_cast<int>(status));
    }
    else
    {
      RCLCPP_WARN(
        rclcpp::get_logger("KassowKordAdapter"), "%s command status remains unknown.",
        name.c_str());
      return false;
    }
  }

  return true;
}

bool KordAdapter::waitSync()
{
  if (!connected_)
  {
    return false;
  }

  return kord_->waitSync(std::chrono::milliseconds(waitSync_timeout_ms_));
}

// reads joint states: positions (rad), velocities and efforts
bool KordAdapter::readJointStates(
  std::array<double, 7> & positions, std::array<double, 7> & velocities,
  std::array<double, 7> & efforts)
{
  if (!connected_)
  {
    return false;
  }

  rcv_iface_->fetchData();
  positions = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_ACTUAL_Q);
  velocities = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_ACTUAL_QD);
  efforts = rcv_iface_->getJoint(kr2::kord::ReceiverInterface::EJointValue::S_SENSED_TRQ);

  return true;
}

bool KordAdapter::writeJointPositions(const std::array<double, 7> & position_cmds)
{
  if (!connected_)
  {
    return false;
  }

  try
  {
    if (!ctl_iface_->moveJ(
          position_cmds, kr2::kord::TrackingType::TT_NONE, TT_VALUE, kr2::kord::BlendType::BT_TIME,
          BT_VALUE, kr2::kord::OverlayType::OT_VIAPOINT))
    {
      return false;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KassowKordAdapter"), "Exception in writeJointPositions: %s", e.what());
    return false;
  }

  return true;
}

// TODO(yara): put in on cleanup
void KordAdapter::disconnect() { connected_ = false; }

// -----------------------------
// KassowKordHardwareInterface implementation
// -----------------------------
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
  // Get required parameter IP address
  if (hw_params.find("ip_address") == hw_params.end())
  {
    RCLCPP_FATAL(get_logger(), "Parameter 'ip_address' is required but not provided");
    return hardware_interface::CallbackReturn::ERROR;
  }
  else
  {
    ip_address = hw_params.at("ip_address");
  }

  // Get optional parameters with defaults
  int port = 7582;
  if (hw_params.find("port") != hw_params.end())
  {
    port = std::stoi(hw_params.at("port"));
  }

  int session_id = 1;
  if (hw_params.find("session_id") != hw_params.end())
  {
    session_id = std::stoi(hw_params.at("session_id"));
  }

  int waitSync_timeout_ms = 500;
  if (hw_params.find("waitSync_timeout_ms") != hw_params.end())
  {
    waitSync_timeout_ms = std::stoi(hw_params.at("waitSync_timeout_ms"));
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
    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' command interface invalid. Expected exactly one position interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate we have three state interface
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' state interface invalid. Expected exactly three state interfaces.",
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
      {
        has_position = true;
      }
      else if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity = true;
      }
      else if (state_interface.name == hardware_interface::HW_IF_EFFORT)
      {
        has_effort = true;
      }
    }

    if (!has_position || !has_velocity || !has_effort)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' missing required state interfaces. Expected position, velocity, and effort.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_position_itfs_[joint_index] = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    joint_velocity_itfs_[joint_index] = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    joint_effort_itfs_[joint_index] = joint.name + "/" + hardware_interface::HW_IF_EFFORT;
    joint_index++;
  }

  // init adapter with joint count
  kord_adapter_ = std::make_shared<KordAdapter>(ip_address, port, session_id, waitSync_timeout_ms);

  RCLCPP_INFO(
    get_logger(), "KassowKordHardwareInterface on_init completed for %zu joints", KORD_JOINT_COUNT);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KassowKordHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Establish connection to the robot using the adapter.
  try
  {
    RCLCPP_INFO(get_logger(), "Connecting to Kassow Kord robot at %s...", ip_address.c_str());
    if (!kord_adapter_->connect())
    {
      RCLCPP_FATAL(get_logger(), "Failed to connect to Kassow Kord robot via KordAdapter.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(get_logger(), "Exception while connecting to Kassow Kord robot: %s", e.what());
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
    if (!kord_adapter_->clean_alarms())
    {
      RCLCPP_FATAL(get_logger(), "Failed to reset alarms to Kassow Kord robot via KordAdapter.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Exception while resetting alarms: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial joint positions and set them as the initial command values
  try
  {
    std::array<double, KORD_JOINT_COUNT> initial_positions{};
    std::array<double, KORD_JOINT_COUNT> initial_velocities{};
    std::array<double, KORD_JOINT_COUNT> initial_torques{};
    kord_adapter_->readJointStates(initial_positions, initial_velocities, initial_torques);
    for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
    {
      RCLCPP_INFO(
        get_logger(), "Initial position for interface %s: %f", joint_position_itfs_[i].c_str(),
        initial_positions[i]);
      set_command(joint_position_itfs_[i], initial_positions[i]);
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to read initial state: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

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
  if (!kord_adapter_->waitSync())
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "KordAdapter waitSync timeout");
    return hardware_interface::return_type::ERROR;
  }

  std::array<double, KORD_JOINT_COUNT> position_states{};
  std::array<double, KORD_JOINT_COUNT> velocity_states{};
  std::array<double, KORD_JOINT_COUNT> torque_states{};
  if (!kord_adapter_->readJointStates(position_states, velocity_states, torque_states))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "KordAdapter failed to read states (using last known).");
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
  {
    set_state(joint_position_itfs_[i], position_states[i]);
    set_state(joint_velocity_itfs_[i], velocity_states[i]);
    set_state(joint_effort_itfs_[i], torque_states[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KassowKordHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::array<double, KORD_JOINT_COUNT> position_cmds{};
  for (size_t i = 0; i < KORD_JOINT_COUNT; ++i)
  {
    position_cmds[i] = get_command(joint_position_itfs_[i]);
  }
  if (!kord_adapter_->writeJointPositions(position_cmds))
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "KordAdapter failed to write joint positions");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace kassow_kord_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kassow_kord_hardware_interface::KassowKordHardwareInterface, hardware_interface::SystemInterface)
