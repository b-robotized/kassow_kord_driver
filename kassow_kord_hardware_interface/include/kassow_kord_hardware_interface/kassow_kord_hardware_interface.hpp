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

#ifndef KASSOW_KORD_HARDWARE_INTERFACE__KASSOW_KORD_HARDWARE_INTERFACE_HPP_
#define KASSOW_KORD_HARDWARE_INTERFACE__KASSOW_KORD_HARDWARE_INTERFACE_HPP_

#include <kord/api/kord.h>
#include <kord/api/kord_control_interface.h>
#include <kord/api/kord_io_request.h>
#include <kord/api/kord_receive_interface.h>
#include <kord/utils/utils.h>
#include <kord/version.h>

#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kassow_kord_hardware_interface/kassow_ros_services.hpp"
#include "kassow_kord_hardware_interface/kord_services.hpp"

namespace kassow_kord_hardware_interface
{
const size_t KORD_JOINT_COUNT = 7;
const size_t KORD_INPUT_COUNT = 16;
const size_t KORD_OUTPUT_COUNT = 16;
// TCP pose: position x, y, z and orientation quaternion x, y, z, w
const size_t KORD_TCP_POSE_SIZE = 7;

class KassowKordHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KassowKordHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Copy of the kord-api 3.0.2 protocol enums. Keep the names and values the same.
  enum EMotionFlags : unsigned int
  {
    MOTION_FLAG_STANDBY = 1,
    MOTION_FLAG_TRACKING = 2,
    MOTION_FLAG_TERMINATED = 4,
    MOTION_FLAG_HALT = 8,
    MOTION_FLAG_SYNC = 16,
    MOTION_FLAG_SUSPENDED = 32,
    MOTION_FLAG_OFFLINE = 64,
    MOTION_FLAG_INIT = 128,
    MOTION_FLAG_REINIT = 256,
    MOTION_FLAG_BACKDRIVE = 512,
    MOTION_FLAG_PAUSED = 1024,
    MOTION_FLAG_MAINTENANCE = 2048,
    MOTION_FLAG_VELOCITYCTL = 4096,
    MOTION_FLAG_ARTOACTIVE = 8192
  };

  enum ESafetyFlags : unsigned int
  {
    SAFETY_FLAG_UPDATE = 1,
    SAFETY_FLAG_ESTOP = 2,
    SAFETY_FLAG_PSTOP = 4,
    SAFETY_FLAG_SSTOP = 8,
    SAFETY_FLAG_USER_CONF_REQ = 16
  };

  enum EButtonFlags : unsigned int
  {
    BUTTONS_FLAG_ESTOP = 0x01,
    BUTTONS_FLAG_PSTOP = 0x02,
    BUTTONS_FLAG_TOGGLE = 0x04,
    BUTTONS_FLAG_BACKDRIVE = 0x08,
    BUTTONS_FLAG_TEACH = 0x10
  };

  enum EHWFlags : unsigned int
  {
    HW_STAT_LOW_VOLTAGE_AFTER_RELAY_DETECTED = 0x01,
    HW_STAT_INIT_BLOCKED = 0x02,
    HW_STAT_INIT_RUID_MISMATCH = 0x04,
    HW_STAT_HARD_FAULT = 0x08,
    HW_STATUS_DISCOVERY_FAILED = 0x0100,
    HW_STATUS_ERROR_BITS_SET = 0x0200,
    HW_STATUS_DEVICE_DISABLED = 0x0400,
    HW_STATUS_LOW_VOLTAGE = 0x0800,
    HW_STATUS_IOB_INIT_TIMEOUT = 0x1000,
    HW_STATUS_SYNC_INIT_TIMEOUT = 0x2000,
    HW_STATUS_IOB_ESTOP_STALL = 0x4000,
    HW_STATUS_IOB_PSTOP_STALL = 0x8000
  };

  enum ESystemAlarmCategory : unsigned int
  {
    CAT_SAFETY_EVENT = 0x01,
    CAT_SOFT_STOP_EVENT = 0x02,
    CAT_HW_STAT = 0x03,
    CAT_CBUN_EVENT = 0x04
  };

  enum ESystemAlarmContext : unsigned int
  {
    CNTXT_ESTOP = 0x01,
    CNTXT_PSTOP = 0x02,
    CNTXT_SSTOP = 0x04,
    CNTXT_SYSERR = 0x08
  };

  // Motion flags that are normal while the hardware is active.
  // Any other motion flag deactivates it.
  static constexpr unsigned int ALLOWED_MOTION_FLAGS = MOTION_FLAG_STANDBY | MOTION_FLAG_TRACKING;
  // Safety and button flags that deactivate the hardware. Any HW flag deactivates it.
  static constexpr unsigned int STOP_SAFETY_FLAGS =
    SAFETY_FLAG_ESTOP | SAFETY_FLAG_PSTOP | SAFETY_FLAG_SSTOP | SAFETY_FLAG_USER_CONF_REQ;
  static constexpr unsigned int STOP_BUTTON_FLAGS =
    BUTTONS_FLAG_ESTOP | BUTTONS_FLAG_PSTOP | BUTTONS_FLAG_BACKDRIVE | BUTTONS_FLAG_TEACH;

private:
  // UNCLEARABLE: flags that clean_alarms() cannot clear. ALL: also the flags it can clear.
  enum class FlagScope
  {
    UNCLEARABLE,
    ALL
  };
  std::optional<std::string> find_tripped_flags(FlagScope scope);
  void log_flags(const char * event, bool only_on_change);
  bool clean_alarms();
  void read_digital_io();
  void update_digital_output_request();
  void process_ros_services();
  void write_digital_outputs();
  void log_effective_limits_on_change();
  // bool is_robot_stationary() const;
  double get_max_joint_velocity() const;
  std::string motion_flags_to_string(unsigned int flags) const;

  std::shared_ptr<kr2::kord::KordCore> kord_;
  std::unique_ptr<kr2::kord::ControlInterface> ctl_iface_;
  std::unique_ptr<kr2::kord::ReceiverInterface> rcv_iface_;

  std::array<std::string, KORD_JOINT_COUNT> joint_position_itfs_;
  std::array<std::string, KORD_JOINT_COUNT> joint_velocity_itfs_;
  std::array<std::string, KORD_JOINT_COUNT> joint_acceleration_itfs_;
  std::array<std::string, KORD_JOINT_COUNT> joint_effort_itfs_;
  // Optional state interfaces. Empty if the joint does not have them.
  std::array<std::string, KORD_JOINT_COUNT> joint_effort_reference_itfs_;
  std::array<std::string, KORD_JOINT_COUNT> joint_torque_deviation_itfs_;

  std::array<double, KORD_JOINT_COUNT> position_states{};
  std::array<double, KORD_JOINT_COUNT> velocity_states{};
  std::array<double, KORD_JOINT_COUNT> acceleration_states{};
  std::array<double, KORD_JOINT_COUNT> torque_states{};

  std::array<double, KORD_JOINT_COUNT> position_cmds{};
  std::array<double, KORD_JOINT_COUNT> velocity_cmds{};
  std::array<double, KORD_JOINT_COUNT> acceleration_cmds{};

  // Last effective limits from KORD. The torque lower limits are NaN until the first read.
  std::array<double, KORD_JOINT_COUNT> limit_torque_lower_;
  std::array<double, KORD_JOINT_COUNT> limit_torque_upper_;
  std::array<double, KORD_JOINT_COUNT> limit_joint_speed_;

  std::array<std::string, KORD_INPUT_COUNT> digital_inputs_itfs_;
  std::array<std::string, KORD_OUTPUT_COUNT> digital_outputs_itfs_;

  // TCP pose state interfaces. Empty if the description has no TCP pose sensor.
  std::array<std::string, KORD_TCP_POSE_SIZE> tcp_pose_itfs_;
  bool has_tcp_pose_ = false;

  // Digital outputs: bitfield of the outputs confirmed by KORD, and the request in progress.
  uint64_t digital_outputs_sent_ = 0;
  KordDigitalOutputService digital_output_service_;

  std::string ip_address;
  int session_id;
  int port;
  int waitSync_timeout_ms;

  // ROS services in their own NRT thread. Created in on_configure, stopped in on_shutdown.
  std::unique_ptr<KassowRosServices> ros_services_;
  size_t update_rate_ = 500;

  // Testing, to be removed:
  // Motion, safety, button, HW and alarm flags at the last log. The start value forces a log.
  std::array<unsigned int, 5> flags_logged_{~0u, ~0u, ~0u, ~0u, ~0u};
  bool skip_jcontrol_ = false;

  // Logs the status flags and the alarm clearing at INFO level.
  bool logging_verbose_ = true;

  // Set at the first tripped flag and cleared in on_activate(). The async worker can overwrite a
  // DEACTIVATE result before the controller manager reads it, so write() returns it again.
  bool deactivate_latched_ = false;
};

}  // namespace kassow_kord_hardware_interface

#endif  // KASSOW_KORD_HARDWARE_INTERFACE__KASSOW_KORD_HARDWARE_INTERFACE_HPP_
