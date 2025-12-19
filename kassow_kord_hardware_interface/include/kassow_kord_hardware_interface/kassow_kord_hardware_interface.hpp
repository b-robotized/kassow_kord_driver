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
#include <kord/api/kord_receive_interface.h>
#include <kord/utils/utils.h>
#include <kord/version.h>

#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace kassow_kord_hardware_interface
{
const size_t KORD_JOINT_COUNT = 7;

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

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool clean_alarms();

  std::shared_ptr<kr2::kord::KordCore> kord_;
  std::unique_ptr<kr2::kord::ControlInterface> ctl_iface_;
  std::unique_ptr<kr2::kord::ReceiverInterface> rcv_iface_;

  std::array<std::string, KORD_JOINT_COUNT> joint_position_itfs_;
  std::array<std::string, KORD_JOINT_COUNT> joint_velocity_itfs_;
  std::array<std::string, KORD_JOINT_COUNT> joint_acceleration_itfs_;
  std::array<std::string, KORD_JOINT_COUNT> joint_effort_itfs_;

  std::array<double, KORD_JOINT_COUNT> position_states{};
  std::array<double, KORD_JOINT_COUNT> velocity_states{};
  std::array<double, KORD_JOINT_COUNT> acceleration_states{};
  std::array<double, KORD_JOINT_COUNT> torque_states{};

  std::array<double, KORD_JOINT_COUNT> position_cmds{};
  std::array<double, KORD_JOINT_COUNT> velocity_cmds{};
  std::array<double, KORD_JOINT_COUNT> acceleration_cmds{};

  std::string ip_address;
  int session_id;
  int port;
  int waitSync_timeout_ms;
};

}  // namespace kassow_kord_hardware_interface

#endif  // KASSOW_KORD_HARDWARE_INTERFACE__KASSOW_KORD_HARDWARE_INTERFACE_HPP_
