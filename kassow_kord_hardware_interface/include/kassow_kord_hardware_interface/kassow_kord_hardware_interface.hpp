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

#ifndef kassow_kord_hardware_interface__KASSOW_KORD_HARDWARE_INTERFACE_HPP_
#define kassow_kord_hardware_interface__KASSOW_KORD_HARDWARE_INTERFACE_HPP_

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

#include <kord/api/kord.h> 
#include <kord/version.h>
#include <kord/api/kord_control_interface.h>
#include <kord/api/kord_receive_interface.h>

namespace kassow_kord_hardware_interface
{
const size_t KORD_JOINT_COUNT = 7;

class KordAdapter
{
public:
  KordAdapter(
  const std::string & ip_address,
  int port,
  int session_id,
  int waitSync_timeout_ms);
  ~KordAdapter();

  // Initialize underlying kord connection resources. Returns true on success.
  bool init(const std::string & ip_address, int port, int session_id, int waitSync_timeout_ms);

  // Connect/disconnect
  bool connect();
  void disconnect();
  bool isConnected() const { return connected_; }

  // Wait for sync (blocking with timeout configured in init)
  bool waitSync();

  // Reset alarms
  bool reset_alarms();

  // Read joint states. All arrays must have size KORD_JOINT_COUNT.
  bool readJointStates(std::array<double, KORD_JOINT_COUNT>& positions,
                       std::array<double, KORD_JOINT_COUNT>& velocities,
                       std::array<double, KORD_JOINT_COUNT>& efforts);

  // Write joint position commands (size KORD_JOINT_COUNT).
  bool writeJointPositions(const std::array<double, KORD_JOINT_COUNT>& position_cmds);

  // Lightweight configuration helper
  void configure(int waitSync_timeout_ms);

private:
  std::shared_ptr<kr2::kord::KordCore> kord_;
  std::unique_ptr<kr2::kord::ControlInterface> ctl_iface_;
  std::unique_ptr<kr2::kord::ReceiverInterface> rcv_iface_;
  bool connected_{false};
  int waitSync_timeout_ms_{500};
};

class KassowKordHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KassowKordHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

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
  std::shared_ptr<KordAdapter> kord_adapter_;
  std::array<std::string, KORD_JOINT_COUNT> joint_names_;
  std::string ip_address;
};

}  // namespace kassow_kord_hardware_interface

#endif  // kassow_kord_hardware_interface__KASSOW_KORD_HARDWARE_INTERFACE_HPP_