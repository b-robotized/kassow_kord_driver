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

#ifndef kassow_kord_driver__KASSOW_KORD_DRIVER_HPP_
#define kassow_kord_driver__KASSOW_KORD_DRIVER_HPP_

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

#include "kassow_kord_driver.hpp"
#include "kassow_kord_adapter.hpp"

namespace kassow_kord_driver
{
class KassowKordDriver : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KassowKordDriver)

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
  const size_t JOINT_COUNT = 7;
  std::shared_ptr<kassow_kord_adapter::KordAdapter> kord_adapter_;
  std::array<std::string, JOINT_COUNT> joint_names_;
  std::map<std::string, uint8_t> joint_finger_ids_;
};

}  // namespace kassow_kord_driver

#endif  // kassow_kord_driver__KASSOW_KORD_DRIVER_HPP_