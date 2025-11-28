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

#include "kassow_kord_driver/kassow_kord_adapter.hpp"
#include <kord/api/kord_control_interface.h>
#include <kord/api/kord_receive_interface.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <cmath>

using namespace kr2;

namespace kassow_kord_driver
{
  constexpr size_t JOINT_COUNT = 7;

class KordAdapter
{
public:
  KordAdapter() = default;
  ~KordAdapter() = default;

  bool init(const std::string & ip_address, int port, int session_id, int waitSync_timeout_ms)
  {
    // Create an instance of KordCore for handling RX/TX KORD frames.
    std::shared_ptr<kord::KordCore> kord(new kord::KordCore(ip_address, port, session_id, kord::UDP_CLIENT));

    // Initialize Control and Receiver Interfaces.
    kord::ControlInterface ctl_iface(kord);
    kord::ReceiverInterface rcv_iface(kord);

    waitSync_timeout_ms_ = waitSync_timeout_ms;
    return true;
  }

  bool connect()
  {
    if (!kord->connect()) {
      std::cout << "Connecting to KR failed\n";
      connected_ = false;
      return connected_;
    }
    connected_ = true;
    return connected_;
  }

  // TODO
  bool reset_alarms()
  {
    if (!connected_)
      return false;

    // Reset alarms logic here
    return true;
  }

  bool waitSync()
  {
    if (!connected_)
      return false;

    return kord->waitSync(std::chrono::milliseconds(waitSync_timeout_ms_));
  }

  // reads joint states: positions (rad), velocities and efforts (optional)
  bool readJointStates(std::array<double, 7>& positions,
                       std::array<double, 7>& velocities,
                       std::array<double, 7>& efforts)
  {
    if (!connected_)
      return false;

    rcv_iface.fetchData();
    positions = rcv_iface.getJoint(kord::ReceiverInterface::EJointValue::S_ACTUAL_Q);
    velocities = rcv_iface.getJoint(kord::ReceiverInterface::EJointValue::S_ACTUAL_QD);
    efforts = rcv_iface.getJoint(kord::ReceiverInterface::EJointValue::S_SENSED_TRQ);
    
    return true;
  }

  // TODO
  bool writeJointPositions(const std::array<double, 7>& position_cmds)
  {
    if (!connected_)
      return false;

    return true;
  }

  // TODO
  void disconnect()
  {
    connected_ = false;
  }

private:
  bool connected_{false};
  std::vector<double> last_written_;
  int waitSync_timeout_ms_{500};
};

}  // namespace kassow_kord_driver