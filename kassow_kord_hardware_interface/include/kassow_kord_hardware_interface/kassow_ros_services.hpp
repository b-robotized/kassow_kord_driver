// Copyright (c) 2026 b»robotized

#ifndef KASSOW_KORD_HARDWARE_INTERFACE__KASSOW_ROS_SERVICES_HPP_
#define KASSOW_KORD_HARDWARE_INTERFACE__KASSOW_ROS_SERVICES_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <control_msgs/srv/get_payload.hpp>
#include <control_msgs/srv/set_payload.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "kassow_kord_hardware_interface/kord_services.hpp"

namespace kassow_kord_hardware_interface
{

/// Robot status flags written in read() (RT) and published by the services thread.
struct RobotSafetyState
{
  std::atomic<uint32_t> motion_flags{0};
  std::atomic<uint32_t> safety_flags{0};
  std::atomic<uint32_t> button_flags{0};
  std::atomic<uint32_t> hw_flags{0};
  std::atomic<uint32_t> alarm_state{0};
};

/// ROS services of the hardware interface, executed in their own NRT thread.
///
/// Lifecycle of the thread:
/// - start() in on_configure: creates the thread in the paused state.
/// - resume() in on_activate, pause() in on_deactivate and on_error: do not block.
///   pause() aborts pending requests. The thread answers them, then waits on a condition variable.
/// - stop() in on_shutdown: ends and joins the thread.
///
/// The services answer with deferred responses, so no executor thread waits for the RT loop.
class KassowRosServices
{
public:
  KassowRosServices(const std::string & node_name, size_t update_rate);
  ~KassowRosServices();

  KassowRosServices(const KassowRosServices &) = delete;
  KassowRosServices & operator=(const KassowRosServices &) = delete;

  void start();
  void pause();
  void resume();
  void stop();

  /// RT access to the services.
  KordSetLoadService & set_load() { return set_load_; }
  KordGetPayloadService & get_payload() { return get_payload_; }
  RobotSafetyState & robot_state() { return robot_state_; }

  /// Abort all pending requests. Does not block.
  void abort_pending();

private:
  using SetPayload = control_msgs::srv::SetPayload;
  using GetPayload = control_msgs::srv::GetPayload;

  void thread_main();
  void configure_thread();
  void send_finished_responses();
  std::optional<kr2::kord::ELoadID> to_kord_load_id(uint8_t ros_load_type) const;

  void on_set_payload(
    rclcpp::Service<SetPayload>::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
    std::shared_ptr<SetPayload::Request> request);
  void on_get_payload(
    rclcpp::Service<GetPayload>::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
    std::shared_ptr<GetPayload::Request> request);
  void publish_state_if_changed();

  size_t update_rate_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  rclcpp::Service<SetPayload>::SharedPtr set_payload_srv_;
  rclcpp::Service<GetPayload>::SharedPtr get_payload_srv_;
  std::optional<rmw_request_id_t> set_payload_header_;
  std::optional<rmw_request_id_t> get_payload_header_;

  KordSetLoadService set_load_;
  KordGetPayloadService get_payload_;
  RobotSafetyState robot_state_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  std::array<uint32_t, 5> last_published_state_;

  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool running_ = false;
  bool active_ = false;
};

}  // namespace kassow_kord_hardware_interface

#endif  // KASSOW_KORD_HARDWARE_INTERFACE__KASSOW_ROS_SERVICES_HPP_
