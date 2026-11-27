// Copyright (c) 2026 b»robotized

#include "kassow_kord_hardware_interface/kassow_ros_services.hpp"

#include <pthread.h>
#include <sched.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

#include "realtime_tools/realtime_helpers.hpp"

namespace kassow_kord_hardware_interface
{

KassowRosServices::KassowRosServices(const std::string & node_name, size_t update_rate)
: update_rate_(update_rate), node_(std::make_shared<rclcpp::Node>(node_name))
{
  last_published_state_.fill(0xFFFFFFFF);

  set_payload_srv_ = node_->create_service<SetPayload>(
    "~/set_payload",
    [this](
      rclcpp::Service<SetPayload>::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
      std::shared_ptr<SetPayload::Request> request)
    { on_set_payload(std::move(service), std::move(header), std::move(request)); });

  get_payload_srv_ = node_->create_service<GetPayload>(
    "~/get_payload",
    [this](
      rclcpp::Service<GetPayload>::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
      std::shared_ptr<GetPayload::Request> request)
    { on_get_payload(std::move(service), std::move(header), std::move(request)); });

  state_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "~/robot_state", rclcpp::QoS(1).transient_local());
  state_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(20), [this]() { publish_state_if_changed(); });

  executor_.add_node(node_);
}

KassowRosServices::~KassowRosServices() { stop(); }

void KassowRosServices::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_)
  {
    return;
  }
  running_ = true;
  active_ = false;
  thread_ = std::thread([this]() { thread_main(); });
}

void KassowRosServices::pause()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
  }
  // The thread answers the aborted requests before it waits.
  abort_pending();
}

void KassowRosServices::resume()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = true;
  }
  cv_.notify_all();
}

void KassowRosServices::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_)
    {
      return;
    }
    running_ = false;
  }
  abort_pending();
  cv_.notify_all();
  executor_.cancel();
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void KassowRosServices::abort_pending()
{
  set_load_.abort();
  get_payload_.abort();
}

void KassowRosServices::configure_thread()
{
  // The thread inherits SCHED_FIFO and a high priority from the hardware interface thread.
  // Move it to NRT cores and to SCHED_OTHER.
  const std::vector<int> target_cores = {0, 1};
  const auto affinity_result = realtime_tools::set_current_thread_affinity(target_cores);
  if (!affinity_result.first)
  {
    RCLCPP_WARN(
      node_->get_logger(), "Failed to set thread affinity: %s", affinity_result.second.c_str());
  }
  realtime_tools::set_current_thread_name("kassow_ros_srvs");

  struct sched_param schedp;
  std::memset(&schedp, 0, sizeof(schedp));
  schedp.sched_priority = 0;
  if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &schedp) != 0)
  {
    RCLCPP_WARN(
      node_->get_logger(), "Failed to set thread to SCHED_OTHER: %s", std::strerror(errno));
  }
}

void KassowRosServices::thread_main()
{
  configure_thread();
  while (true)
  {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() { return !running_ || active_; });
      if (!running_)
      {
        break;
      }
    }
    executor_.spin_once(std::chrono::milliseconds(10));
    send_finished_responses();
  }
  // Answer the requests that were aborted on stop.
  send_finished_responses();
}

std::optional<kr2::kord::ELoadID> KassowRosServices::to_kord_load_id(uint8_t ros_load_type) const
{
  switch (ros_load_type)
  {
    case SetPayload::Request::LOAD_TYPE_END_EFFECTOR:
      return kr2::kord::ELoadID::LOAD1;
    case SetPayload::Request::LOAD_TYPE_PAYLOAD:
      return kr2::kord::ELoadID::LOAD2;
    default:
      RCLCPP_ERROR(
        node_->get_logger(),
        "No KORD load ID for ROS load_type %d. Supported: END_EFFECTOR=%d, PAYLOAD=%d.",
        ros_load_type, SetPayload::Request::LOAD_TYPE_END_EFFECTOR,
        SetPayload::Request::LOAD_TYPE_PAYLOAD);
      return std::nullopt;
  }
}

void KassowRosServices::on_set_payload(
  rclcpp::Service<SetPayload>::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
  std::shared_ptr<SetPayload::Request> request)
{
  SetPayload::Response response;
  response.success = false;

  if (set_payload_header_)
  {
    RCLCPP_WARN(node_->get_logger(), "A SetPayload request is already in progress. Rejecting.");
    service->send_response(*header, response);
    return;
  }

  const auto load_id = to_kord_load_id(request->load_type);
  if (!load_id)
  {
    service->send_response(*header, response);
    return;
  }

  const std::array<double, 3> cog = {
    request->center_of_gravity.x, request->center_of_gravity.y, request->center_of_gravity.z};
  const std::array<double, 6> inertia = {request->ixx, request->iyy, request->izz,
                                         request->ixy, request->ixz, request->iyz};
  const double timeout_sec = rclcpp::Duration(request->timeout).seconds();
  const auto timeout_ticks = static_cast<size_t>(timeout_sec * static_cast<double>(update_rate_));

  if (!set_load_.request(
        load_id.value(), static_cast<double>(request->mass), cog, inertia, timeout_ticks))
  {
    RCLCPP_WARN(node_->get_logger(), "SetLoad service is busy. Rejecting.");
    service->send_response(*header, response);
    return;
  }
  set_payload_header_ = *header;
}

void KassowRosServices::on_get_payload(
  rclcpp::Service<GetPayload>::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
  std::shared_ptr<GetPayload::Request> request)
{
  GetPayload::Response response;
  response.success = false;

  const auto load_id = to_kord_load_id(request->load_type);
  if (get_payload_header_ || !load_id || !get_payload_.request(load_id.value()))
  {
    service->send_response(*header, response);
    return;
  }
  get_payload_header_ = *header;
}

void KassowRosServices::send_finished_responses()
{
  if (set_payload_header_ && set_load_.is_finished())
  {
    SetPayload::Response response;
    const auto state = set_load_.get_state();
    response.success = (state == KordServiceState::SUCCESS);
    switch (state)
    {
      case KordServiceState::SUCCESS:
        RCLCPP_INFO(
          node_->get_logger(), "SetPayload successful. KORD status: %d", set_load_.get_status());
        break;
      case KordServiceState::FAILURE:
        RCLCPP_ERROR(
          node_->get_logger(), "SetPayload failed on the Kassow controller. KORD status: %d",
          set_load_.get_status());
        break;
      case KordServiceState::TIMEOUT:
        RCLCPP_ERROR(node_->get_logger(), "SetPayload timed out.");
        break;
      default:
        RCLCPP_WARN(node_->get_logger(), "SetPayload aborted.");
        break;
    }
    set_payload_srv_->send_response(*set_payload_header_, response);
    set_payload_header_.reset();
    set_load_.reset();
  }

  if (get_payload_header_ && get_payload_.is_finished())
  {
    GetPayload::Response response;
    response.success = (get_payload_.get_state() == KordServiceState::SUCCESS);
    if (response.success)
    {
      const auto & payload = get_payload_.get_payload();
      response.mass = static_cast<float>(payload.mass);
      response.center_of_gravity.x = payload.cog[0];
      response.center_of_gravity.y = payload.cog[1];
      response.center_of_gravity.z = payload.cog[2];
      response.ixx = payload.inertia[0];
      response.iyy = payload.inertia[1];
      response.izz = payload.inertia[2];
      response.ixy = payload.inertia[3];
      response.ixz = payload.inertia[4];
      response.iyz = payload.inertia[5];
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "GetPayload failed.");
    }
    get_payload_srv_->send_response(*get_payload_header_, response);
    get_payload_header_.reset();
    get_payload_.reset();
  }
}

void KassowRosServices::publish_state_if_changed()
{
  const std::array<uint32_t, 5> state = {
    robot_state_.motion_flags.load(std::memory_order_relaxed),
    robot_state_.safety_flags.load(std::memory_order_relaxed),
    robot_state_.button_flags.load(std::memory_order_relaxed),
    robot_state_.hw_flags.load(std::memory_order_relaxed),
    robot_state_.alarm_state.load(std::memory_order_relaxed)};

  if (state == last_published_state_)
  {
    return;
  }

  char json_buffer[256];
  std::snprintf(
    json_buffer, sizeof(json_buffer),
    "{\"motion_flags\": %u, \"safety_flags\": %u, \"button_flags\": %u, \"hw_flags\": %u, "
    "\"alarm_state\": %u}",
    state[0], state[1], state[2], state[3], state[4]);

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = json_buffer;
  state_pub_->publish(std::move(msg));
  last_published_state_ = state;
}

}  // namespace kassow_kord_hardware_interface
