// Copyright (c) 2026 b»robotized

#ifndef KASSOW_KORD_HARDWARE_INTERFACE__KORD_SERVICES_HPP_
#define KASSOW_KORD_HARDWARE_INTERFACE__KORD_SERVICES_HPP_

#include <kord/api/kord.h>
#include <kord/api/kord_control_interface.h>
#include <kord/api/kord_io_request.h>
#include <kord/api/kord_receive_interface.h>
#include <kord/protocol/DataDescriptions/Requests/ControlCommandStatus.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <variant>
#include <vector>

namespace kassow_kord_hardware_interface
{

enum class KordServiceState
{
  IDLE,
  REQUESTED,
  DISPATCHED,
  SUCCESS,
  FAILURE,  // failed on the Kassow controller side
  TIMEOUT,  // no answer in time
  ABORTED   // aborted on the ROS side, for example on deactivation
};

/// State of one service, shared between the services thread (NRT) and read()/write() (RT).
/// NRT: request() when IDLE, wait for a final state, then reset().
/// RT: process the request and set a final state.
/// All state changes use compare-and-exchange, so a final state is never overwritten.
class KordServiceBase
{
public:
  virtual ~KordServiceBase() = default;

  KordServiceState get_state() const { return state_.load(std::memory_order_acquire); }

  bool is_pending() const
  {
    const auto state = get_state();
    return state == KordServiceState::REQUESTED || state == KordServiceState::DISPATCHED;
  }

  bool is_finished() const
  {
    const auto state = get_state();
    return state != KordServiceState::IDLE && !is_pending();
  }

  /// NRT and RT: abort a pending request.
  void abort()
  {
    if (!transition(KordServiceState::REQUESTED, KordServiceState::ABORTED))
    {
      transition(KordServiceState::DISPATCHED, KordServiceState::ABORTED);
    }
  }

  /// NRT: release a finished service for the next request.
  void reset()
  {
    if (is_finished())
    {
      state_.store(KordServiceState::IDLE, std::memory_order_release);
    }
  }

protected:
  bool transition(KordServiceState from, KordServiceState to)
  {
    return state_.compare_exchange_strong(from, to, std::memory_order_acq_rel);
  }

  /// RT: count one cycle and set TIMEOUT when the limit is reached. 0 means no timeout.
  void tick_timeout()
  {
    if (timeout_ticks_ > 0 && ++ticks_ >= timeout_ticks_)
    {
      transition(KordServiceState::DISPATCHED, KordServiceState::TIMEOUT);
    }
  }

  /// RT: map a KORD command status to a service state. Returns false if still in progress.
  bool finish_with(kr2::kord::protocol::EControlCommandStatus status)
  {
    using kr2::kord::protocol::EControlCommandStatus;
    switch (status)
    {
      case EControlCommandStatus::eSuccess:
      // For instantaneous commands like SetLoad, eNone is the final acknowledgment.
      case EControlCommandStatus::eNone:
        transition(KordServiceState::DISPATCHED, KordServiceState::SUCCESS);
        return true;
      case EControlCommandStatus::eInProgress:
      case EControlCommandStatus::eIdle:
        return false;
      default:
        transition(KordServiceState::DISPATCHED, KordServiceState::FAILURE);
        return true;
    }
  }

  alignas(64) std::atomic<KordServiceState> state_{KordServiceState::IDLE};
  size_t timeout_ticks_ = 0;
  size_t ticks_ = 0;
};

/// SetLoad: a KORD command. The robot must stand still and directJControl is paused until the
/// command is finished. Monitored with the command token from fetchData().
class KordSetLoadService : public KordServiceBase
{
public:
  /// NRT
  bool request(
    kr2::kord::ELoadID load_id, double mass, const std::array<double, 3> & cog,
    const std::array<double, 6> & inertia, size_t timeout_ticks)
  {
    if (get_state() != KordServiceState::IDLE)
    {
      return false;
    }
    load_id_ = load_id;
    mass_ = mass;
    cog_ = cog;
    inertia_ = inertia;
    timeout_ticks_ = timeout_ticks;
    ticks_ = 0;
    status_.store(-1, std::memory_order_relaxed);
    return transition(KordServiceState::IDLE, KordServiceState::REQUESTED);
  }

  /// RT
  void dispatch(kr2::kord::ControlInterface & ctl)
  {
    if (transition(KordServiceState::REQUESTED, KordServiceState::DISPATCHED))
    {
      ctl.setLoad(load_id_, mass_, cog_, inertia_, token_);
    }
  }

  /// RT
  void poll(kr2::kord::ReceiverInterface & rcv)
  {
    const int8_t raw_status = rcv.getCommandStatus(token_);
    // -1 means the status for the token is not received yet
    if (raw_status == -1)
    {
      tick_timeout();
      return;
    }
    status_.store(raw_status, std::memory_order_relaxed);
    if (!finish_with(static_cast<kr2::kord::protocol::EControlCommandStatus>(raw_status)))
    {
      tick_timeout();
    }
  }

  /// Last KORD command status, for logging.
  int8_t get_status() const { return status_.load(std::memory_order_relaxed); }

private:
  kr2::kord::ELoadID load_id_ = kr2::kord::ELoadID::LOAD1;
  double mass_ = 0.0;
  std::array<double, 3> cog_{};
  std::array<double, 6> inertia_{};
  int64_t token_ = -1;
  std::atomic<int8_t> status_{-1};
};

/// GetPayload: no request to KORD. read() copies the load values that fetchData() updated in
/// the same cycle.
class KordGetPayloadService : public KordServiceBase
{
public:
  struct Payload
  {
    double mass = 0.0;
    std::array<double, 3> cog{};
    std::array<double, 6> inertia{};
  };

  /// NRT
  bool request(kr2::kord::ELoadID load_id)
  {
    if (get_state() != KordServiceState::IDLE)
    {
      return false;
    }
    load_id_ = load_id;
    return transition(KordServiceState::IDLE, KordServiceState::REQUESTED);
  }

  /// RT: called after fetchData(). getLoad() allocates, but only on request.
  void serve(kr2::kord::ReceiverInterface & rcv)
  {
    if (get_state() != KordServiceState::REQUESTED)
    {
      return;
    }
    const auto mass = rcv.getLoad(load_id_, kr2::kord::MASS_VAL);
    const auto cog = rcv.getLoad(load_id_, kr2::kord::COG_VAL);
    const auto inertia = rcv.getLoad(load_id_, kr2::kord::INERTIA_VAL);
    if (mass.empty() || cog.size() < 3 || inertia.size() < 6)
    {
      transition(KordServiceState::REQUESTED, KordServiceState::FAILURE);
      return;
    }
    payload_.mass = to_double(mass[0]);
    for (size_t i = 0; i < 3; ++i)
    {
      payload_.cog[i] = to_double(cog[i]);
    }
    for (size_t i = 0; i < 6; ++i)
    {
      payload_.inertia[i] = to_double(inertia[i]);
    }
    transition(KordServiceState::REQUESTED, KordServiceState::SUCCESS);
  }

  /// NRT: valid after SUCCESS.
  const Payload & get_payload() const { return payload_; }

private:
  static double to_double(const std::variant<double, int> & value)
  {
    return std::holds_alternative<double>(value) ? std::get<double>(value)
                                                 : static_cast<double>(std::get<int>(value));
  }

  kr2::kord::ELoadID load_id_ = kr2::kord::ELoadID::LOAD1;
  Payload payload_;
};

/// Digital outputs: a KORD IO request. directJControl is not paused. Monitored with the request
/// ID of the latest request. Used only from read()/write() (RT).
class KordDigitalOutputService : public KordServiceBase
{
public:
  /// RT: send one request. KORD accepts either enabled or disabled ports per request.
  void dispatch(
    kr2::kord::ControlInterface & ctl, uint64_t enable_mask, uint64_t disable_mask,
    size_t timeout_ticks)
  {
    if (get_state() != KordServiceState::IDLE)
    {
      return;
    }
    kr2::kord::RequestIO request;  // new request ID
    if (enable_mask)
    {
      request.asSetIODigitalOut().withEnabledPorts(static_cast<int64_t>(enable_mask));
    }
    else
    {
      request.asSetIODigitalOut().withDisabledPorts(static_cast<int64_t>(disable_mask));
    }
    request_rid_ = request.request_rid_;
    enable_mask_ = enable_mask;
    disable_mask_ = disable_mask;
    timeout_ticks_ = timeout_ticks;
    ticks_ = 0;
    state_.store(KordServiceState::DISPATCHED, std::memory_order_release);
    ctl.transmitRequest(request);
  }

  /// RT
  void poll(kr2::kord::ReceiverInterface & rcv)
  {
    const auto latest = rcv.getLatestRequest();
    if (latest.request_rid_ != request_rid_ || !finish_with(latest.request_status_))
    {
      tick_timeout();
    }
  }

  /// RT: release the service after a final state. Returns the final state.
  KordServiceState release()
  {
    const auto state = get_state();
    reset();
    return state;
  }

  int64_t get_request_rid() const { return request_rid_; }
  uint64_t get_enable_mask() const { return enable_mask_; }
  uint64_t get_disable_mask() const { return disable_mask_; }

private:
  int64_t request_rid_ = 0;
  uint64_t enable_mask_ = 0;
  uint64_t disable_mask_ = 0;
};

}  // namespace kassow_kord_hardware_interface

#endif  // KASSOW_KORD_HARDWARE_INTERFACE__KORD_SERVICES_HPP_
