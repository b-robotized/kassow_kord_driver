#ifndef KASSOW_KORD_HARDWARE_INTERFACE__ASYNC_COMMANDS_HPP_
#define KASSOW_KORD_HARDWARE_INTERFACE__ASYNC_COMMANDS_HPP_

#include <atomic>
#include <array>
#include <kord/protocol/DataDescriptions/Requests/ControlCommandStatus.h>
#include <kord/api/kord.h>
#include <kord/api/kord_control_interface.h>
#include <kord/api/kord_receive_interface.h>


enum class KordServiceState { 
    IDLE, 
    REQUESTED, 
    DISPATCHED, 
    SUCCESS, 
    FAILURE, // failed on kassow controller-side
    TIMEOUT, // timed out
    ABORTED // aborted on ROS-side, for example on shutdown
};

class KordServiceInterface {
public:
    virtual ~KordServiceInterface() = default;
    
    inline KordServiceState get_state() const { return state_.load(std::memory_order_acquire); }

    // NRT
    inline void reset() { state_.store(KordServiceState::IDLE, std::memory_order_release); }
    // NRT
    inline void abort() { state_.store(KordServiceState::ABORTED, std::memory_order_release); }
    // RT
    virtual void dispatch(kr2::kord::ControlInterface& ctl) = 0;
    // RT
    virtual void poll(kr2::kord::ReceiverInterface& rcv) = 0;

    // Variables -----------------

    // minoor: padded so non-atomic tics are not on the same cache line
    alignas(64) std::atomic<KordServiceState> state_{KordServiceState::IDLE};
    size_t ticks_timeout_ = 0; // converted to ticks, as poll is called cyclically
    size_t ticks_ = 0;
};

class KordServiceSetLoad : public KordServiceInterface {
private:
    int64_t token_ = -1;
    alignas(64) std::atomic<int8_t> status_{-1};
    kr2::kord::ELoadID load_id_ = kr2::kord::ELoadID::LOAD1;
    double mass_ = 0.0;
    std::array<double, 3> cog_ = {0.0, 0.0, 0.0};
    std::array<double, 6> inertia_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    
public:
    int8_t get_status(){ return status_.load(std::memory_order_acquire); };

    // NRT
    void populate(kr2::kord::ELoadID load_type, 
                  double mass, 
                  const std::array<double, 3>& cog, 
                  const std::array<double, 6>& inertia, 
                  size_t timeout_ticks) {
        
        load_id_ = static_cast<kr2::kord::ELoadID>(load_type);
        mass_ = mass;
        cog_ = cog;
        inertia_ = inertia;
        ticks_timeout_ = timeout_ticks;
        ticks_ = 0;
    }

    // NRT
    void request() { 
        status_.store(-1, std::memory_order_release);
        state_.store(KordServiceState::REQUESTED, std::memory_order_release); 
    }

    // RT
    void dispatch(kr2::kord::ControlInterface& ctl) override {
        // actual KORD-API call
        ctl.setLoad(load_id_, mass_, cog_, inertia_, token_);
        state_.store(KordServiceState::DISPATCHED, std::memory_order_release);
    }

    // RT
    void poll(kr2::kord::ReceiverInterface& rcv) override {
        int8_t raw_status = rcv.getCommandStatus(token_);
        
        // -1 means token not received yet
        if (raw_status == -1) {
            ticks_++;
            if (ticks_timeout_ > 0 && ticks_ >= ticks_timeout_) {
                state_.store(KordServiceState::TIMEOUT, std::memory_order_release);
            }
            return;
        }

        status_.store(raw_status, std::memory_order_release);
        auto kord_status = static_cast<kr2::kord::protocol::EControlCommandStatus>(raw_status);
        
        // Exact enum from <kord/protocol/DataDescriptions/Requests/ControlCommandStatus.h>
        // enum EControlCommandStatus : uint16_t {
        //     eNone = 0, /**< No status */
        //     eIdle, /**< Idle */
        //     eInProgress, /**< In progress */
        //     eSuccess, /**< Success */
        //     eFailure /**< Failure */
        // };
        switch (kord_status) {
            case kr2::kord::protocol::EControlCommandStatus::eSuccess:
                state_.store(KordServiceState::SUCCESS, std::memory_order_release);
                break;
            case kr2::kord::protocol::EControlCommandStatus::eFailure:
                state_.store(KordServiceState::FAILURE, std::memory_order_release);
                break;
            case kr2::kord::protocol::EControlCommandStatus::eInProgress:
                state_.store(KordServiceState::DISPATCHED, std::memory_order_release);
                break;
            case kr2::kord::protocol::EControlCommandStatus::eNone:
                // For instantaneous setup commands like SetLoad, eNone is the 
                // terminal acknowledgment. We map it to SUCCESS to advance the state engine.
                state_.store(KordServiceState::SUCCESS, std::memory_order_release); 
                break;
            case kr2::kord::protocol::EControlCommandStatus::eIdle:
                state_.store(KordServiceState::IDLE, std::memory_order_release);
                break;
            default:
                state_.store(KordServiceState::FAILURE, std::memory_order_release);
                break;
        }
    }
};

struct KordServices {
    
    KordServiceSetLoad set_load;
    // here we can add new commands
    // KordServiceSetOPortCommand set_oport;
    // KordServiceSetFrameCommand set_frame;
    static constexpr size_t NUM_SERVICES = 1;

    private:
        std::array<KordServiceInterface*, NUM_SERVICES> kord_services_array_;

    public:
        KordServices() : kord_services_array_{ &set_load /*, and here: &oport, &frame */ } {}

        inline const std::array<KordServiceInterface*, NUM_SERVICES>& as_array() const {
            return kord_services_array_;
        }
};

#endif // KASSOW_KORD_HARDWARE_INTERFACE__ASYNC_COMMANDS_HPP_