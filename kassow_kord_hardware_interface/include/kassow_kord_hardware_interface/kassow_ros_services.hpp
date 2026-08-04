#ifndef KASSOW_KORD_HARDWARE_INTERFACE__ROS_SERVICES_HPP_
#define KASSOW_KORD_HARDWARE_INTERFACE__ROS_SERVICES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/srv/set_payload.hpp>
#include "kord_services.hpp"
#include <thread>
#include <chrono>

namespace kassow_kord_hardware_interface {

class KassowRosServices {
public:
    KassowRosServices(rclcpp::Node::SharedPtr node) 
    : node_(node){
        // allow concurrent service calls, but in each specific service we guard against concurrent calls of the same service
        reentrant_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        ros_server_set_load_ = node_->create_service<control_msgs::srv::SetPayload>(
            "~/set_payload",
            std::bind(&KassowRosServices::RosServiceCallback_SetPayload, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            reentrant_callback_group_
        );
    }

    inline const KordServices& get_kord_services() const {
        return kord_services_;
    }

    // called on deactivation
    void abortActiveServices() {
        for (auto* service : kord_services_.as_array()) {
            KordServiceState state = service->get_state();
            if (state == KordServiceState::REQUESTED || state == KordServiceState::DISPATCHED) {
                service->abort();
            }
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;
    KordServices kord_services_;


    rclcpp::Service<control_msgs::srv::SetPayload>::SharedPtr ros_server_set_load_;

    void RosServiceCallback_SetPayload(
        const std::shared_ptr<control_msgs::srv::SetPayload::Request> req,
        std::shared_ptr<control_msgs::srv::SetPayload::Response> res) 
    {
        // IMPORTANT: no concurrent calls on the same service!
        if (kord_services_.set_load.get_state() != KordServiceState::IDLE) {
            RCLCPP_WARN(node_->get_logger(), "A SetLoad command is already in progress. Rejecting.");
            res->success = false;
            return;
        }

        std::array<double, 3> cog = { req->center_of_gravity.x, req->center_of_gravity.y, req->center_of_gravity.z };
        std::array<double, 6> inertia = { req->ixx, req->iyy, req->izz, req->ixy, req->ixz, req->iyz };
        double timeout_sec = req->timeout.sec + (req->timeout.nanosec * 1e-9);
        // TODO: get this update rate from hw interface
        size_t timeout_ticks = static_cast<size_t>(timeout_sec * 500.0);

        kord_services_.set_load.populate(
            static_cast<kr2::kord::ELoadID>(req->load_type), 
            static_cast<double>(req->mass), 
            cog, 
            inertia, 
            timeout_ticks
        );
        kord_services_.set_load.request();

        while (rclcpp::ok()) {
            KordServiceState current_state = kord_services_.set_load.get_state();
            
            if (current_state == KordServiceState::SUCCESS) {
                res->success = true;
                break;
            } else if (current_state == KordServiceState::FAILURE) {
                RCLCPP_ERROR(node_->get_logger(), "Set Payload command failed at the Kassow controller.");
                res->success = false;
                break;
            } else if (current_state == KordServiceState::TIMEOUT) {
                RCLCPP_ERROR(node_->get_logger(), "Set Payload command timed out.");
                res->success = false;
                break;
            } else if (current_state == KordServiceState::ABORTED) {
                RCLCPP_WARN(node_->get_logger(), "Set Payload command was executing but was aborted due to hardware deactivation.");
                res->success = false;
                break;
            }

            // Sleep briefly to yield the CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        
        // reset for new request
        kord_services_.set_load.reset();
    }
};

} // namespace kassow_kord_hardware_interface

#endif // KASSOW_KORD_HARDWARE_INTERFACE__ROS_SERVICES_HPP_