#ifndef KASSOW_KORD_HARDWARE_INTERFACE__ROS_SERVICES_HPP_
#define KASSOW_KORD_HARDWARE_INTERFACE__ROS_SERVICES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/srv/set_payload.hpp>
#include <control_msgs/srv/get_payload.hpp>
#include "kord_services.hpp"
#include <thread>
#include <chrono>
#include <variant>
#include <optional>

namespace kassow_kord_hardware_interface {

class KassowRosServices {
public:
    KassowRosServices(rclcpp::Node::SharedPtr node, kr2::kord::ReceiverInterface* rcv_iface) 
    : node_(node), rcv_iface_(rcv_iface){
        // allow concurrent service calls, but in each specific service we guard against concurrent calls of the same service
        reentrant_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        ros_server_set_load_ = node_->create_service<control_msgs::srv::SetPayload>(
            "~/set_payload",
            std::bind(&KassowRosServices::RosServiceCallback_SetPayload, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(),
            reentrant_callback_group_
        );

        ros_server_get_load_ = node_->create_service<control_msgs::srv::GetPayload>(
            "~/get_payload",
            std::bind(&KassowRosServices::RosServiceCallback_GetPayload, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(),
            reentrant_callback_group_
        );
    }

    inline const KordServices& get_kord_services() const {
        return kord_services_;
    }

    std::optional<kr2::kord::ELoadID> get_kord_load_id(uint8_t ros_load_type) {
        using SetPayloadRequest = control_msgs::srv::SetPayload::Request;
        switch(ros_load_type) {
            case SetPayloadRequest::LOAD_TYPE_END_EFFECTOR:
                return kr2::kord::ELoadID::LOAD1;
            case SetPayloadRequest::LOAD_TYPE_PAYLOAD:
                return kr2::kord::ELoadID::LOAD2;
            default:
                RCLCPP_ERROR(node_->get_logger(), 
                        "No KORD Load ID for ROS load_type: %d. Available ROS Load Types: END_EFFECTOR=%d, PAYLOAD=%d.", 
                        ros_load_type, 
                        SetPayloadRequest::LOAD_TYPE_END_EFFECTOR, 
                        SetPayloadRequest::LOAD_TYPE_PAYLOAD);
            return std::nullopt;
        }
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

    // TODO: ? pass cmd interface and more during init, for all KORD services to use?
    kr2::kord::ReceiverInterface* rcv_iface_;

    rclcpp::Service<control_msgs::srv::SetPayload>::SharedPtr ros_server_set_load_;
    rclcpp::Service<control_msgs::srv::GetPayload>::SharedPtr ros_server_get_load_;

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

        std::optional<kr2::kord::ELoadID> kord_load_id = get_kord_load_id(req->load_type);
        if (!kord_load_id) {
            res->success = false;
            return;
        }

        kord_services_.set_load.populate(
            kord_load_id.value(), 
            static_cast<double>(req->mass), 
            cog, 
            inertia, 
            timeout_ticks
        );
        kord_services_.set_load.request();

        while (rclcpp::ok()) {
            KordServiceState current_state = kord_services_.set_load.get_state();
            
            if (current_state == KordServiceState::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "Motion flags AFTER completion: %u", rcv_iface_->getMotionFlags());
                RCLCPP_INFO(
                    node_->get_logger(), "Set Payload successful. KORD service status response: %d", 
                    kord_services_.set_load.get_status());
                res->success = true;
                break;
            } else if (current_state == KordServiceState::FAILURE) {
                RCLCPP_ERROR(
                    node_->get_logger(), "Set Payload command failed at the Kassow controller. KORD service status response: %d", 
                    kord_services_.set_load.get_status());
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
            } else if (current_state == KordServiceState::DISPATCHED || 
                       current_state == KordServiceState::REQUESTED) {
                // Do nothing. command is processing.
            } else if (current_state == KordServiceState::IDLE) {
                RCLCPP_WARN(node_->get_logger(), "Set Payload state unexpectedly reset to IDLE during wait.");
                res->success = false;
                break;
            }
            else {
                RCLCPP_WARN(node_->get_logger(), "Set Payload unexpected state! This should not happen!");
                res->success = false;
                break;
            }
            // Sleep briefly to yield the CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        
        // reset for new request
        kord_services_.set_load.reset();
    }

    void RosServiceCallback_GetPayload(
        const std::shared_ptr<control_msgs::srv::GetPayload::Request> req,
        std::shared_ptr<control_msgs::srv::GetPayload::Response> res) 
    {
        std::optional<kr2::kord::ELoadID> kord_load_id = get_kord_load_id(req->load_type);
        if (!kord_load_id) {
            res->success = false;
            return;
        }

        //  mass
        auto mass_data = rcv_iface_->getLoad(kord_load_id.value(), kr2::kord::MASS_VAL);
        if (mass_data.empty()) {
            RCLCPP_WARN(node_->get_logger(), "Failed to get payload: Invalid Load ID.");
            res->success = false;
            return;
        }
        res->mass = static_cast<float>(std::get<double>(mass_data[0]));

        // CoG
        auto cog_data = rcv_iface_->getLoad(kord_load_id.value(), kr2::kord::COG_VAL);
        if (cog_data.size() >= 3) {
            res->center_of_gravity.x = std::get<double>(cog_data[0]);
            res->center_of_gravity.y = std::get<double>(cog_data[1]);
            res->center_of_gravity.z = std::get<double>(cog_data[2]);
        }

        // interia
        auto inertia_data = rcv_iface_->getLoad(kord_load_id.value(), kr2::kord::INERTIA_VAL);
        if (inertia_data.size() >= 6) {
            res->ixx = std::get<double>(inertia_data[0]);
            res->iyy = std::get<double>(inertia_data[1]);
            res->izz = std::get<double>(inertia_data[2]);
            res->ixy = std::get<double>(inertia_data[3]);
            res->ixz = std::get<double>(inertia_data[4]);
            res->iyz = std::get<double>(inertia_data[5]);
        }

        res->success = true;
    }
};

} // namespace kassow_kord_hardware_interface

#endif // KASSOW_KORD_HARDWARE_INTERFACE__ROS_SERVICES_HPP_