#pragma once
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <vector>
#include <string>

namespace inverse_kinematic_controller 
{
    class InverseKinematicController : public controller_interface::ControllerInterface
    {
        public:
        InverseKinematicController();
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        protected:
        std::vector<std::string> joint_names_;
        std::string interface_name_;
    };
};