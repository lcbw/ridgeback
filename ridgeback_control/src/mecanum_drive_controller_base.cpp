
#include "mecanum_drive_controller/mecanum_drive_controller_base.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

namespace mecanum_drive_controller {
MecanumDriveControllerBase::MecanumDriveControllerBase()
    : controller_interface::ControllerInterface()
    , rt_command_ptr_(nullptr)
    , joints_command_subscriber_(nullptr)
{}

controller_interface::CallbackReturn MecanumDriveControllerBase::on_init()
{
    try {
        declare_parameters();
    } catch (const std::exception &e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveControllerBase::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto ret = this->read_parameters();
    if (ret != controller_interface::CallbackReturn::SUCCESS) {
        return ret;
    }

    joints_command_subscriber_
        = get_node()->create_subscription<CmdType>("~/commands",
                                                   rclcpp::SystemDefaultsQoS(),
                                                   [this](const CmdType::SharedPtr msg) {
                                                       rt_command_ptr_.writeFromNonRT(msg);
                                                   });

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MecanumDriveControllerBase::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names = command_interface_types_;

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
MecanumDriveControllerBase::state_interface_configuration() const
{
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn MecanumDriveControllerBase::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    //  check if we have all resources defined in the "points" parameter
    //  also verify that we *only* have the resources defined in the "points" parameter
    // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        ordered_interfaces;
    if (!controller_interface::get_ordered_interfaces(command_interfaces_,
                                                      command_interface_types_,
                                                      std::string(""),
                                                      ordered_interfaces)
        || command_interface_types_.size() != ordered_interfaces.size()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Expected %zu command interfaces, got %zu",
                     command_interface_types_.size(),
                     ordered_interfaces.size());
        return controller_interface::CallbackReturn::ERROR;
    }

    // reset command buffer if a command came through callback when controller was inactive
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveControllerBase::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // reset command buffer
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MecanumDriveControllerBase::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    auto joint_commands = rt_command_ptr_.readFromRT();

    // no command received yet
    if (!joint_commands || !(*joint_commands)) {
        return controller_interface::return_type::OK;
    }

    if ((*joint_commands)->data.size() != command_interfaces_.size()) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(),
                              *(get_node()->get_clock()),
                              1000,
                              "command size (%zu) does not match number of interfaces (%zu)",
                              (*joint_commands)->data.size(),
                              command_interfaces_.size());
        return controller_interface::return_type::ERROR;
    }

    for (auto index = 0ul; index < command_interfaces_.size(); ++index) {
        command_interfaces_[index].set_value((*joint_commands)->data[index]);
    }

    return controller_interface::return_type::OK;
}

} // namespace mecanum_drive_controller
