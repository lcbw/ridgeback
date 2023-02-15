/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Enrique Fernández
 */

#ifndef MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_
#define MECANUM_DRIVE_CONTROLLER__MECANUM_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <queue>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_list_macros.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "mecanum_drive_controller_parameters.hpp"
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <mecanum_drive_controller/odometry.h>
#include <mecanum_drive_controller/speed_limiter.h>
#include <mecanum_drive_controller/visibility_control.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rcpputils/asserts.hpp>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace mecanum_drive_controller {

/**
 * This class makes some assumptions on the model of the robot:
 *  - the rotation axes of wheels are collinear
 *  - the wheels are identical in radius
 * Additional assumptions to not duplicate information readily available in the URDF:
 *  - the wheels have the same parent frame
 *  - a wheel collision geometry is a cylinder in the urdf
 *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
 */
class MecanumDriveController : public controller_interface::ControllerInterface
{
public:
    MECANUM_DRIVE_CONTROLLER_PUBLIC
    MecanumDriveController();

    MECANUM_DRIVE_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    MECANUM_DRIVE_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /**
   * \brief Configures controller, partially replaces init(), replaces setupRtPublishersMsg & cmdVelCallback, and calls setWheelParamsFromUrdf
   */
    MECANUM_DRIVE_CONTROLLER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    /**
   * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
   * replaces void update 
   * \param time   Current time
   * \param period Time since the last called to update
   */
    MECANUM_DRIVE_CONTROLLER_PUBLIC
    controller_interface::return_type update() override;

    /**
   * \brief Initialize controller, partially replaces old init()
   */

    MECANUM_DRIVE_CONTROLLER_PUBLIC
    controller_interface::return_type init(const std::string &controller_name) override;

    //    /**
    //   * \brief Stops controller, this replaces stopping()
    //   */
    MECANUM_DRIVE_CONTROLLER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &previous_state) override;

    MECANUM_DRIVE_CONTROLLER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    MECANUM_DRIVE_CONTROLLER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    MECANUM_DRIVE_CONTROLLER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    MECANUM_DRIVE_CONTROLLER_PUBLIC
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
        const rclcpp_lifecycle::State &previous_state) override;

    //    /**
    //   * \brief Starts controller
    //   * \param time Current time
    //   */
    void starting(const rclcpp::Time &time);

protected:
    std::shared_ptr<mecanum_drive_controller::ParamListener> param_listener_;
    Params params_;

    // these are replacing wheel0_jointHandle_; & so forth
    struct WheelHandle
    {
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
    };

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn configure_wheel_handles(
        const std::vector<std::string> &wheel_names, std::vector<WheelHandle> &registered_handles);

    std::vector<WheelHandle> wheel_handle_;

    // Timeout to consider cmd_vel commands old
    std::chrono::milliseconds cmd_vel_timeout_{500};

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
        realtime_odometry_publisher_ = nullptr;

    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_
        = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
        realtime_odometry_transform_publisher_ = nullptr;

    bool subscriber_is_active_ = false;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_command_subscriber_
        = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_unstamped_subscriber_
        = nullptr;

    realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>>
        received_velocity_msg_ptr_{nullptr};

    std::queue<geometry_msgs::msg::TwistStamped> previous_commands_; // last two commands

    bool publish_limited_velocity_ = false;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> limited_velocity_publisher_
        = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>
        realtime_limited_velocity_publisher_ = nullptr;

    rclcpp::Time previous_update_timestamp_{0};

    // publish rate limiter
    double publish_rate_ = 50.0;
    rclcpp::Duration publish_period_{0, 0};
    rclcpp::Time previous_publish_timestamp_{0};

    bool reset();
    void halt();

private:
    std::string name_;
    std::string filename_;

    /// Odometry related:
    rclcpp::Time last_state_publish_time_;

    bool open_loop_;
    bool is_halted = false;
    bool use_stamped_vel_ = true;

    /// Odometry related:
    Odometry odometry_;
    geometry_msgs::msg::TransformStamped odom_frame_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheels_k; // wheels geometric param used in mecanum wheels' ik
    double wheel_radius;
    double wheel_separation_x;
    double wheel_separation_y;

    /// Frame to use for the robot base:
    std::string base_frame_id_;
    std::string odom_frame_id_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf;

    // Speed limiters:
    SpeedLimiter limiter_linX_;
    SpeedLimiter limiter_linY_;
    SpeedLimiter limiter_ang_;

private:
    /**
       * \brief Brakes the wheels, i.e. sets the velocity to 0
       */
    void brake(); //  void halt();

    /**
       * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and separation
       * \param root_nh Root node handle
       * \param wheel0_name Name of wheel0 joint
       * \param wheel1_name Name of wheel1 joint
       * \param wheel2_name Name of wheel2 joint
       * \param wheel3_name Name of wheel3 joint
       */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn setWheelParamsFromUrdf(
        const rclcpp_lifecycle::State &);

    /**
       * \brief Get the radius of a given wheel
       * \param       model         urdf model used
       * \param       wheel_link    link of the wheel from which to get the radius
       * \param[out]  wheels_radius radius of the wheel read from the urdf
       */
    bool getWheelRadius(const urdf::ModelInterfaceSharedPtr model,
                        const urdf::LinkConstSharedPtr &wheel_link,
                        double &wheel_radius);
};
//PLUGINLIB_EXPORT_CLASS(mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerBase)
} // namespace mecanum_drive_controller
#endif
