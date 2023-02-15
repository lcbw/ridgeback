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
 * Author: Bence Magyar
 */

//#include <tf/transform_datatypes.h>
// we'll need to include tf2_msgs and tf2 seperately

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <queue>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>
#include <eigen3/Eigen/Geometry>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.h>
#include <mecanum_drive_controller/mecanum_drive_controller.h>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

// TODO: Breaking Change:
// use_realigned_roller_joints_; was not ported - you'll have to just input the wheel's radius, srry

// ASSUMPTIONS:
// constant wheel radius
// 2 wheels per side

// this is just a helper function, so I don't see why this wouldn't work, has not been tested
static bool isCylinderOrSphere(const urdf::LinkConstSharedPtr &link)
{
    auto logger = get_node()->get_logger();
    if (!link) {
        RCLCPP_ERROR(logger, "Link == NULL.");
        return false;
    }

    if (!link->collision) {
        RCLCPP_ERROR(logger,
                     "Link %d does not have collision description. Add collision description "
                     "for link to urdf.",
                     link->name);
        return false;
    }

    if (!link->collision->geometry) {
        RCLCPP_ERROR(logger,
                     "Link %d does not have collision geometry description. Add collision "
                     "geometry description for link to urdf.",
                     link->name);
        return false;
    }

    if (link->collision->geometry->type != urdf::Geometry::CYLINDER
        && link->collision->geometry->type != urdf::Geometry::SPHERE) {
        RCLCPP_ERROR(logger, "Link %d does not have cylinder nor sphere geometry", link->name);
        return false;
    }

    return true;
}

namespace mecanum_drive_controller
{

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MecanumDriveController::MecanumDriveController()
    : controller_interface::ControllerInterface()
{}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BASICALLY PORTED* NOT TESTED!! //
controller_interface::return_type MecanumDriveController::init(const std::string &controller_name)
{
    try {
        // Create the parameter listener and get the parameters
        param_listener_ = std::make_shared<mecanum_drive_controller::ParamListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception &e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return controller_interface::return_type::OK;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BASICALLY PORTED* NOT TESTED!! //
controller_interface::InterfaceConfiguration MecanumDriveController::command_interface_configuration()
    const
{
    std::vector<std::string> conf_names;
    for (const auto &joint_name : params_.wheel_names) {
        conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
    }
    return {interface_configuration_type::INDIVIDUAL, conf_names};
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BASICALLY PORTED* NOT TESTED!! //
controller_interface::InterfaceConfiguration MecanumDriveController::state_interface_configuration()
    const
{
    std::vector<std::string> conf_names;
    for (const auto &joint_name : params_.wheel_names) {
        conf_names.push_back(joint_name + "/" + feedback_type());
    }
    return {interface_configuration_type::INDIVIDUAL, conf_names};
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BASICALLY PORTED* NOT TESTED!! //
controller_interface::return_type MecanumDriveController::update()
{
    auto logger = get_node()->get_logger();

    if (get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
        if (!is_halted) {
            halt();
            is_halted = true;
        }
        return controller_interface::return_type::OK;
    }

    const auto current_time = node_->get_clock()->now();

    std::shared_ptr<geometry_msgs::msg::TwistStamped> last_command_msg;
    received_velocity_msg_ptr_.get(last_command_msg);

    if (last_command_msg == nullptr) {
        RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
        return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = current_time - last_command_msg->header.stamp;

    // Brake if cmd_vel has timeout, override the stored command
    if (age_of_last_command > cmd_vel_timeout_) {
        last_command_msg->twist.linear.x = 0.0;
        last_command_msg->twist.linear.y = 0.0;
        last_command_msg->twist.angular.z = 0.0;
    }

    // command may be limited further by SpeedLimit,
    // without affecting the stored twist command
    <geometry_msgs::msg::TwistStamped> command = *last_command_msg;
    double &linear_command_x = command.twist.linear.x;
    double &linear_command_y = command.twist.linear.y;
    double &angular_command = command.twist.angular.z;
    previous_update_timestamp_ = current_time;

    // COMPUTE AND PUBLISH ODOMETRY
    if (params_.open_loop) {
        odometry_.updateOpenLoop(linear_command_x,
                                 linear_command_y,
                                 command.twist.angular.z,
                                 current_time);
    } else {
        for (size_t index = 0; index < 4;
             ++index) { //index < static_cast<size_t>(params_.wheels_per_side); ++index) {
                        //            const double left_feedback
            //                = registered_left_wheel_handles_[index].feedback.get().get_value();
            //            const double right_feedback
            //                = registered_right_wheel_handles_[index].feedback.get().get_value();
            const double feedback_[index] = wheel_handle_[index].feedback.get().get_value();

            if (std::isnan(feedback)) {
                RCLCPP_ERROR(logger,
                             "Either the left or right wheel %s is invalid for index [%zu]",
                             feedback_type(),
                             index);
                return controller_interface::return_type::ERROR;
            }
        }
        if (params_.position_feedback) {
            odometry_.update(left_feedback_mean, right_feedback_mean, current_time);
        } else {
            odometry_.updateFromVelocity(left_feedback_mean * wheel_radius
                                             * publish_period_.seconds(),
                                         right_feedback_mean * wheel_radius
                                             * publish_period_.seconds(),
                                         current_time);
        }
    }

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, odometry_.getHeading());

    if (last_state_publish_time_ + publish_period_ < current_time) {
        last_state_publish_time_ += publish_period_;

        if (realtime_odometry_publisher_->trylock()) {
            // Populate odom message and publish
            auto &odometry_message = realtime_odometry_publisher_->msg_;
            odometry_message.header.stamp = current_time;
            odometry_message.pose.pose.position.x = odometry_.getX();
            odometry_message.pose.pose.position.y = odometry_.getY();
            odometry_message.pose.pose.orientation
                = orientation; // unless it bitches out and wants the 4 part
            odometry_message.twist.twist.linear.x = odometry_.getLinearX();
            odometry_message.twist.twist.linear.y = odometry_.getLinearY();
            odometry_message.twist.twist.angular.z = odometry_.getAngular();
            realtime_odometry_publisher_->unlockAndPublish();
        }
        // Publish tf /odom frame
        if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock()) {
            auto &transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
            transform.header.stamp = current_time;
            transform.transform.translation.x = odometry_.getX();
            transform.transform.translation.y = odometry_.getY();
            transform.transform.rotation = orientation; // unless it bitches out and wants the 4 part
            realtime_odometry_transform_publisher_->unlockAndPublish();
        }
    }

    // Limit velocities and accelerations:
    auto &last_command = previous_commands_.back().twist;
    auto &second_to_last_command = previous_commands_.front().twist;

    limiter_linX_.limit(linear_command_x,
                        last_command.linear.x,
                        second_to_last_command.linear.x,
                        publish_period_.seconds());
    limiter_linY_.limit(linear_command_y,
                        last_command.linear.y,
                        second_to_last_command.linear.y,
                        publish_period_.seconds());
    limiter_ang_.limit(angular_command,
                       last_command.angular.z,
                       second_to_last_command.angular.z,
                       publish_period_.seconds());

    previous_commands_.pop();
    previous_commands_.emplace(command);

    // Compute wheels velocities (this is the actual ik):
    // NOTE: the input desired twist (from topic /cmd_vel) is a body twist.
    const double wheel_velocity_0 = 1.0 / wheel_radius
                                    * (linear_command_x - linear_command_y
                                       - wheels_k * angular_command);
    const double wheel_velocity_1 = 1.0 / wheel_radius
                                    * (linear_command_x + linear_command_y
                                       - wheels_k * angular_command);
    const double wheel_velocity_2 = 1.0 / wheel_radius
                                    * (linear_command_x - linear_command_y
                                       + wheels_k * angular_command);
    const double wheel_velocity_3 = 1.0 / wheel_radius
                                    * (linear_command_x + linear_command_y
                                       + wheels_k * angular_command);
    // Set wheels velocities:
    for (size_t index = 0; index < 4; ++index) {
        wheel_handle_[index].velocity.get().set_value(wheel_velocity_[index]);
    }
    return controller_interface::return_type::OK;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BASICALLY PORTED* NOT TESTED!! //
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumDriveController::on_configure(const rclcpp_lifecycle::State &state)
{
    auto logger = get_node()->get_logger();

    // update parameters if they have changed
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
        RCLCPP_INFO(logger, "Parameters were updated");
    }

    if (params_.wheel_names.empty()) {
        RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Set wheel params for the odometry computation // this is where wheel_k_ is calculated
    if (setWheelParamsFromUrdf(state)
        != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
        RCLCPP_ERROR(logger, "Unable to set wheel parameters");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

    odometry_.setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

    cmd_vel_timeout_ = std::chrono::milliseconds{
        static_cast<int>(get_node()->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
    publish_limited_velocity_ = get_node()->get_parameter("publish_limited_velocity").as_bool();
    use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();

    //  // Velocity and acceleration limits:
    limiter_linX_ = SpeedLimiter(params_.linear.x.has_velocity_limits,
                                 params_.linear.x.has_acceleration_limits,
                                 params_.linear.x.min_velocity,
                                 params_.linear.x.max_velocity,
                                 params_.linear.x.min_acceleration,
                                 params_.linear.x.max_acceleration);

    limiter_linY_ = SpeedLimiter(params_.linear.y.has_velocity_limits,
                                 params_.linear.y.has_acceleration_limits,
                                 params_.linear.y.min_velocity,
                                 params_.linear.y.max_velocity,
                                 params_.linear.y.min_acceleration,
                                 params_.linear.y.max_acceleration);

    limiter_ang_ = SpeedLimiter(params_.angular.z.has_velocity_limits,
                                params_.angular.z.has_acceleration_limits,
                                params_.angular.z.min_velocity,
                                params_.angular.z.max_velocity,
                                params_.angular.z.min_acceleration,
                                params_.angular.z.max_acceleration);

    if (!reset()) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    const geometry_msgs::msg::TwistStamped empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empty_twist));

    // Fill last two commands with default constructed commands
    previous_commands_.emplace(empty_twist);
    previous_commands_.emplace(empty_twist);

    // This replaces cmdVelCallback()

    // initialize command subscriber
    if (use_stamped_vel_) {
        velocity_command_subscriber_
            = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
                DEFAULT_COMMAND_TOPIC,
                rclcpp::SystemDefaultsQoS(),
                [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) -> void {
                    if (!subscriber_is_active_) {
                        RCLCPP_WARN(logger, "Can't accept new commands. subscriber is inactive");
                        return;
                    }
                    if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
                        RCLCPP_WARN_ONCE(
                            logger,
                            "Received TwistStamped with zero timestamp, setting it to current "
                            "time, this message will only be shown once");
                        msg->header.stamp = get_node()->get_clock()->now();
                    }
                    received_velocity_msg_ptr_.set(std::move(msg));
                });
    } else {
        velocity_command_unstamped_subscriber_
            = get_node()->create_subscription<geometry_msgs::msg::Twist>(
                DEFAULT_COMMAND_UNSTAMPED_TOPIC,
                rclcpp::SystemDefaultsQoS(),
                [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
                    if (!subscriber_is_active_) {
                        RCLCPP_WARN(logger, "Can't accept new commands. subscriber is inactive");
                        return;
                    }

                    // Write fake header in the stored stamped command
                    std::shared_ptr<geometry_msgs::msg::TwistStamped> twist_stamped;
                    received_velocity_msg_ptr_.get(twist_stamped);
                    twist_stamped->twist = *msg;
                    twist_stamped->header.stamp = get_node()->get_clock()->now();
                });
    }

    // The following portion REPLACES setupRtPublishersMsg
    // initialize odometry publisher and messasge
    odometry_publisher_
        = get_node()->create_publisher<nav_msgs::msg::Odometry>(DEFAULT_ODOMETRY_TOPIC,
                                                                rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_
        = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
            odometry_publisher_);

    std::string controller_namespace = std::string(get_node()->get_namespace());

    if (controller_namespace == "/") {
        controller_namespace = "";
    } else {
        controller_namespace = controller_namespace + "/";
    }

    const auto odom_frame_id = controller_namespace + params_.odom_frame_id;
    const auto base_frame_id = controller_namespace + params_.base_frame_id;

    // this renamed from odom_pub_
    auto &odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.frame_id = controller_namespace + odom_frame_id;
    odometry_message.child_frame_id = controller_namespace + base_frame_id;

    // limit the publication on the topics /odom and /tf
    publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
    previous_publish_timestamp_ = get_node()->get_clock()->now();

    // initialize odom values zeros
    odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(
        rosidl_runtime_cpp::MessageInitialization::ALL);

    constexpr size_t NUM_DIMENSIONS = 6;
    for (size_t index = 0; index < 6; ++index) {
        // 0, 7, 14, 21, 28, 35
        const size_t diagonal_index = NUM_DIMENSIONS * index + index;
        odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
        odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];

        // we'll see if the above is equivalent to:
        //        odometry_message.pose.covariance = boost::assign::list_of(static_cast<double>(pose_cov_list[0]))(
        //            0)(0)(0)(0)(0)(0)(static_cast<double>(pose_cov_list[1]))(0)(0)(0)(0)(0)(0)(
        //            static_cast<double>(pose_cov_list[2]))(0)(0)(0)(0)(0)(0)(static_cast<double>(
        //            pose_cov_list[3]))(0)(0)(0)(0)(0)(0)(static_cast<double>(pose_cov_list[4]))(0)(0)(0)(0)(0)(
        //            0)(static_cast<double>(pose_cov_list[5]));

        //        odometry_message.twist.covariance = boost::assign::list_of(static_cast<double>(
        //            twist_cov_list[0]))(0)(0)(0)(0)(0)(0)(static_cast<double>(twist_cov_list[1]))(0)(0)(0)(0)(
        //            0)(0)(static_cast<double>(twist_cov_list[2]))(0)(0)(0)(0)(0)(0)(static_cast<double>(
        //            twist_cov_list[3]))(0)(0)(0)(0)(0)(0)(static_cast<double>(twist_cov_list[4]))(0)(0)(0)(0)(
        //            0)(0)(static_cast<double>(twist_cov_list[5]));
    }

    // initialize transform publisher and message
    odometry_transform_publisher_
        = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(DEFAULT_TRANSFORM_TOPIC,
                                                                 rclcpp::SystemDefaultsQoS());
    realtime_odometry_transform_publisher_
        = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
            odometry_transform_publisher_);

    // this renamed from tf_odom_pub_
    // keeping track of odom and base_link transforms only
    auto &odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
    odometry_transform_message.transforms.resize(1);
    odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
    odometry_transform_message.transforms.front().child_frame_id = base_frame_id;
    odometry_transform_message.transforms.front().transform.translation.z = 0.0;

    previous_update_timestamp_ = get_node()->get_clock()->now();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
//////////////////////////////////////////////////////////////////////

void MecanumDriveController::starting(const rclcpp::Time &time)
{
    brake();
    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;

    odometry_.init(time);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SHOULD BE PORTED!! //
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumDriveController::on_shutdown(const rclcpp_lifecycle::State &)
{
    brake();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumDriveController::on_activate(const rclcpp_lifecycle::State &)
{
    auto logger = get_node()->get_logger();
    const auto configuration_result = configure_wheel_handles(params_.wheel_names, wheel_handle_);

    if (configuration_result
        == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    if (wheel_handle_.empty()) {
        RCLCPP_ERROR(logger,
                     "Either left wheel interfaces, right wheel interfaces are non existent");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    is_halted = false;
    subscriber_is_active_ = true;

    RCLCPP_DEBUG(logger, "Subscriber and publisher are now active.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
    subscriber_is_active_ = false;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumDriveController::on_cleanup(const rclcpp_lifecycle::State &)
{
    if (!reset()) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    received_velocity_msg_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumDriveController::on_error(const rclcpp_lifecycle::State &)
{
    if (!reset()) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool MecanumDriveController::reset()
{
    //    odometry_.resetOdometry(); // this does not exist in our Odometry script at the moment

    // release the old queue
    std::queue<geometry_msgs::msg::TwistStamped> empty;
    std::swap(previous_commands_, empty);

    wheel_handle_.clear();

    subscriber_is_active_ = false;
    velocity_command_subscriber_.reset();
    velocity_command_unstamped_subscriber_.reset();

    received_velocity_msg_ptr_.set(nullptr);
    is_halted = false;
    return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumDriveController::configure_wheel_handles(const std::vector<std::string> &wheel_names,
                                                std::vector<WheelHandle> &registered_handles)
{
    auto logger = get_node()->get_logger();

    // register handles
    registered_handles.reserve(wheel_names.size());
    for (const auto &wheel_name : wheel_names) {
        const auto interface_name = feedback_type();
        const auto state_handle
            = std::find_if(state_interfaces_.cbegin(),
                           state_interfaces_.cend(),
                           [&wheel_name, &interface_name](const auto &interface) {
                               return interface.get_name() == wheel_name
                                      && interface.get_interface_name() == interface_name;
                               //the helper function interface.get_prefix_name() does not exist in foxy.
                               // we'll see how get_name() fails to do what we need here
                           });

        if (state_handle == state_interfaces_.cend()) {
            RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        const auto command_handle = std::find_if(command_interfaces_.begin(),
                                                 command_interfaces_.end(),
                                                 [&wheel_name](const auto &interface) {
                                                     return interface.get_name() == wheel_name
                                                            && interface.get_interface_name()
                                                                   == HW_IF_VELOCITY;
                                                 });

        if (command_handle == command_interfaces_.end()) {
            RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        registered_handles.emplace_back(
            WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SHOULD BE PORTED!! //
void MecanumDriveController::brake()
{
    for (size_t index = 0; index < 4; ++index) {
        wheel_handle_[index].velocity.get().set_value(0.0);
    };
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SHOULD BE PORTED!! //
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumDriveController::setWheelParamsFromUrdf(const rclcpp_lifecycle::State &)
{
    auto logger = get_node()->get_logger();
    bool has_wheel_separation_x = params_.wheel_separation_x > 0.0;
    bool has_wheel_separation_y = params_.wheel_separation_y > 0.0;

    if (has_wheel_separation_x != has_wheel_separation_y) {
        RCLCPP_ERROR(logger, "Only one wheel separation overrided");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    const double wheel_separation_x = params_.wheel_separation_x;
    const double wheel_separation_y = params_.wheel_separation_y;

    bool lookup_wheel_separation = ((params_.wheel_separation_x > 0.0)
                                    && (params_.wheel_separation_y > 0.0));
    bool lookup_wheel_radius = (params_.wheel_radius > 0.0);

    // Avoid URDF requirement if wheel separation and radius already specified
    /// TODO  BRACKET MISMATCH
    if (lookup_wheel_separation || lookup_wheel_radius) {
        urdf::ModelInterfaceSharedPtr model(urdf::Model::initFile(filename_))
            // Get wheels position and compute parameter k_ (used in mecanum wheels IK).
            urdf::JointConstSharedPtr wheel0_urdfJoint(model->getJoint(wheel0_name));
        if (!wheel0_urdfJoint) {
            RCLCPP_ERROR(logger, "%s couldn't be retrieved from model description", wheel0_name);
            return false;
        }
        urdf::JointConstSharedPtr wheel1_urdfJoint(model->getJoint(wheel1_name));
        if (!wheel1_urdfJoint) {
            RCLCPP_ERROR(logger, "%s couldn't be retrieved from model description", wheel1_name);
            return false;
        }
        urdf::JointConstSharedPtr wheel2_urdfJoint(model->getJoint(wheel2_name));
        if (!wheel2_urdfJoint) {
            RCLCPP_ERROR(logger, "%s couldn't be retrieved from model description", wheel2_name);
            return false;
        }
        urdf::JointConstSharedPtr wheel3_urdfJoint(model->getJoint(wheel3_name));
        if (!wheel3_urdfJoint) {
            RCLCPP_ERROR(logger, "%s couldn't be retrieved from model description", wheel3_name);
            return false;
        }

        if (lookup_wheel_separation) {
            RCLCPP_INFO(logger,
                        "wheel0 to origin: %d, %d, %d",
                        wheel0_urdfJoint->parent_to_joint_origin_transform.position.x,
                        wheel0_urdfJoint->parent_to_joint_origin_transform.position.y,
                        wheel0_urdfJoint->parent_to_joint_origin_transform.position.z);
            RCLCPP_INFO(logger,
                        "wheel1 to origin: %d, %d, %d",
                        wheel1_urdfJoint->parent_to_joint_origin_transform.position.x,
                        wheel1_urdfJoint->parent_to_joint_origin_transform.position.y,
                        wheel1_urdfJoint->parent_to_joint_origin_transform.position.z);
            RCLCPP_INFO(logger,
                        "wheel2 to origin: %d, %d, %d",
                        wheel2_urdfJoint->parent_to_joint_origin_transform.position.x,
                        wheel2_urdfJoint->parent_to_joint_origin_transform.position.y,
                        wheel2_urdfJoint->parent_to_joint_origin_transform.position.z);

            RCLCPP_INFO(logger,
                        "wheel3 to origin: %d, %d, %d",
                        wheel3_urdfJoint->parent_to_joint_origin_transform.position.x,
                        wheel3_urdfJoint->parent_to_joint_origin_transform.position.y,
                        wheel3_urdfJoint->parent_to_joint_origin_transform.position.z);

            double wheel0_x = wheel0_urdfJoint->parent_to_joint_origin_transform.position.x;
            double wheel0_y = wheel0_urdfJoint->parent_to_joint_origin_transform.position.y;
            double wheel1_x = wheel1_urdfJoint->parent_to_joint_origin_transform.position.x;
            double wheel1_y = wheel1_urdfJoint->parent_to_joint_origin_transform.position.y;
            double wheel2_x = wheel2_urdfJoint->parent_to_joint_origin_transform.position.x;
            double wheel2_y = wheel2_urdfJoint->parent_to_joint_origin_transform.position.y;
            double wheel3_x = wheel3_urdfJoint->parent_to_joint_origin_transform.position.x;
            double wheel3_y = wheel3_urdfJoint->parent_to_joint_origin_transform.position.y;

            wheels_k = (-(-wheel0_x - wheel0_y) - (wheel1_x - wheel1_y) + (-wheel2_x - wheel2_y)
                        + (wheel3_x - wheel3_y))
                       / 4.0;
        } else {
            RCLCPP_INFO(logger, "Wheel seperation in X: %d", wheel_separation_x);
            RCLCPP_INFO(logger, "Wheel seperation in Y: %d", wheel_separation_y);

            // The seperation is the total distance between the wheels in X and Y.

            wheels_k = (wheel_separation_x + wheel_separation_y) / 2.0;
        }

        if (lookup_wheel_radius) {
            // Get wheels radius
            double wheel0_radius = 0.0;
            double wheel1_radius = 0.0;
            double wheel2_radius = 0.0;
            double wheel3_radius = 0.0;

            if (!getWheelRadius(model,
                                model->getLink(wheel0_urdfJoint->child_link_name),
                                wheel0_radius)
                || !getWheelRadius(model,
                                   model->getLink(wheel1_urdfJoint->child_link_name),
                                   wheel1_radius)
                || !getWheelRadius(model,
                                   model->getLink(wheel2_urdfJoint->child_link_name),
                                   wheel2_radius)
                || !getWheelRadius(model,
                                   model->getLink(wheel3_urdfJoint->child_link_name),
                                   wheel3_radius)) {
                RCLCPP_ERROR(logger, "Couldn't retrieve wheels' radius");
                return false;
            }

            if (abs(wheel0_radius - wheel1_radius) > 1e-3
                || abs(wheel0_radius - wheel2_radius) > 1e-3
                || abs(wheel0_radius - wheel3_radius) > 1e-3) {
                RCLCPP_ERROR(logger, "Wheels radius are not egual");
                return false;
            }

            wheel_radius = wheel0_radius;
        }
    }

    RCLCPP_INFO(logger, "Wheel radius: %d", wheel_radius);

    // Set wheel params for the odometry computation
    odometry_.setWheelsParams(wheels_k, wheel_radius);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// THIS JUST ASSUMES THAT URDF JUST WORKS THE SAME :D YEAH RIGHT AHAHAHAHA!! //

bool MecanumDriveController::getWheelRadius(const urdf::ModelInterfaceSharedPtr model,
                                            const urdf::LinkConstSharedPtr &wheel_link,
                                            double &wheel_radius)
{
    auto logger = get_node()->get_logger();
    urdf::LinkConstSharedPtr radius_link = wheel_link;
    if (!isCylinderOrSphere(radius_link)) {
        RCLCPP_ERROR(logger, "Wheel link %d is NOT modeled as a cylinder!", radius_link->name);
        return false;
    }

    if (radius_link->collision->geometry->type == urdf::Geometry::CYLINDER)
        wheel_radius = (static_cast<urdf::Cylinder *>(radius_link->collision->geometry.get()))
                           ->radius;
    else
        wheel_radius = (static_cast<urdf::Sphere *>(radius_link->collision->geometry.get()))->radius;
    return true;
}
} // namespace mecanum_drive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mecanum_drive_controller::MecanumDriveController,
                       controller_interface::ControllerInterface)
