#include <algorithm>
#include <cmath>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"

namespace rmcs_core::controller::gantry {

class GantryController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , lead_(get_parameter("lead").as_double())
        , gear_ratio_(get_parameter("gear_ratio").as_double())
        , sync_coefficient_(get_parameter("sync_coefficient").as_double())
        , max_velocity_(get_parameter("max_velocity").as_double())
        , max_acceleration_(get_parameter("max_acceleration").as_double())
        , position_pid_(
              get_parameter("position_kp").as_double(), get_parameter("position_ki").as_double(),
              get_parameter("position_kd").as_double())
        , velocity_pid_(
              get_parameter("velocity_kp").as_double(), get_parameter("velocity_ki").as_double(),
              get_parameter("velocity_kd").as_double()) {

        register_input("/gantry/left/angle", left_angle_);
        register_input("/gantry/right/angle", right_angle_);
        register_input("/gantry/left/velocity", left_velocity_);
        register_input("/gantry/right/velocity", right_velocity_);
        register_input("/gantry/left/max_torque", left_max_torque_);
        register_input("/gantry/right/max_torque", right_max_torque_);
        register_input("/predefined/update_rate", update_rate_, false);

        register_output("/gantry/target_height", target_height_);
        register_output("/gantry/left/control_torque", left_control_torque_);
        register_output("/gantry/right/control_torque", right_control_torque_);

        *target_height_ = get_parameter("target_height").as_double();
    }

    void before_updating() override {
        if (!update_rate_.ready())
            update_rate_.make_and_bind_directly(1000.0);
    }

    void update() override {
        const double dt = update_dt_();

        if (!left_angle_.ready() || !right_angle_.ready()) {
            *left_control_torque_ = 0.0;
            *right_control_torque_ = 0.0;
            return;
        }

        const double h_left = angle_to_height(*left_angle_);
        const double h_right = angle_to_height(*right_angle_);
        const double h = 0.5 * (h_left + h_right);

        const double height_error = *target_height_ - h;
        const double stopping_distance =
            planned_velocity_ * planned_velocity_ / (2.0 * max_acceleration_);
        const double desired_velocity = std::abs(height_error) > stopping_distance
                                          ? std::copysign(max_velocity_, height_error)
                                          : 0.0;
        planned_velocity_ += std::clamp(
            desired_velocity - planned_velocity_, -max_acceleration_ * dt, max_acceleration_ * dt);

        const Eigen::Vector2d position_error{*target_height_ - h_left, *target_height_ - h_right};
        const Eigen::Vector2d relative_position{h_left - h_right, h_right - h_left};

        Eigen::Vector2d velocity_setpoints =
            Eigen::Vector2d::Constant(planned_velocity_)
            + position_pid_.update(position_error - sync_coefficient_ * relative_position);
        velocity_setpoints = velocity_setpoints.cwiseMax(-max_velocity_).cwiseMin(max_velocity_);

        const Eigen::Vector2d motor_velocity_setpoints{
            line_to_motor_velocity(velocity_setpoints[0]),
            line_to_motor_velocity(velocity_setpoints[1])};
        const Eigen::Vector2d measured_velocity{*left_velocity_, *right_velocity_};
        const Eigen::Vector2d torque_limit{*left_max_torque_, *right_max_torque_};

        Eigen::Vector2d torques = velocity_pid_.update(motor_velocity_setpoints - measured_velocity);
        torques = torques.cwiseMax(-torque_limit).cwiseMin(torque_limit);

        const bool blocked =
            (std::abs(measured_velocity[0]) < 0.5 && std::abs(torques[0]) >= 0.9 * torque_limit[0])
            || (std::abs(measured_velocity[1]) < 0.5
                && std::abs(torques[1]) >= 0.9 * torque_limit[1]);
        if (blocked) {
            torques.setZero();
            position_pid_.reset();
            velocity_pid_.reset();
        }

        *left_control_torque_ = torques[0];
        *right_control_torque_ = torques[1];
    }

private:
    double update_dt_() const {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return 1e-3;
    }

    double angle_to_height(double angle) const {
        return angle / (2.0 * std::numbers::pi) * lead_ / gear_ratio_;
    }

    double line_to_motor_velocity(double velocity) const {
        return velocity * 2.0 * std::numbers::pi * gear_ratio_ / lead_;
    }

    double lead_, gear_ratio_, sync_coefficient_, max_velocity_, max_acceleration_;
    double planned_velocity_ = 0.0;

    pid::MatrixPidCalculator<2> position_pid_, velocity_pid_;

    InputInterface<double> left_angle_, right_angle_, left_velocity_, right_velocity_;
    InputInterface<double> left_max_torque_, right_max_torque_, update_rate_;
    OutputInterface<double> target_height_, left_control_torque_, right_control_torque_;
};

} // namespace rmcs_core::controller::gantry

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gantry::GantryController, rmcs_executor::Component)
