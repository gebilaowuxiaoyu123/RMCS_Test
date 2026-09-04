// ============================================================================
// 任务三 · 角度目标桥组件：把外部 ros2 话题的角度指令 → 卷绕角度误差
// ----------------------------------------------------------------------------
//   [ros2 topic pub /motor_demo/angle_cmd <Float64 rad>]  (外部 ROS2 话题)
//        │ create_subscription（复用 omni_infantry.cpp 里 gimbal_calibrate 的写法）
//        ▼
//   读内部当前角 /motor_demo/motor/angle （复用 DjiMotor 自动输出的角度）
//        ▼ std::remainder 卷绕到 [-π, π)  ← 优弧（走短弧）
//   出 /motor_demo/angle_error  → 给外环 ErrorPidController 吃
// ============================================================================

#include <cmath>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rmcs_core::controller::motor_demo {

class AngleTargetController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AngleTargetController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/motor_demo/motor/angle", current_angle_);
        register_output("/motor_demo/angle_error", angle_error_, 0.0);

        // 复用仓库 omni_infantry.cpp 的 create_subscription 模式（订阅一个外部 ROS2 话题）
        cmd_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "/motor_demo/angle_cmd", rclcpp::QoS{0},
            [this](std_msgs::msg::Float64::UniquePtr&& msg) { target_angle_ = msg->data; });
    }

    void update() override {
        if (!current_angle_.ready())
            return;

        // 优弧：误差卷绕到 [-π, π)，正=要往前走、负=要往回走（走短弧）
        // remainder(a, 2π) 的返回落在 (-π, π] —— 正是一圈里离当前角最近的那条有向弧
        *angle_error_ =
            std::remainder(target_angle_ - *current_angle_, 2.0 * std::numbers::pi);
    }

private:
    InputInterface<double> current_angle_;
    OutputInterface<double> angle_error_;

    double target_angle_ = 0.0;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_subscription_;
};

} // namespace rmcs_core::controller::motor_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor_demo::AngleTargetController, rmcs_executor::Component)
