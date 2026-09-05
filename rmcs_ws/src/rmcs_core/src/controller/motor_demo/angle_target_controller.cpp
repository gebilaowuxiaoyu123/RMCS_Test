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
        // QoS{1}=KeepLast(1)：收发各存最近1帧即可，避免 QoS{0} 在 jazzy 的 KEEP_LAST 警告
        cmd_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "/motor_demo/angle_cmd", rclcpp::QoS{1},
            [this](std_msgs::msg::Float64::UniquePtr&& msg) {
                target_angle_ = msg->data;
                has_command_ = true;   // 收到第一条指令后，目标不再跟随当前角
            });
    }

    void update() override {
        if (!current_angle_.ready())
            return;

        // 安全：还没收到角度指令前，把目标锁在当前角 → 一启动电机不乱转（停在原地）
        if (!has_command_)
            target_angle_ = *current_angle_;

        // 优弧：误差卷绕到 [-π, π)，正=要往前走、负=要往回走（走短弧）
        // remainder(a, 2π) 的返回落在 (-π, π] —— 正是一圈里离当前角最近的那条有向弧
        *angle_error_ =
            std::remainder(target_angle_ - *current_angle_, 2.0 * std::numbers::pi);
    }

private:
    InputInterface<double> current_angle_;
    OutputInterface<double> angle_error_;

    double target_angle_ = 0.0;
    bool has_command_ = false;   // 是否已收到用户角度指令
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_subscription_;
};

} // namespace rmcs_core::controller::motor_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor_demo::AngleTargetController, rmcs_executor::Component)
