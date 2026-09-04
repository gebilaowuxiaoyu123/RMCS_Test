// ============================================================================
// 任务二软件层组件③：固定目标速度源（无遥控测试/对比用）
//   出 /motor_demo/target_velocity = 参数 value（double 常量）
//   用途：让"理想值(目标转速)"成为一条话题，能被 ValueBroadcaster 转发给
//         Foxglove，与真实转速画在同一张图对比。等遥控器到位后，
//         停用本组件、启用 JoystickVelocityMapping（输出同一话题）即可无缝切换。
// ============================================================================

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::motor_demo {

class FixedTargetVelocity
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FixedTargetVelocity()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , value_(get_parameter("value").as_double()) {

        register_output("/motor_demo/target_velocity", target_velocity_, 0.0);
    }

    void update() override { *target_velocity_ = value_; }

private:
    double value_;
    OutputInterface<double> target_velocity_;
};

} // namespace rmcs_core::controller::motor_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor_demo::FixedTargetVelocity, rmcs_executor::Component)
