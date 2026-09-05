// ============================================================================
// 任务二硬件层：单个 M3508 电机 + DR16 遥控  (rmcs_core/src/hardware/test.cpp)
// ----------------------------------------------------------------------------
// M3508: CAN1, 拨码 id=3  (命令走 0x200 大帧第3槽; 反馈帧 0x203)
// DR16 : 板子 DBUS 串口 → RemoteControl 发布 /remote/joystick/* 等话题
// 板子 : CBoard，串口由参数 board_serial 注入(yaml)
// 本文件只做“碰硬件 + 出/收话题”：PID/摇杆映射/滤波都在启动配置里拼。
// ============================================================================

#include <cstddef>
#include <memory>
#include <span>
#include <utility>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class MotorTest
    : public rmcs_executor::Component               // RMCS 组件基类
    , public rclcpp::Node                           // ROS2 节点
    , public librmcs::board::CBoard::Callback {  // 板子 CAN/串口回调
public:
    MotorTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<MotorCommand>(get_component_name() + "_command", *this))
        // 设备构造：需要 status 组件(*this) + command 组件(*command_)，
        // 前缀决定话题名 → 自动注册 /motor_demo/motor/{angle,velocity,torque,max_torque,control_torque}
        , motor_(*this, *command_, "/motor_demo/motor")
        , dr16_{} {
        // 1) 配置电机：M3508, id=3。可选链：
        //    .set_reversed()           默认即可（实测速度方向正确，别加）
        //    .enable_multi_turn_angle()  ★必开：单圈角度在 2π↔0 会跳变导致角度环绕圈/抖动
        motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                .enable_multi_turn_angle());

        // 2) 打开板子（串口名来自 yaml 的 board_serial 参数）
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());

        // 3) 遥控：DR16 挂到 RemoteControl，由它发布 /remote/* 话题
        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);
    }

    // 每周期(1000Hz)：把收到的反馈解析成话题
    void update() override {
        motor_.update_status();        // 解析 CAN 反馈 → 刷新 /velocity 等输出
        dr16_.update_status();         // 解析 DBUS 遥控帧
        remote_control_->update();     // 发布 /remote/joystick/left 等
    }

    // 由伙伴组件 MotorCommand 错相调用：把命令发出去
    void command_update() {
        auto builder = board_->start_transmit();

        // operator<< 把电机电流放到 (id-1)%4 = 第3槽；send_id() = 0x200
        auto packet = device::CanPacket8{};
        packet << motor_;

        builder.can_transmit(
            Spec::kCans.kCan1,                      // 电机在 CAN1
            {.can_id = motor_.send_id(), .can_data = packet.as_bytes()});
    }

    // CAN1 收到反馈帧 → 按 id(0x203) 匹配后存入电机
    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        if (can == Spec::kCans.kCan1)
            motor_.match_then_store_status(data.can_id, data.can_data);
    }

    // 板子 DBUS 串口数据 → 喂给 DR16（遥控）
    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

private:
    std::unique_ptr<librmcs::board::CBoard> board_;

    // 伙伴“命令”组件：update 里回调 command_update()，与主 update 错相执行
    class MotorCommand : public rmcs_executor::Component {
    public:
        explicit MotorCommand(MotorTest& motor)
            : motor_(motor) {}
        void update() override { motor_.command_update(); }
    private:
        MotorTest& motor_;
    };
    std::shared_ptr<MotorCommand> command_;

    device::DjiMotor motor_;                        // 被控电机
    device::Dr16 dr16_;                             // 遥控解码
    std::unique_ptr<device::RemoteControl> remote_control_;  // DR16 → /remote/* 话题
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::MotorTest, rmcs_executor::Component)
