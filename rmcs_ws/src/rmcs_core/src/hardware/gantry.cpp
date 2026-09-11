#include <cstdint>
#include <memory>
#include <utility>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"

namespace rmcs_core::hardware {

class Gantry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    Gantry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<GantryCommand>(get_component_name() + "_command", *this))
        , left_motor_(*this, *command_, "/gantry/left")
        , right_motor_(*this, *command_, "/gantry/right") {

        left_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, kLeftCanId}
                .enable_multi_turn_angle());
        right_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508, kRightCanId}
                .set_reversed()
                .enable_multi_turn_angle());

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());
    }

    void update() override {
        left_motor_.update_status();
        right_motor_.update_status();
    }

    void command_update() {
        auto packet = device::CanPacket8{};
        packet << left_motor_ << right_motor_;

        board_->start_transmit().can_transmit(
            Spec::kCans.kCan1, {.can_id = left_motor_.send_id(), .can_data = packet.as_bytes()});
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        if (can != Spec::kCans.kCan1)
            return;
        left_motor_.match_then_store_status(data.can_id, data.can_data);
        right_motor_.match_then_store_status(data.can_id, data.can_data);
    }

private:
    class GantryCommand : public rmcs_executor::Component {
    public:
        explicit GantryCommand(Gantry& gantry)
            : gantry_(gantry) {}
        void update() override { gantry_.command_update(); }

    private:
        Gantry& gantry_;
    };

    static constexpr std::uint8_t kLeftCanId = 1;
    static constexpr std::uint8_t kRightCanId = 2;

    std::unique_ptr<librmcs::board::CBoard> board_;
    std::shared_ptr<GantryCommand> command_;

    device::DjiMotor left_motor_, right_motor_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Gantry, rmcs_executor::Component)
