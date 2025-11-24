#include "robot.hpp"
#include <chrono>

Robot::Robot(rclcpp::Node::SharedPtr node) : node_(node) {
    move_client_ = node_->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    current_position_ = Position(-1, -1);
    target_position_ = Position(-1, -1);
}

bool Robot::move(std::string direction) {
    // Espera o serviço estar disponível
    if (!move_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Service /move_command not available");
        return false;
    }

    // Crie e envia a requisição
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direction;
    auto future = move_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service /move_command");
        return false;
    }

    auto response = future.get();

    // Atualiza as posições
    current_position_ = Position(response->robot_pos[0], response->robot_pos[1]);
    target_position_ = Position(response->target_pos[0], response->target_pos[1]);

    if (!response->success) {
        RCLCPP_WARN(node_->get_logger(), "Move to '%s' failed", direction.c_str());
    }

    return response->success;
}

Position Robot::getCurrentPosition() const {
    return current_position_;
}

Position Robot::getTargetPosition() const {
    return target_position_;
}
