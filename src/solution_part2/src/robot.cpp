#include "robot.hpp"
#include <chrono>

Robot::Robot(rclcpp::Node::SharedPtr node) : node_(node) {
    move_client_ = node_->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    current_position_ = Position(-1, -1);
    target_position_ = Position(-1, -1);
    
    // Inicializar sensores
    current_sensors_.up = "";
    current_sensors_.down = "";
    current_sensors_.left = "";
    current_sensors_.right = "";
    current_sensors_.up_left = "";
    current_sensors_.up_right = "";
    current_sensors_.down_left = "";
    current_sensors_.down_right = "";
    
    // Subscriber para sensores
    sensor_sub_ = node_->create_subscription<cg_interfaces::msg::RobotSensors>(
        "/culling_games/robot_sensors", 10,
        std::bind(&Robot::sensorCallback, this, std::placeholders::_1)
    );
}

void Robot::sensorCallback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
    current_sensors_.up = msg->up;
    current_sensors_.down = msg->down;
    current_sensors_.left = msg->left;
    current_sensors_.right = msg->right;
    current_sensors_.up_left = msg->up_left;
    current_sensors_.up_right = msg->up_right;
    current_sensors_.down_left = msg->down_left;
    current_sensors_.down_right = msg->down_right;
}

bool Robot::move(std::string direction) {
    // Espera o serviço ficar disponível
    if (!move_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Não foi possível acessar o servico /move_command");
        return false;
    }

    // Cria e envia a requisição
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direction;
    auto future = move_client_->async_send_request(request);

    // Espera a resposta
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Erro ao acessar o servico /move_command");
        return false;
    }

    auto response = future.get();

    // Atualiza a posição atual
    current_position_ = Position(response->robot_pos[0], response->robot_pos[1]);
    target_position_ = Position(response->target_pos[0], response->target_pos[1]);

    return response->success;
}

SensorData Robot::getSensorData() const {
    return current_sensors_;
}

Position Robot::getCurrentPosition() const {
    return current_position_;
}

Position Robot::getTargetPosition() const {
    return target_position_;
}
