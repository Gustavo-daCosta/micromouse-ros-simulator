#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "types.hpp"

class Robot {
public:
    Robot(rclcpp::Node::SharedPtr node);
    
    // Chama o tópico de movimentação do robô com o parâmetro definido
    bool move(std::string direction);

    // Pega a posição atual do robô
    Position getCurrentPosition() const;

    // Pega a posição do alvo
    Position getTargetPosition() const;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    Position current_position_;
    Position target_position_;
};

#endif
