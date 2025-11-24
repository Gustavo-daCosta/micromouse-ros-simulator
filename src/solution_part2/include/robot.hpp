#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "types.hpp"
#include "sensor_data.hpp"

class Robot {
public:
    Robot(rclcpp::Node::SharedPtr node);

    // Executar movimento
    bool move(std::string direction);

    // Obter dados dos sensores atuais
    SensorData getSensorData() const;

    // Posições
    Position getCurrentPosition() const;
    Position getTargetPosition() const;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    
    Position current_position_;
    Position target_position_;
    SensorData current_sensors_;
    
    // Callback para atualizar sensores
    void sensorCallback(const cg_interfaces::msg::RobotSensors::SharedPtr msg);
};

#endif
