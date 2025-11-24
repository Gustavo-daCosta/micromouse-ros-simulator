#ifndef MAP_HPP
#define MAP_HPP

#include <vector>
#include <string>
#include "types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"

// Função para converter string em Maze
Maze stringToMaze(std::string flatGrid, int height, int width);

class Map {
public:
    // Função para capturar o Maze
    Maze getMaze(
        rclcpp::Node::SharedPtr node,
        rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client
    );

    // Função para capturar a posição inicial ou a posição do alvo
    // positionType = 't' -> retorna posição do target
    // positionType = 'r' -> retorna posição inicial  
    Position findPosition(const Maze &map, char positionType);

private:
    Maze maze_;
    Position actualPosition_;
    Position endPosition_;
};

#endif
