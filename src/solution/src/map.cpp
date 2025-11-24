#include "map.hpp"
#include <memory>
#include <chrono>
#include <string>
#include <iostream>

Maze Map::getMaze(rclcpp::Node::SharedPtr node, rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client) {
	Maze emptyMaze;

	// Espera até o serviço /get_map estar disponível
	int attempts = 0;
	while (!client->wait_for_service(std::chrono::seconds(1))) {
		RCLCPP_INFO(node->get_logger(), "Acessando o serviço /get_map... (tentativa %d)", ++attempts);
		if (attempts >= 10) {
			RCLCPP_ERROR(node->get_logger(), "Não foi possível acessar o serviço /get_map");
			return emptyMaze;
		}
	}

	auto future = client->async_send_request(std::make_shared<cg_interfaces::srv::GetMap::Request>());

	if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(node->get_logger(), "Não foi possível acessar o serviço /get_map");
		return emptyMaze;
	}

	auto response = future.get();

	int height = static_cast<int>(response->occupancy_grid_shape[0]);
	int width = static_cast<int>(response->occupancy_grid_shape[1]);
	
	// Concatenar todos os elementos do array em uma string
	std::string flatGrid;
	for (const auto& element : response->occupancy_grid_flattened) {
		flatGrid += element;
	}
	
	Maze maze = stringToMaze(flatGrid, height, width);

	RCLCPP_INFO(node->get_logger(), "Mapa obtido com sucesso! (%d x %d)", height, width);

	return maze;
}

Position Map::findPosition(const Maze &map, char positionType) {
	Position nullPosition = Position(-1, -1);
	if (map.matrix.empty()) {
		return nullPosition;
	}

	for (int i = 0; i < map.height; ++i) {
		for (int j = 0; j < map.width; ++j) {
			if (map.matrix[i][j] == positionType) {
				Position foundPosition = Position(i, j);

				if (positionType == 'r') {
					Map::actualPosition_ = foundPosition;
				} else if (positionType == 't' || positionType == 'b') {
					Map::endPosition_ = foundPosition;
				}

				return foundPosition;
			}
		}
	}
	return nullPosition;
}

Maze stringToMaze(std::string flatGrid, int height, int width) {
	Maze maze;
	maze.height = height;
	maze.width = width;

	Matrix matrix;

	for (int i = 0; i < height; ++i) {
		std::vector<char> row;
		for (int j = 0; j < width; ++j) {
			row.push_back(flatGrid[i * width + j]);
		}
		matrix.push_back(row);
	}

	maze.matrix = matrix;
	return maze;
}
