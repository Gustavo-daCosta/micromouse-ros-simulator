#include "rclcpp/rclcpp.hpp"
#include "robot.hpp"
#include "pathfinding.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int exploration_steps = 0;
bool target_found = false;

bool searchTargetDFS(rclcpp::Node::SharedPtr node, Robot& robot, Pathfinding& pathfinder,
    Position current_pos, const Position& target) {
    
    pathfinder.markVisited(current_pos);
    exploration_steps++;
    
    // Verificar se chegamos ao target
    if (current_pos == target) {
        RCLCPP_INFO(node->get_logger(), "TARGET ALCANCADO!");
        target_found = true;
        return true;
    }
    
    rclcpp::spin_some(node);
    SensorData sensors = robot.getSensorData();
    pathfinder.updateMapFromSensors(current_pos, sensors);
    
    if (exploration_steps % 50 == 0) {
        RCLCPP_INFO(node->get_logger(), "Explorando... passo %d (Pos: %d,%d)", 
                   exploration_steps, current_pos.x, current_pos.y);
    }
    
    // Verificar se o target está em alguma direção adjacente
    std::vector<std::string> directions = {"up", "right", "down", "left"};
    std::vector<Position> next_positions = {
        Position(current_pos.x - 1, current_pos.y),
        Position(current_pos.x, current_pos.y + 1),
        Position(current_pos.x + 1, current_pos.y),
        Position(current_pos.x, current_pos.y - 1)
    };
    
    for (size_t i = 0; i < directions.size(); i++) {
        Position next_pos = next_positions[i];
        
        // Verificar se o sensor detecta target nesta direção
        bool is_target_ahead = false;
        if (directions[i] == "up" && sensors.up == "t") is_target_ahead = true;
        if (directions[i] == "right" && sensors.right == "t") is_target_ahead = true;
        if (directions[i] == "down" && sensors.down == "t") is_target_ahead = true;
        if (directions[i] == "left" && sensors.left == "t") is_target_ahead = true;
        
        if (is_target_ahead) {
            RCLCPP_INFO(node->get_logger(), "TARGET ENCONTRADO na direcao %s!", directions[i].c_str());
            RCLCPP_INFO(node->get_logger(), "Movendo para o target...");
            
            bool success = robot.move(directions[i]);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            if (success) {
                exploration_steps++;
                Position final_pos = robot.getCurrentPosition();
                if (final_pos == target) {
                    target_found = true;
                    return true;
                }
            }
        }
    }
    
    // Explorar células não visitadas
    for (size_t i = 0; i < directions.size(); i++) {
        Position next_pos = next_positions[i];
        
        if (pathfinder.isVisited(next_pos)) continue;
        
        bool success = robot.move(directions[i]);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        if (success) {
            Position new_pos = robot.getCurrentPosition();
            
            if (new_pos.isValid()) {
                // Buscar recursivamente
                if (searchTargetDFS(node, robot, pathfinder, new_pos, target)) {
                    return true;  // Target encontrado! Não fazer backtrack
                }
                
                // Target não encontrado neste caminho - fazer backtrack
                std::string opposite = Pathfinding::getOppositeDirection(directions[i]);
                robot.move(opposite);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }
    
    return false;  // Target não encontrado nesta branch
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = rclcpp::Node::make_shared("maze_solver_part2");
        RCLCPP_INFO(node->get_logger(), "Iniciando solver com sensores...");

        Robot robot(node);
        
        std::cout << "Aguardando inicializacao dos sensores...\n";
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        rclcpp::spin_some(node);

        std::cout << "Obtendo posicao inicial...\n";
        robot.move("up");
        robot.move("down");
        
        Position start = robot.getCurrentPosition();
        Position target = robot.getTargetPosition();

        if (!start.isValid()) {
            RCLCPP_ERROR(node->get_logger(), "Não foi possivel obter posicao inicial!");
            rclcpp::shutdown();
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "Posição inicial: (%d, %d)", start.x, start.y);
        RCLCPP_INFO(node->get_logger(), "Posição alvo: (%d, %d)", target.x, target.y);

        // Verificar se já está no target
        if (start == target) {
            std::cout << "Alvo encontrado com sucesso!\n\n";
            rclcpp::shutdown();
            return 0;
        }

        int maze_size = 29;
        Pathfinding pathfinder(maze_size, maze_size);

        std::cout << "\nBUSCANDO TARGET COM DFS...\n";
        std::cout << "(Explorando com backtracking ate encontrar o alvo)\n\n";

        bool found = searchTargetDFS(node, robot, pathfinder, start, target);
        
        Position final_pos = robot.getCurrentPosition();
        
        std::cout << "\nEstatisticas:\n";
        std::cout << "   Passos totais (DFS): " << exploration_steps << "\n";
        std::cout << "   Posicao final: (" << final_pos.x << ", " << final_pos.y << ")\n";
        std::cout << "\n";

        if (found && final_pos == target) {
            std::cout << "Alvo encontrado com sucesso!\n\n";
            RCLCPP_INFO(node->get_logger(), "Target alcançado em %d passos!", exploration_steps);
        } else {
            std::cout << "Target não encontrado\n\n";
            RCLCPP_WARN(node->get_logger(), "Target não foi alcançado");
        }

        rclcpp::shutdown();
        return 0;

    } catch (const std::exception &e) {
        std::cerr << "Erro: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
}
