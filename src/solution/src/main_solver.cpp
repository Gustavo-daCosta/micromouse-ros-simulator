#include "rclcpp/rclcpp.hpp"
#include "map.hpp"
#include "robot.hpp"
#include "pathfinding.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = rclcpp::Node::make_shared("maze_solver");

        auto map_client = node->create_client<cg_interfaces::srv::GetMap>("/get_map");
        Map mapHandler;

        RCLCPP_INFO(node->get_logger(), "Obtendo mapa do servidor...");
        Maze maze = mapHandler.getMaze(node, map_client);

        if (maze.matrix.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Falha ao obter mapa");
            rclcpp::shutdown();
            return 1;
        }

        std::cout << "\nMAPA DO LABIRINTO (" << maze.height << "x" << maze.width << "):\n";

        Position start = mapHandler.findPosition(maze, 'r');
        Position target = mapHandler.findPosition(maze, 't');
        
        if (!target.isValid()) {
            target = Position(maze.height / 2, maze.width / 2);
            RCLCPP_WARN(node->get_logger(), "Alvo 't' nao encontrado, usando centro do labirinto: (%d, %d)",
                       target.x, target.y);
        }

        if (!start.isValid() || !target.isValid()) {
            RCLCPP_ERROR(node->get_logger(), "Posicoes inicial ou final nao encontradas!");
            RCLCPP_ERROR(node->get_logger(), "   Inicio: (%d, %d), Alvo: (%d, %d)",
                         start.x, start.y, target.x, target.y);
            rclcpp::shutdown();
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "Posicao inicial (robo): (%d, %d)", start.x, start.y);
        RCLCPP_INFO(node->get_logger(), "Posicao alvo: (%d, %d)", target.x, target.y);

        std::cout << "Procurando caminho mais curto...";
        Pathfinding pathfinder;
        Path path = pathfinder.findPath(maze, start, target);

        if (path.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Nenhum caminho encontrado!");
            rclcpp::shutdown();
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "Caminho encontrado! Comprimento: %zu passos", path.size() - 1);

        std::vector<std::string> directions = pathfinder.pathToDirections(path);
        
        std::cout << "\nMovimentos:\n";
        for (size_t i = 0; i < directions.size(); ++i) {
            std::cout << "   " << (i + 1) << ". " << directions[i] << "\n";
        }
        std::cout << "\n";

        std::cout << "Iniciando execucao no simulador...\n";
        Robot robot(node);

        int step = 1;
        for (const auto &direction : directions) {
            RCLCPP_INFO(node->get_logger(), "Passo %d/%zu: Movendo para '%s'",
                        step, directions.size(), direction.c_str());

            bool success = robot.move(direction);

            if (!success) {
                RCLCPP_ERROR(node->get_logger(), "Movimento falhou no passo %d", step);
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            step++;
        }

        Position final_pos = robot.getCurrentPosition();
        Position target_pos = robot.getTargetPosition();

        std::cout << "\n";
        if (final_pos == target_pos) {
            std::cout << "Alvo alcançado com sucesso!";
            RCLCPP_INFO(node->get_logger(), "Robo alcancou o alvo em %d passos!", step - 1);
        } else {
            RCLCPP_WARN(node->get_logger(), "Robo nao alcancou o alvo");
            RCLCPP_INFO(node->get_logger(), "   Posicao final: (%d, %d)", final_pos.x, final_pos.y);
            RCLCPP_INFO(node->get_logger(), "   Posicao alvo: (%d, %d)", target_pos.x, target_pos.y);
        }

        std::cout << "\n";
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("maze_solver"), "Erro: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}
