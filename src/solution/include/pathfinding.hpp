#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include "types.hpp"
#include <vector>
#include <queue>
#include <algorithm>

class Pathfinding {
public:
    // Encontra o melhor caminho com base no algoritmo
    Path findPath(const Maze &maze, const Position &start, const Position &end);

    // Converte Path 
    std::vector<std::string> pathToDirections(const Path &path);

    // Verifica se uma posição é válida (se é possível andar até ela)
    static bool isValidPosition(const Maze &maze, const Position &pos);

    // Captura os blocos vizinhos de uma posição
    static std::vector<Position> getNeighbors(const Maze &maze, const Position &pos);
};

// Gera a matriz de "pesos" do floodfill
std::vector<std::vector<int>> floodFill(const Maze &maze, const Position &target);

#endif
