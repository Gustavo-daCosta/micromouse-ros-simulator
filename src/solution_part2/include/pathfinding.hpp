#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include "types.hpp"
#include "sensor_data.hpp"
#include <vector>
#include <queue>
#include <algorithm>
#include <map>
#include <string>


class Pathfinding {
public:
    Pathfinding(int height, int width);
    
    // Atualiza mapa interno com dados dos sensores
    void updateMapFromSensors(const Position &current_pos, const SensorData &sensors);
    
    // Função auxiliar para obter direção oposta (backtracking)
    static std::string getOppositeDirection(const std::string& direction);
    
    // Verifica se o alvo foi descoberto
    bool hasFoundTarget(const Position& target) const;
    
    // Verifica se célula foi visitada
    bool isVisited(const Position &pos) const;
    
    // Marca célula como visitada
    void markVisited(const Position &pos);
    
    // Marca célula como target
    void markAsTarget(const Position &pos);
    
    // Obtem mapa atual
    Maze getCurrentMaze() const;
    
    // Funções auxiliares
    static bool isValidPosition(const Maze &maze, const Position &pos);
    static std::vector<Position> getNeighbors(const Maze &maze, const Position &pos);

private:
    Maze internal_maze_;
    std::vector<std::vector<bool>> visited_cells_;
    std::vector<std::vector<bool>> target_cells_;
    
    // Atualizar célula específica no mapa
    void updateCell(const Position &pos, char value);
};

#endif
