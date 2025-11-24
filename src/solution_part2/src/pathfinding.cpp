#include "pathfinding.hpp"
#include <algorithm>
#include <iostream>

bool Pathfinding::isValidPosition(const Maze &maze, const Position &pos) {
    if (pos.x < 0 || pos.x >= maze.height || pos.y < 0 || pos.y >= maze.width) {
        return false;
    }
    
    char cell = maze.matrix[pos.x][pos.y];
    return (cell == 'f' || cell == 'r' || cell == 't');
}

std::vector<Position> Pathfinding::getNeighbors(const Maze &maze, const Position &pos) {
    std::vector<Position> neighbors;
    
    std::vector<Position> candidates = {
        Position(pos.x - 1, pos.y),
        Position(pos.x + 1, pos.y),
        Position(pos.x, pos.y - 1),
        Position(pos.x, pos.y + 1)
    };
    
    for (const Position& candidate : candidates) {
        if (isValidPosition(maze, candidate)) {
            neighbors.push_back(candidate);
        }
    }
    
    return neighbors;
}

Pathfinding::Pathfinding(int height, int width) {
    internal_maze_.height = height;
    internal_maze_.width = width;
    internal_maze_.matrix.resize(height, std::vector<char>(width, '?'));
    
    visited_cells_.resize(height, std::vector<bool>(width, false));
    target_cells_.resize(height, std::vector<bool>(width, false));
}

void Pathfinding::updateCell(const Position &pos, char value) {
    if (pos.x >= 0 && pos.x < internal_maze_.height && 
        pos.y >= 0 && pos.y < internal_maze_.width) {
        
        // Se estamos marcando como target, proteger permanentemente
        if (value == 't') {
            internal_maze_.matrix[pos.x][pos.y] = 't';
            target_cells_[pos.x][pos.y] = true;
            return;
        }
        
        // NUNCA sobrescrever celula protegida como target
        if (target_cells_[pos.x][pos.y]) {
            return;
        }
        
        char current = internal_maze_.matrix[pos.x][pos.y];
        
        // Nunca sobrescrever 't' no mapa
        if (current == 't') {
            target_cells_[pos.x][pos.y] = true;  // Marcar proteção
            return;
        }
        
        // Nao sobrescrever 'f' (visitado) com 'b' (barreira)
        if (current == 'f' && value == 'b') {
            return;
        }
        
        internal_maze_.matrix[pos.x][pos.y] = value;
    }
}

void Pathfinding::updateMapFromSensors(const Position &current_pos, const SensorData &sensors) {
    // PRIMEIRA PASSADA: Detectar e marcar TARGET antes de qualquer outra coisa
    if (current_pos.x > 0 && sensors.up == "t") {
        Position up(current_pos.x - 1, current_pos.y);
        updateCell(up, 't');
    }
    if (current_pos.x < internal_maze_.height - 1 && sensors.down == "t") {
        Position down(current_pos.x + 1, current_pos.y);
        updateCell(down, 't');
    }
    if (current_pos.y > 0 && sensors.left == "t") {
        Position left(current_pos.x, current_pos.y - 1);
        updateCell(left, 't');
    }
    if (current_pos.y < internal_maze_.width - 1 && sensors.right == "t") {
        Position right(current_pos.x, current_pos.y + 1);
        updateCell(right, 't');
    }
    
    // SEGUNDA PASSADA: Marcar posicao atual e demais células
    char current_cell = internal_maze_.matrix[current_pos.x][current_pos.y];
    if (current_cell != 't') {
        updateCell(current_pos, 'f');
    }
    
    if (current_pos.x > 0) {
        Position up(current_pos.x - 1, current_pos.y);
        if (sensors.up != "t") {  // Ja marcou acima se fosse 't'
            char cell_value = (sensors.up == "f" || sensors.up == " ") ? 'f' : 'b';
            updateCell(up, cell_value);
        }
    }
    
    if (current_pos.x < internal_maze_.height - 1) {
        Position down(current_pos.x + 1, current_pos.y);
        if (sensors.down != "t") {
            char cell_value = (sensors.down == "f" || sensors.down == " ") ? 'f' : 'b';
            updateCell(down, cell_value);
        }
    }
    
    if (current_pos.y > 0) {
        Position left(current_pos.x, current_pos.y - 1);
        if (sensors.left != "t") {
            char cell_value = (sensors.left == "f" || sensors.left == " ") ? 'f' : 'b';
            updateCell(left, cell_value);
        }
    }
    
    if (current_pos.y < internal_maze_.width - 1) {
        Position right(current_pos.x, current_pos.y + 1);
        if (sensors.right != "t") {
            char cell_value = (sensors.right == "f" || sensors.right == " ") ? 'f' : 'b';
            updateCell(right, cell_value);
        }
    }
}

bool Pathfinding::hasFoundTarget(const Position& target) const {
    if (target.x >= 0 && target.x < internal_maze_.height &&
        target.y >= 0 && target.y < internal_maze_.width) {
        char cell = internal_maze_.matrix[target.x][target.y];
        return (cell != '?');
    }
    return false;
}

std::string Pathfinding::getOppositeDirection(const std::string& direction) {
    if (direction == "up") return "down";
    if (direction == "down") return "up";
    if (direction == "left") return "right";
    if (direction == "right") return "left";
    return "";
}

bool Pathfinding::isVisited(const Position &pos) const {
    if (pos.x >= 0 && pos.x < internal_maze_.height && 
        pos.y >= 0 && pos.y < internal_maze_.width) {
        return visited_cells_[pos.x][pos.y];
    }
    return false;
}

void Pathfinding::markVisited(const Position &pos) {
    if (pos.x >= 0 && pos.x < internal_maze_.height &&
        pos.y >= 0 && pos.y < internal_maze_.width) {
        visited_cells_[pos.x][pos.y] = true;
        // NAO atualizar o mapa aqui - deixar updateMapFromSensors fazer isso
        // Porque updateMapFromSensors pode detectar 't' e tem prioridade
    }
}

void Pathfinding::markAsTarget(const Position &pos) {
    if (pos.x >= 0 && pos.x < internal_maze_.height &&
        pos.y >= 0 && pos.y < internal_maze_.width) {
        updateCell(pos, 't');
    }
}

Maze Pathfinding::getCurrentMaze() const {
    return internal_maze_;
}
