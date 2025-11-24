#include "pathfinding.hpp"
#include "map.hpp"
#include <queue>
#include <limits>

// Função para verificar a posição é válida
bool Pathfinding::isValidPosition(const Maze &maze, const Position &pos) {
    // Verifica limites do mapa
    if (pos.x < 0 || pos.x >= maze.height || pos.y < 0 || pos.y >= maze.width) {
        return false;
    }

    // Verifica se é transitável (não é parede)
    // 'f' = livre, 'r' = robo, 't' = fim, 'b' = parede
    char cell = maze.matrix[pos.x][pos.y];
    return (cell == 'f' || cell == 'r' || cell == 't');
}

std::vector<Position> Pathfinding::getNeighbors(const Maze &maze, const Position &pos) {
    std::vector<Position> neighbors;

    // 4 direções: acima, baixo, esquerda, direita (sem diagonais)
    Position up(pos.x - 1, pos.y);
    Position down(pos.x + 1, pos.y);
    Position left(pos.x, pos.y - 1);
    Position right(pos.x, pos.y + 1);

    std::vector<Position> positions = {up, down, left, right};

    for (int i = 0; i < positions.size(); ++i) {
        if (isValidPosition(maze, positions[i])) {
            neighbors.push_back(positions[i]);
        }
    }

    return neighbors;
}

std::vector<std::string> Pathfinding::pathToDirections(const Path &path) {
    std::vector<std::string> directions;

    for (size_t i = 1; i < path.size(); ++i) {
        int dx = path[i].x - path[i - 1].x;
        int dy = path[i].y - path[i - 1].y;

        if (dx == -1 && dy == 0) {
            directions.push_back("up");
        } else if (dx == 1 && dy == 0) {
            directions.push_back("down");
        } else if (dx == 0 && dy == -1) {
            directions.push_back("left");
        } else if (dx == 0 && dy == 1) {
            directions.push_back("right");
        }
    }

    return directions;
}

Path Pathfinding::findPath(const Maze &maze, const Position &start, const Position &end) {
    // Gera a matriz de distâncias usando Floodfill a partir do target
    std::vector<std::vector<int>> floodMap = floodFill(maze, end);
    
    // Verificar se existe caminho (se o start foi alcançado)
    if (floodMap[start.x][start.y] == -1) {
        // Não há caminho do start ao end
        return Path();
    }
    
    // Reconstruir o caminho seguindo o gradiente descendente
    Path path;
    Position current = start;
    path.push_back(current);
    
    while (!(current == end)) {
        std::vector<Position> neighbors = getNeighbors(maze, current);
        
        // Encontrar o vizinho com menor distância
        int minDist = floodMap[current.x][current.y];
        Position nextPos = current;
        
        for (const Position& neighbor : neighbors) {
            int neighborDist = floodMap[neighbor.x][neighbor.y];
            if (neighborDist >= 0 && neighborDist < minDist) {
                minDist = neighborDist;
                nextPos = neighbor;
            }
        }
        
        // Se não encontrou vizinho melhor, caminho inválido
        if (nextPos == current) {
            return Path();
        }
        
        current = nextPos;
        path.push_back(current);
        
        // Segurança contra loops infinitos
        if (path.size() > maze.height * maze.width) {
            return Path();
        }
    }
    
    return path;
}

std::vector<std::vector<int>> floodFill(const Maze &maze, const Position &target) {
    // Inicializar matriz de distâncias com -1 (não visitado)
    std::vector<std::vector<int>> floodMap(maze.height, std::vector<int>(maze.width, -1));
    
    // Verificar se o target é válido
    if (!Pathfinding::isValidPosition(maze, target)) {
        return floodMap;
    }
    
    std::queue<Position> queue;
    queue.push(target);
    floodMap[target.x][target.y] = 0;
    
    while (!queue.empty()) {
        Position current = queue.front();
        queue.pop();
        
        int currentDist = floodMap[current.x][current.y];
        
        // Obter vizinhos válidos
        std::vector<Position> neighbors = Pathfinding::getNeighbors(maze, current);
        
        for (const Position& neighbor : neighbors) {
            // Se ainda não foi visitado
            if (floodMap[neighbor.x][neighbor.y] == -1) {
                floodMap[neighbor.x][neighbor.y] = currentDist + 1;
                queue.push(neighbor);
            }
        }
    }
    
    return floodMap;
}
