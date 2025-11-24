#ifndef TYPES_HPP
#define TYPES_HPP

#include <vector>
#include <string>

using Matrix = std::vector<std::vector<char>>;

struct Maze {
    Matrix matrix;
    int width;
    int height;
};

struct Position {
    int x;
    int y;

    Position() : x(-1), y(-1) {}
    Position(int x_, int y_) : x(x_), y(y_) {}

    bool isValid() const { return x >= 0 && y >= 0; }
    bool operator==(const Position &other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Position &other) const {
        return !(*this == other);
    }
};

using Path = std::vector<Position>;

#endif
