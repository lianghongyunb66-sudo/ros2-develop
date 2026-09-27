#pragma once
#include <vector>

namespace navigation {
// Coordinate order is (row, column). 0=free; 1=obstacle.
struct Cell {
    int row = 0;
    int col = 0;
    bool operator==(const Cell& other) const {
        return row == other.row && col == other.col;
    }
    bool operator!=(const Cell& other) const { return !(*this == other); }
};
using Grid = std::vector<std::vector<int>>;

struct SearchResult {
    std::vector<Cell> path;
    std::vector<Cell> expanded; // pop order, including the goal if found
    int cost = -1;             // -1 means unreachable; start==goal has cost 0
    bool found() const { return !path.empty(); }
};

// Four-neighbour, unit-cost A*. Throws std::invalid_argument on malformed input.
SearchResult astar(const Grid& grid, Cell start, Cell goal);
} // namespace navigation
