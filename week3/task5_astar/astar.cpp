#include "astar.hpp"
#include <algorithm>
#include <array>
#include <cstdlib>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace navigation {
namespace {
int manhattan(Cell a, Cell b) {
    return std::abs(a.row - b.row) + std::abs(a.col - b.col);
}
void validate(const Grid& grid, Cell start, Cell goal) {
    if (grid.empty() || grid.front().empty())
        throw std::invalid_argument("Grid cannot be empty.");
    const auto rows = grid.size(), cols = grid.front().size();
    // Protect signed cost/index arithmetic before converting sizes to int.
    const auto max_cells = static_cast<std::size_t>(std::numeric_limits<int>::max() / 4);
    if (rows > max_cells || cols > max_cells || rows > max_cells / cols)
        throw std::invalid_argument("Grid is too large for this implementation.");
    for (const auto& row : grid) {
        if (row.size() != cols) throw std::invalid_argument("Grid must be rectangular.");
        for (int value : row)
            if (value != 0 && value != 1)
                throw std::invalid_argument("Only 0 (free) and 1 (obstacle) are allowed.");
    }
    for (Cell point : {start, goal}) {
        if (point.row < 0 || point.col < 0 || point.row >= static_cast<int>(rows)
            || point.col >= static_cast<int>(cols))
            throw std::invalid_argument("Start/goal is outside the grid.");
        if (grid[point.row][point.col] != 0)
            throw std::invalid_argument("Start/goal is an obstacle.");
    }
}
struct Entry { int f, g; Cell cell; };
struct Later {
    bool operator()(const Entry& a, const Entry& b) const {
        // std::priority_queue is a max-heap by default: reverse for minimum f.
        // Tie-break by smaller g, row, column for deterministic replay.
        return std::tie(a.f, a.g, a.cell.row, a.cell.col)
             > std::tie(b.f, b.g, b.cell.row, b.cell.col);
    }
};
} // namespace

SearchResult astar(const Grid& grid, Cell start, Cell goal) {
    validate(grid, start, goal);
    const int rows = static_cast<int>(grid.size());
    const int cols = static_cast<int>(grid.front().size());
    const int count = rows * cols;
    const auto index = [cols](Cell p) { return p.row * cols + p.col; };
    const int infinity = std::numeric_limits<int>::max() / 2;
    std::vector<int> best_g(count, infinity);
    std::vector<int> parent(count, -1);
    std::vector<bool> closed(count, false);
    std::priority_queue<Entry, std::vector<Entry>, Later> frontier;
    SearchResult result;
    best_g[index(start)] = 0;
    frontier.push({manhattan(start, goal), 0, start});
    constexpr std::array<Cell, 4> directions{{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};

    while (!frontier.empty()) {
        const Entry current = frontier.top();
        frontier.pop();
        const int id = index(current.cell);
        if (closed[id] || current.g != best_g[id]) continue; // obsolete heap entry
        closed[id] = true;
        result.expanded.push_back(current.cell);
        if (current.cell == goal) {
            for (int step = id; step != -1; step = parent[step])
                result.path.push_back({step / cols, step % cols});
            std::reverse(result.path.begin(), result.path.end());
            result.cost = current.g;
            return result;
        }
        for (Cell d : directions) {
            const Cell next{current.cell.row + d.row, current.cell.col + d.col};
            if (next.row < 0 || next.row >= rows || next.col < 0 || next.col >= cols
                || grid[next.row][next.col] == 1) continue;
            const int next_id = index(next);
            const int candidate = current.g + 1;
            if (!closed[next_id] && candidate < best_g[next_id]) {
                best_g[next_id] = candidate;
                parent[next_id] = id;
                frontier.push({candidate + manhattan(next, goal), candidate, next});
            }
        }
    }
    return result; // no route: path empty, cost=-1, expanded still available
}
} // namespace navigation

