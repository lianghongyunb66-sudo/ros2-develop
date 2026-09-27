#include "astar.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using navigation::Cell;
using navigation::Grid;
using navigation::SearchResult;
using navigation::astar;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message); // works in Release builds too
}

// Independent shortest-distance oracle: no calls into the A* implementation.
int bfs_distance(const Grid& grid, Cell start, Cell goal) {
    const int rows = static_cast<int>(grid.size());
    const int cols = static_cast<int>(grid.front().size());
    std::vector<std::vector<int>> distance(rows, std::vector<int>(cols, -1));
    std::queue<Cell> queue;
    queue.push(start);
    distance[start.row][start.col] = 0;
    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = {0, 0, -1, 1};
    while (!queue.empty()) {
        const Cell p = queue.front();
        queue.pop();
        if (p == goal) return distance[p.row][p.col];
        for (int i = 0; i < 4; ++i) {
            const int row = p.row + dr[i], col = p.col + dc[i];
            if (row >= 0 && row < rows && col >= 0 && col < cols
                && grid[row][col] == 0 && distance[row][col] == -1) {
                distance[row][col] = distance[p.row][p.col] + 1;
                queue.push({row, col});
            }
        }
    }
    return -1;
}

void check_path(const Grid& grid, Cell start, Cell goal, const SearchResult& result) {
    require(result.cost == bfs_distance(grid, start, goal), "A* cost differs from BFS.");
    std::set<std::pair<int, int>> expanded;
    for (Cell p : result.expanded) {
        require(p.row >= 0 && p.row < static_cast<int>(grid.size())
            && p.col >= 0 && p.col < static_cast<int>(grid.front().size()), "Expanded out of bounds.");
        require(grid[p.row][p.col] == 0, "Expanded an obstacle.");
        require(expanded.insert({p.row, p.col}).second, "Expanded a node twice.");
    }
    if (!result.found()) {
        require(result.cost == -1, "No route must use cost=-1.");
        return;
    }
    require(result.path.front() == start && result.path.back() == goal, "Wrong path endpoints.");
    require(static_cast<int>(result.path.size()) - 1 == result.cost, "Wrong path length.");
    for (std::size_t i = 0; i < result.path.size(); ++i) {
        const Cell p = result.path[i];
        require(p.row >= 0 && p.row < static_cast<int>(grid.size())
            && p.col >= 0 && p.col < static_cast<int>(grid.front().size()), "Path out of bounds.");
        require(grid[p.row][p.col] == 0, "Path enters an obstacle.");
        if (i != 0) {
            const Cell prev = result.path[i - 1];
            require(std::abs(p.row - prev.row) + std::abs(p.col - prev.col) == 1,
                    "Path contains a jump or diagonal move.");
        }
    }
}

Grid demonstration_grid() {
    Grid grid(12, std::vector<int>(16, 0));
    for (int row = 0; row < 11; ++row) grid[row][5] = 1;
    grid[9][5] = 0;
    for (int row = 1; row < 12; ++row) grid[row][10] = 1;
    grid[2][10] = 0;
    return grid;
}

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"empty_map", [] {
            const Grid grid(4, std::vector<int>(5, 0));
            auto result = astar(grid, {0, 0}, {3, 4});
            require(result.cost == 7, "Expected 7 steps.");
            check_path(grid, {0, 0}, {3, 4}, result);
        }},
        {"same_point", [] {
            auto result = astar({{0}}, {0, 0}, {0, 0});
            require(result.cost == 0 && result.path.size() == 1, "Expected zero-cost singleton.");
        }},
        {"unreachable", [] {
            const Grid grid{{0, 1, 0}};
            auto result = astar(grid, {0, 0}, {0, 2});
            require(!result.found(), "No path should exist.");
            check_path(grid, {0, 0}, {0, 2}, result);
        }},
        {"no_diagonal_shortcut", [] {
            const Grid grid{{0, 1}, {1, 0}};
            require(!astar(grid, {0, 0}, {1, 1}).found(), "Diagonal moves are forbidden.");
        }},
        {"single_row", [] {
            const Grid grid(1, std::vector<int>(9, 0));
            auto result = astar(grid, {0, 8}, {0, 0});
            require(result.cost == 8, "Expected 8 steps.");
            check_path(grid, {0, 8}, {0, 0}, result);
        }},
        {"detour", [] {
            const Grid grid{{0, 1, 0}, {0, 1, 0}, {0, 0, 0}};
            auto result = astar(grid, {0, 0}, {0, 2});
            require(result.cost == 6, "Expected a 6-step detour.");
            check_path(grid, {0, 0}, {0, 2}, result);
        }},
        {"invalid_input", [] {
            const std::vector<std::tuple<Grid, Cell, Cell>> cases{
                {Grid{}, {0, 0}, {0, 0}}, {Grid{{}}, {0, 0}, {0, 0}},
                {Grid{{0}, {0, 0}}, {0, 0}, {1, 0}},
                {Grid{{0, 2}}, {0, 0}, {0, 1}},
                {Grid{{1, 0}}, {0, 0}, {0, 1}},
                {Grid{{0, 1}}, {0, 0}, {0, 1}},
                {Grid{{0}}, {-1, 0}, {0, 0}},
                {Grid{{0}}, {0, 0}, {0, 1}}
            };
            for (const auto& [grid, start, goal] : cases) {
                bool rejected = false;
                try { (void)astar(grid, start, goal); }
                catch (const std::invalid_argument&) { rejected = true; }
                require(rejected, "Malformed input was accepted.");
            }
        }},
        {"fixed_demo_cost_36", [] {
            const Grid grid = demonstration_grid();
            auto result = astar(grid, {1, 1}, {10, 14});
            require(result.cost == 36, "Demo map should take 36 steps.");
            check_path(grid, {1, 1}, {10, 14}, result);
        }},
        {"deterministic_replay", [] {
            const Grid grid = demonstration_grid();
            const auto a = astar(grid, {1, 1}, {10, 14});
            const auto b = astar(grid, {1, 1}, {10, 14});
            require(a.path == b.path && a.expanded == b.expanded, "Replay is not deterministic.");
        }},
        {"200_random_maps_against_bfs", [] {
            std::mt19937 rng(2027);
            for (int trial = 0; trial < 200; ++trial) {
                Grid grid(8, std::vector<int>(10, 0));
                for (auto& row : grid)
                    for (int& value : row) value = (rng() % 100 < 28) ? 1 : 0;
                grid[0][0] = grid[7][9] = 0;
                check_path(grid, {0, 0}, {7, 9}, astar(grid, {0, 0}, {7, 9}));
            }
        }}
    };
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "[PASS] " << name << '\n';
        } catch (const std::exception& e) {
            ++failures;
            std::cerr << "[FAIL] " << name << ": " << e.what() << '\n';
        }
    }
    std::cout << (failures == 0 ? "PASS: " : "FAIL: ") << tests.size() - failures
              << "/" << tests.size() << " test groups; random group includes 200 BFS comparisons.\n";
    return failures == 0 ? 0 : 1;
}
