// C++17 + OpenCV 4. Save PNGs, a search replay AVI and PNG replay frames.
// No cv::imshow(): suitable for a headless development container.
#include "astar.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using navigation::Cell;
using navigation::Grid;
using navigation::SearchResult;

struct Scenario {
    std::string name;
    Grid grid;
    Cell start;
    Cell goal;
};

std::vector<Scenario> scenarios() {
    Grid grid(12, std::vector<int>(16, 0));
    for (int row = 0; row < 11; ++row) grid[row][5] = 1;
    grid[9][5] = 0;
    for (int row = 1; row < 12; ++row) grid[row][10] = 1;
    grid[2][10] = 0;
    Grid blocked = grid;
    for (auto& row : blocked) row[7] = 1;
    return {{"success", grid, {1, 1}, {10, 14}},
            {"no_path", blocked, {1, 1}, {10, 14}},
            {"same_point", grid, {1, 1}, {1, 1}}};
}

void text(cv::Mat& canvas, const std::string& value, cv::Point where, double size = 0.55) {
    cv::putText(canvas, value, where, cv::FONT_HERSHEY_SIMPLEX, size,
                cv::Scalar(25, 25, 25), 1, cv::LINE_AA);
}

cv::Mat render(const Scenario& s, const SearchResult& result, std::size_t count) {
    constexpr int cell = 36, left = 50, top = 105;
    const int rows = static_cast<int>(s.grid.size());
    const int cols = static_cast<int>(s.grid.front().size());
    const int width = left + cols * cell + 300;
    const int height = top + rows * cell + 65;
    cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    const auto centre = [](Cell p) {
        return cv::Point(left + p.col * cell + cell / 2, top + p.row * cell + cell / 2);
    };
    text(canvas, "GRID A* | " + s.name, {left, 35}, 0.9);
    text(canvas, "C++17 / OpenCV | 4-neighbour moves | cost per step = 1", {left, 65});
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const cv::Rect box(left + col * cell, top + row * cell, cell, cell);
            if (s.grid[row][col] == 1)
                cv::rectangle(canvas, box, cv::Scalar(65, 65, 65), cv::FILLED);
            cv::rectangle(canvas, box, cv::Scalar(205, 205, 205), 1);
        }
        text(canvas, std::to_string(row), {16, top + row * cell + 23}, 0.45);
    }
    for (int col = 0; col < cols; ++col)
        text(canvas, std::to_string(col), {left + col * cell + 9, top - 10}, 0.45);
    count = std::min(count, result.expanded.size());
    for (std::size_t i = 0; i < count; ++i)
        cv::circle(canvas, centre(result.expanded[i]), 4, cv::Scalar(125, 125, 125), cv::FILLED);
    const bool done = count == result.expanded.size();
    if (done && result.found()) {
        for (std::size_t i = 1; i < result.path.size(); ++i)
            cv::line(canvas, centre(result.path[i - 1]), centre(result.path[i]),
                     cv::Scalar(15, 15, 15), 3, cv::LINE_AA);
    }
    const auto endpoint = [&](Cell p, const std::string& label) {
        const auto c = centre(p);
        cv::circle(canvas, c, 14, cv::Scalar(255, 255, 255), cv::FILLED);
        cv::circle(canvas, c, 14, cv::Scalar(0, 0, 0), 2);
        text(canvas, label, c + cv::Point(label == "S/G" ? -12 : -5, 5),
             label == "S/G" ? 0.42 : 0.55);
    };
    if (s.start == s.goal) endpoint(s.start, "S/G");
    else { endpoint(s.start, "S"); endpoint(s.goal, "G"); }

    const int x = left + cols * cell + 25;
    text(canvas, "LEGEND", {x, top + 15}, 0.65);
    cv::rectangle(canvas, cv::Rect(x, top + 35, 18, 18), cv::Scalar(65, 65, 65), cv::FILLED);
    text(canvas, "Obstacle", {x + 30, top + 50});
    cv::circle(canvas, {x + 9, top + 77}, 4, cv::Scalar(125, 125, 125), cv::FILLED);
    text(canvas, "Expanded node", {x + 30, top + 82});
    cv::line(canvas, {x, top + 110}, {x + 20, top + 110}, cv::Scalar(15, 15, 15), 3);
    text(canvas, "Final path", {x + 30, top + 115});
    text(canvas, "S: start / G: goal", {x, top + 150});
    text(canvas, "Expanded: " + std::to_string(count), {x, top + 195});
    text(canvas, done ? (result.found() ? "Path cost: " + std::to_string(result.cost)
                                      : "NO PATH") : "Searching...", {x, top + 225});
    text(canvas, "Heuristic: Manhattan", {x, top + 265});
    text(canvas, "Replay is not runtime", {x, top + 300}, 0.48);
    text(canvas, "Coordinates: (row, column).  White cells are free.",
         {left, top + rows * cell + 35}, 0.5);
    return canvas;
}

void save_png(const fs::path& path, const cv::Mat& image) {
    if (!cv::imwrite(path.string(), image))
        throw std::runtime_error("Cannot save " + path.string());
}

void save_cells(const fs::path& path, const std::vector<Cell>& cells) {
    std::ofstream file(path);
    if (!file) throw std::runtime_error("Cannot create " + path.string());
    file << "step,row,column\n";
    for (std::size_t i = 0; i < cells.size(); ++i)
        file << i << ',' << cells[i].row << ',' << cells[i].col << '\n';
    file.flush();
    if (!file) throw std::runtime_error("Write failed: " + path.string());
}

bool replay(const fs::path& out, const Scenario& s, const SearchResult& result, bool frames_only) {
    const fs::path frames = out / "success_frames";
    fs::create_directories(frames);
    std::ofstream manifest(frames / "frames.txt");
    if (!manifest) throw std::runtime_error("Cannot create frame manifest.");
    const auto first = render(s, result, 0);
    cv::VideoWriter writer;
    bool video_ok = false;
    if (!frames_only) {
        try {
            const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            video_ok = writer.open((out / "success.avi").string(), fourcc, 12.0, first.size());
            if (!video_ok)
                video_ok = writer.open((out / "success.avi").string(), cv::CAP_OPENCV_MJPEG,
                                       fourcc, 12.0, first.size());
        } catch (const cv::Exception& e) {
            std::cerr << "Video codec unavailable: " << e.what() << '\n';
        }
        if (!video_ok) std::cerr << "Video unavailable; PNG frames still provide a replay.\n";
    }
    // Each PNG represents exactly one additional expanded node.
    for (std::size_t i = 0; i <= result.expanded.size(); ++i) {
        const cv::Mat frame = render(s, result, i);
        std::ostringstream name;
        name << "frame_" << std::setw(4) << std::setfill('0') << i << ".png";
        save_png(frames / name.str(), frame);
        manifest << name.str() << '\n';
        if (video_ok) {
            try { writer.write(frame); }
            catch (const cv::Exception& e) {
                std::cerr << "Video write failed; use PNG frames: " << e.what() << '\n';
                video_ok = false;
                writer.release();
            }
        }
    }
    if (video_ok) {
        const cv::Mat final_frame = render(s, result, result.expanded.size());
        for (int i = 0; i < 24; ++i) writer.write(final_frame); // hold result for 2 seconds
        writer.release();
    }
    manifest.flush();
    if (!manifest) throw std::runtime_error("Frame manifest write failed.");
    return video_ok;
}

int main(int argc, char** argv) {
    try {
        if (argc > 3 || (argc == 2 && std::string(argv[1]) == "--help")) {
            std::cout << "Usage: ./build/astar_demo [OUTPUT_DIR] [--frames-only]\n";
            return argc > 3 ? 2 : 0;
        }
        const fs::path out = argc >= 2 ? fs::path(argv[1]) : fs::path("images");
        const bool frames_only = argc == 3 && std::string(argv[2]) == "--frames-only";
        if (argc == 3 && !frames_only) throw std::invalid_argument("Unknown option.");
        fs::create_directories(out);
        std::ofstream report(out / "results.txt");
        if (!report) throw std::runtime_error("Cannot create results.txt.");
        report << "Implementation: C++17 / OpenCV " << CV_VERSION << "\n"
                  "A* is implemented in astar.cpp; OpenCV only renders the result.\n"
                  "cost=-1 means unreachable. Playback speed is artificial.\n";
        for (const auto& s : scenarios()) {
            const auto result = navigation::astar(s.grid, s.start, s.goal);
            save_png(out / (s.name + ".png"), render(s, result, result.expanded.size()));
            save_cells(out / (s.name + "_path.csv"), result.path);
            save_cells(out / (s.name + "_expanded.csv"), result.expanded);
            const std::string line = s.name + ": cost=" + std::to_string(result.cost)
                + ", expanded=" + std::to_string(result.expanded.size())
                + ", path_nodes=" + std::to_string(result.path.size());
            std::cout << line << '\n';
            report << line << '\n';
            if (s.name == "success") {
                const bool video_ok = replay(out, s, result, frames_only);
                report << "success_video_written=" << (video_ok ? "true" : "false") << '\n';
            }
        }
        report.flush();
        if (!report) throw std::runtime_error("results.txt write failed.");
        std::cout << "Saved results to: " << fs::absolute(out) << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "A* demo error: " << e.what() << '\n';
        return 1;
    }
}

