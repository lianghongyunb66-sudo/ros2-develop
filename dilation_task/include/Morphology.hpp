#pragma once

#include <vector>

using Matrix = std::vector<std::vector<int>>;

class Morphology
{
public:
    Matrix dilate5x5(const Matrix& image) const;
};
