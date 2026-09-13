#pragma once

#include <vector>

using Matrix = std::vector<std::vector<int>>;

class Morphology
{
public:
    explicit Morphology(const Matrix& kernel);

    void setKernel(const Matrix& kernel);

    Matrix dilate(const Matrix& image) const;

private:
    Matrix kernel_;
};
