#include "Morphology.hpp"

Matrix Morphology::dilate5x5(const Matrix& image) const
{
    if (image.empty() || image[0].empty())
    {
        return {};
    }

    const int rows = static_cast<int>(image.size());
    const int cols = static_cast<int>(image[0].size());

    Matrix result(rows, std::vector<int>(cols, 0));

    constexpr int radius = 2;

    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            if (image[row][col] != 1)
            {
                continue;
            }

            for (int dr = -radius; dr <= radius; ++dr)
            {
                for (int dc = -radius; dc <= radius; ++dc)
                {
                    const int newRow = row + dr;
                    const int newCol = col + dc;

                    if (newRow >= 0 && newRow < rows &&
                        newCol >= 0 && newCol < cols)
                    {
                        result[newRow][newCol] = 1;
                    }
                }
            }
        }
    }

    return result;
}
