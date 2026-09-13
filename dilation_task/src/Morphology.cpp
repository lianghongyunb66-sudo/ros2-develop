#include "Morphology.hpp"

Morphology::Morphology(const Matrix& kernel)
    : kernel_(kernel)
{
}

void Morphology::setKernel(const Matrix& kernel)
{
    kernel_ = kernel;
}

Matrix Morphology::dilate(const Matrix& image) const
{
    if (image.empty() || image[0].empty())
    {
        return {};
    }

    const int rows = static_cast<int>(image.size());
    const int cols = static_cast<int>(image[0].size());

    const int kernelRows = static_cast<int>(kernel_.size());
    const int kernelCols = static_cast<int>(kernel_[0].size());

    const int centerRow = kernelRows / 2;
    const int centerCol = kernelCols / 2;

    Matrix result(rows, std::vector<int>(cols, 0));

    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            if (image[row][col] != 1)
            {
                continue;
            }

            for (int kr = 0; kr < kernelRows; ++kr)
            {
                for (int kc = 0; kc < kernelCols; ++kc)
                {
                    if (kernel_[kr][kc] == 0)
                    {
                        continue;
                    }

                    const int newRow = row + kr - centerRow;
                    const int newCol = col + kc - centerCol;

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
