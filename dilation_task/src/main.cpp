#include <iostream>
#include <string>

#include "Morphology.hpp"

void printMatrix(const Matrix& matrix)
{
    for (const auto& row : matrix)
    {
        for (int value : row)
        {
            std::cout << value << ' ';
        }

        std::cout << '\n';
    }
}
bool matricesEqual(const Matrix& a, const Matrix& b)
{
    return a == b;
}
bool testSinglePixel()
{
    Morphology morphology;

    Matrix input =
    {
        {0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0},
        {0,0,0,1,0,0,0},
        {0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0}
    };

    Matrix expected =
    {
        {0,0,0,0,0,0,0},
        {0,1,1,1,1,1,0},
        {0,1,1,1,1,1,0},
        {0,1,1,1,1,1,0},
        {0,1,1,1,1,1,0},
        {0,1,1,1,1,1,0},
        {0,0,0,0,0,0,0}
    };

    Matrix result = morphology.dilate5x5(input);

    return matricesEqual(result, expected);
}
bool testBoundaryPixel()
{
    Morphology morphology;

    Matrix input =
    {
        {1,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0}
    };

    Matrix expected =
    {
        {1,1,1,0,0},
        {1,1,1,0,0},
        {1,1,1,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0}
    };

    Matrix result = morphology.dilate5x5(input);

    return matricesEqual(result, expected);
}
bool testEmptyForeground()
{
    Morphology morphology;

    Matrix input =
    {
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0}
    };

    Matrix expected = input;

    Matrix result = morphology.dilate5x5(input);

    return matricesEqual(result, expected);
}
int main()
{
    Morphology morphology;

    Matrix image =
    {
        {0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0},
        {0,0,0,1,1,1,0,0,0},
        {0,0,0,1,1,1,0,0,0},
        {0,0,0,1,1,1,0,0,0},
        {0,0,0,1,1,1,0,0,0},
        {0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0}
    };

    std::cout << "Original image:\n";
    printMatrix(image);

    Matrix result = morphology.dilate5x5(image);

    std::cout << "\nDilated image with 5x5 structuring element:\n";
    printMatrix(result);

    std::cout << "\nRunning tests:\n";

    std::cout
        << "Test 1 - single centre pixel: "
        << (testSinglePixel() ? "PASS" : "FAIL")
        << '\n';

    std::cout
        << "Test 2 - boundary pixel: "
        << (testBoundaryPixel() ? "PASS" : "FAIL")
        << '\n';

    std::cout
        << "Test 3 - empty foreground: "
        << (testEmptyForeground() ? "PASS" : "FAIL")
        << '\n';

    return 0;
}
