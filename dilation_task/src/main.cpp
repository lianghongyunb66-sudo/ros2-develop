#include <iostream>

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

Matrix createSquareKernel()
{
    return
    {
        {1,1,1,1,1},
        {1,1,1,1,1},
        {1,1,1,1,1},
        {1,1,1,1,1},
        {1,1,1,1,1}
    };
}

Matrix createRoundedKernel()
{
    return
    {
        {0,0,1,0,0},
        {0,1,1,1,0},
        {1,1,1,1,1},
        {0,1,1,1,0},
        {0,0,1,0,0}
    };
}

bool testSinglePixel()
{
    Morphology morphology(createSquareKernel());

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

    Matrix result = morphology.dilate(input);

    return matricesEqual(result, expected);
}

bool testBoundaryPixel()
{
    Morphology morphology(createSquareKernel());

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

    Matrix result = morphology.dilate(input);

    return matricesEqual(result, expected);
}

bool testEmptyForeground()
{
    Morphology morphology(createSquareKernel());

    Matrix input =
    {
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0}
    };

    Matrix expected = input;

    Matrix result = morphology.dilate(input);

    return matricesEqual(result, expected);
}

bool testDifferentKernels()
{
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

    Morphology morphology(createSquareKernel());

    Matrix squareResult = morphology.dilate(input);

    morphology.setKernel(createRoundedKernel());

    Matrix roundedResult = morphology.dilate(input);

    return squareResult != roundedResult;
}

int main()
{
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

    Matrix squareKernel = createSquareKernel();
    Matrix roundedKernel = createRoundedKernel();

    Morphology morphology(squareKernel);

    std::cout << "Original image:\n";
    printMatrix(image);

    Matrix dilation1 = morphology.dilate(image);

    std::cout << "\nDilation 1 - square kernel:\n";
    printMatrix(dilation1);

    morphology.setKernel(roundedKernel);

    Matrix dilation2 = morphology.dilate(image);

    std::cout << "\nDilation 2 - rounded kernel:\n";
    printMatrix(dilation2);

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

    std::cout
        << "Test 4 - different kernels: "
        << (testDifferentKernels() ? "PASS" : "FAIL")
        << '\n';

    return 0;
}
