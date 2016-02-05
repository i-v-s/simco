#include "linearregression.h"
#include <gtest/gtest.h>

using namespace Eigen;

TEST(LinearRegressionTest, Scalar)
{
    typedef Array<double, 1, 1> Arr;
    typedef XYStruct<double, 1, 1> Str;
    std::vector<Str> vec;

    vec.push_back({(Arr() << 1.0).finished(), (Arr() << 2.0).finished()});
    vec.push_back({(Arr() << 2.0).finished(), (Arr() << 4.0).finished()});

    LinearRegression<double, 1, 1> lr(vec);
    EXPECT_EQ(2.0, lr.b[0]);
    EXPECT_NEAR(0.0, lr.e.sum(), 1E-10);
}

/*TEST(LinearRegressionTest, Scalar_const)
{
    typedef Array<double, 2, 1> Arr;
    typedef Array<double, 1, 1> ArrY;
    typedef XYStruct<double, 2, 1> Str;
    std::vector<Str> vec;

    // t = {{1}, {2}};
    //Eigen::Array<double, 1, 1> f = {1.0};
    vec.push_back({(Arr() << 1.0, 1.0).finished(), (ArrY() << 2.0).finished()});
    vec.push_back({(Arr() << 3.0, 1.0).finished(), (ArrY() << 3.0).finished()});
    vec.push_back({(Arr() << 5.0, 1.0).finished(), (ArrY() << 4.0).finished()});

    LinearRegression<double, 2, 1> lr(vec);
    EXPECT_EQ(0.5, lr.b[0]);
    //EXPECT_EQ(1.5, lr.a[0]);
    EXPECT_NEAR(0.0, lr.e.sum(), 1E-10);
}*/

TEST(LinearRegressionTest, TwoByOne)
{
    typedef Matrix<double, 2, 1> ArrX;
    typedef Matrix<double, 1, 1> ArrY;
    typedef XYStruct<double, 2, 1> Str;
    std::vector<Str> vec;

    // t = {{1}, {2}};
    //Eigen::Array<double, 1, 1> f = {1.0};
    vec.push_back({(ArrX() << 0, 0).finished(), (ArrY() << 0).finished()});
    vec.push_back({(ArrX() << 2, 1).finished(), (ArrY() << 1).finished()});
    vec.push_back({(ArrX() << 1, 2).finished(), (ArrY() << 1).finished()});

    LinearRegression<double, 2, 1> lr(vec);
    EXPECT_NEAR(1.0 / 3, lr.b[0], 1E-10);
    EXPECT_NEAR(1.0 / 3, lr.b[1], 1E-10);
    EXPECT_NEAR(0.0, lr.e.sum(), 1E-10);


}

/*TEST(LinearRegressionTest, ThreeByTwo)
{
    typedef Matrix<double, 3, 1> ArrX;
    typedef Matrix<double, 2, 1> ArrY;
    typedef XYStruct<double, 3, 2> Str;
    std::vector<Str> vec;

    // t = {{1}, {2}};
    //Eigen::Array<double, 1, 1> f = {1.0};
    vec.push_back({(ArrX() << 1.0, 3, 1).finished(), (ArrY() << 2.0, 2).finished()});
    vec.push_back({(ArrX() << 3.0, 4, 2).finished(), (ArrY() << 4.0, 5).finished()});
    //vec.push_back({(ArrX() << -4.0, 6, 9).finished(), (ArrY() << 4.0, 5).finished()});

    LinearRegression<double, 3, 2> lr(vec);
    EXPECT_NEAR(0.0, lr.e.sum(), 1E-10);


}*/
