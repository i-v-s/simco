#ifndef LINEARREGRESSION_H
#define LINEARREGRESSION_H
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

template <class T, int M, int N>
struct XYStruct
{
    Eigen::Matrix<T, M, 1> x;
    Eigen::Matrix<T, N, 1> y;
};

template<class T, int M, int N>
class LinearRegression
{
public:
    Eigen::Matrix<T, N, M> b;
    Eigen::Matrix<T, N, 1> e;
    LinearRegression(const std::vector<XYStruct<T, M, N> > & xy)
    {
        using namespace Eigen;
        Matrix<T, M, 1> sx;
        Matrix<T, M, M> sx2;
        Matrix<T, N, 1> sy;
        Matrix<T, N, M> syx;
        for(auto s : xy)
        {
            sx += s.x;
            sx2 += s.x * s.x.transpose();
            sy += s.y;
            syx += Matrix<T, N, 1>(s.y) * Matrix<T, M, 1>(s.x).transpose();
        }
        syx *= xy.size();
        syx -= Matrix<T, N, 1>(sy) * Matrix<T, M, 1>(sx).transpose();
        sx2 *= xy.size();
        sx2 -= sx * sx.transpose();
        b = syx * sx2.inverse();
        //b = (sx2.transpose() * sx2).pseudoinverse() * (sx2.transpose() * syx);

        for(auto s : xy)
        {
            Array<T, N, 1> err = b * s.x - s.y;
            e += Matrix<T, N, 1> (err * err);
        }
    }
};



/*class LinReg
{
    itg::real _sx, _sy, _sxy, _sx2;
public:
    int _count;
    LinReg(): _sx(0), _sy(0), _sxy(0), _sx2(0), _count(0) {}
    void add(itg::real x, itg::real y)
    {
        _count++;
        _sx += x;
        _sy += y;
        _sxy += x * y;
        _sx2 += x * x;
    }
    itg::real _a, _b;
    void calc()
    {
        itg::real mx = _sx / _count,
                  my = _sy / _count,
                  mxy = _sxy / _count,
                  mx2 = _sx2 / _count;
        _b = (mxy - mx * my) / (mx2 - mx * mx);
        _a = my - _b * mx;
    }
};*/

#endif // LINEARREGRESSION_H
