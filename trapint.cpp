#include "trapint.h"
#include <assert.h>
#include <gtest/gtest.h>
#include <math.h>

TrapInt::TrapInt() :_start(NAN), _tlast(NAN)
{

}

void TrapInt::add(double t, const std::vector<double> &x)
{
    if(!isnan(_start))
    {
        auto px = x.begin();
        auto ps = _sum.begin(), pl = _vlast.begin();
        if(_tlast > _start)
        {
            double dt05 = (t - _tlast) * 0.5;
            for( ; px != x.end(); px++, ps++, pl++)
                *ps += dt05 * (*px + *pl);
        }
        else
        {
            double dt = t - _start, k = dt / (t - _tlast);
            double kl = dt * 0.5 * k;
            double kx = dt - kl;
            for( ; px != x.end(); px++, ps++, pl++)
                *ps = *px * kx + *pl * kl;
        }
    }
    _vlast = x;
    _tlast = t;
}

void TrapInt::end(double te, double t, const std::vector<double> &x)
{
    assert(t >= te);
    auto px = x.begin();
    auto ps = _sum.begin(), pl = _vlast.begin();
    double dt = te - _tlast, k = dt / (t - _tlast);
    double kx = dt * 0.5 * k;
    double kl = dt - kx;
    for( ; px != x.end(); px++, ps++, pl++)
        *ps += *px * kx + *pl * kl;
}

std::vector<double> TrapInt::endInt(double te, double t, const std::vector<double> &x)
{
    end(te, t, x);
    _start = NAN;
    return _sum;
}

std::vector<double> TrapInt::endMed(double te, double t, const std::vector<double> &x)
{
    end(te, t, x);
    double k = 1.0 / (te - _start);
    for(auto i = _sum.begin(); i != _sum.end(); i++)
        *i *= k;
    _start = NAN;
    return _sum;
}

void TrapInt::begin(double t)
{
    assert(!_vlast.empty() && isnan(_start));
    assert(t >= _tlast);
    _sum.assign(_vlast.size(), 0.0);
    _start = t;
}

TEST(TrapIntTest, Const_mean)
{
    TrapInt ti;
    ti.add(0, {5});
    ti.begin(0);
    EXPECT_EQ(ti.endMed(3, 3, {5})[0], 5);
}

TEST(TrapIntTest, Ajacent_mean)
{
    TrapInt ti;
    ti.add(0.0, {0});
    ti.begin(0.5);
    ti.add(1.0, {1.0}); // 0.375
    EXPECT_EQ(0.8125, ti.endMed(1.5, 2.0, {0.5})[0]);

    ti.begin(1.5);
    ti.add(2.0, {0.5});

    EXPECT_EQ(2.3125 / (4 - 1.5), ti.endMed(4, 4, {1.5})[0]);
}
