#ifndef TRAPINT_H
#define TRAPINT_H
#include <vector>

class TrapInt
{
    double _start, _tlast;
    std::vector<double> _sum, _vlast;
public:
    TrapInt();
    void add(double t, const std::vector<double> & x);
    void end(double te, double t, const std::vector<double> & x);
    std::vector<double> endInt(double te, double t, const std::vector<double> & x);
    std::vector<double> endMed(double te, double t, const std::vector<double> & x);
    void begin(double t);
};

#endif // TRAPINT_H
