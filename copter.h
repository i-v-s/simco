#ifndef COPTER_H
#define COPTER_H
#include "integrator.h"

class PoseRef: public itg::VectorRef
{
public:
    itg::Ref Q, P, V;
    PoseRef(): Q(4), P(3), V(3), itg::VectorRef({&Q, &P, &V}) {};
    virtual ~PoseRef() {};
};

class Copter : public itg::ModelT<PoseRef>
{
public:
    itg::real w[3];
    Copter();
    void f(PoseRef & d, const PoseRef & x);
    void getInitialState(PoseRef & x);
};

#endif // COPTER_H
