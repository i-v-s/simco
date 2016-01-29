#include "copter.h"

Copter::Copter()
{

}

void Copter::f(PoseRef &d, const PoseRef &X)
{
    const itg::real & wx = w[0], & wy = w[1], & wz = w[2];
    const itg::Ref & Q = X.Q;

    d.Q[0] =              wz * Q[1] - wy * Q[2] + wx * Q[3];
    d.Q[1] = -wz * Q[0]             + wx * Q[2] + wy * Q[3];
    d.Q[2] =  wy * Q[0] - wx * Q[1]             + wz * Q[3];
    d.Q[3] = -wx * Q[0] - wy * Q[1] - wz * Q[2]            ;
}

void Copter::getInitialState(PoseRef &x)
{
    x.P[0] = 0;
}
