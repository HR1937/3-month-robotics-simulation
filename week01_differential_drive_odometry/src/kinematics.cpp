#include "kinematics.h"
#include <cmath>
double compute_v(double Vr,double Vl) // forward linear velocity where Vl=velocity of left wheel and similarly for Vr
{
    return ((Vr+Vl)/2);
}
double compute_omega(double Vr,double Vl,double L) // Angular Velocity where L=length between both wheels
{
    return((Vr-Vl)/L);
}
void update_pose(Pose &p,double Vr,double Vl,double L,double dt)
{
    double v=compute_v(Vr,Vl);
    p.x+=v*std::cos(p.theta)*dt;
    p.y+=v*std::sin(p.theta)*dt;
    p.theta+=compute_omega(Vr,Vl,L)*dt;
}
