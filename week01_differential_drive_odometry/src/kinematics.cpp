
#include "kinematics.h"
#include <cmath>
#define M_PI 3.141592653589793
double compute_v(double Vr, double Vl)
{
    return ((Vr + Vl) / 2);
}
double compute_omega(double Vr, double Vl, double L)
{
    return ((Vr - Vl) / L);
}
void update_pose(Pose &p, double Vr, double Vl, double L, double dt)
{
    double v = compute_v(Vr, Vl);
    p.x += v * std::cos(p.theta) * dt;
    p.y += v * std::sin(p.theta) * dt;
    p.theta += compute_omega(Vr, Vl, L) * dt;
}
double ticksL_calc_assuming_Vl(double Vl, double dt, double radius, int N)
{
    return ((Vl * dt * N) / (2 * M_PI * radius));
}
double ticksR_calc_assuming_Vr(double Vr, double dt, double radius, int N)
{
    return ((Vr * dt * N) / (2 * M_PI * radius));
}
double Vl_calc_from_ticks(double ticksL, int N, double radius, double dt)
{
    return ((2 * M_PI * radius * ticksL) / (N * dt));
}
double Vr_calc_from_ticks(double ticksR, int N, double radius, double dt)
{
    return ((2 * M_PI * radius * ticksR) / (N * dt));
}
