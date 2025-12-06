#pragma once // pragmatic directive=>“Compiler, include this header only once.”
//->not required by the language standard, but a useful instruction to compiler
struct Pose
{
    double x, y;  // in meters
    double theta; // in radians
};
double compute_v(double Vr, double Vl); // forward linear velocity where Vl=velocity of left wheel and similarly for Vr

double compute_omega(double Vr, double Vl, double L); // Angular Velocity where L=length between both wheels

void update_pose(Pose &p, double Vr, double Vl, double L, double dt); // Updated so that we know the actual movement (position) to again estimate for the next Vl and Vr to reach to the final position

double ticksL_calc_assuming_Vl(double Vl, double dt, double radius, int N); // Need to assume because we do not have real encoder to directly let us know about the ticks

double ticksR_calc_assuming_Vr(double Vr, double dt, double radius, int N); // considering ticks to be floating point since in simulation it is possible (for real life, small movements get rounded, causing noise)

double Vl_calc_from_ticks(double ticksL, int N, double radius, double dt); // Real-Encoder use a sensor (optical or magnetic) to count wheel rotation (the ticksL and ticksR)

double Vr_calc_from_ticks(double ticksR, int N, double radius, double dt); // Now we can pass this to compute V and Omega
