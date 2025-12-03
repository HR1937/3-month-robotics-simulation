#pragma once //pragmatic directive=>“Compiler, include this header only once.”
//->not required by the language standard, but a useful instruction to compiler
struct Pose{
    double x,y; //in meters
    double theta; // in radians
};
double compute_v(double Vr,double Vl); // forward linear velocity where Vl=velocity of left wheel and similarly for Vr

double compute_omega(double Vr,double Vl,double L); // Angular Velocity where L=length between both wheels

void update_pose(Pose &p,double Vr,double Vl,double L,double dt);
