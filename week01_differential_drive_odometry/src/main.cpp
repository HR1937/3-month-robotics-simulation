#include <iostream>
#include "kinematics.h"
#define M_PI 3.141592653589793

using namespace std;

int main() {
    Pose p = {0, 0, M_PI / 2};
    double Vr = 2, Vl = 2, L = 0.5, dt = 1;

    cout << "x=0,y=0,theta=90" << endl;
    cout << "V for Vr=" << Vr << ",Vl=" << Vl
         << " is " << compute_v(Vr, Vl) << endl;

    cout << "Omega for Vr=" << Vr << ",Vl=" << Vl
         << " and L=" << L
         << " is " << compute_omega(Vr, Vl, L) << endl;

    update_pose(p, Vr, Vl, L, dt);

    cout << "Updated Position is x=" << p.x
         << "  y=" << p.y
         << "   theta=" << p.theta / M_PI * 180
         << " in degrees" << endl;
    return 0;
}
