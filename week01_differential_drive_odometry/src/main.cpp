#include <iostream>
#include "kinematics.h"
#include <cmath>
#define M_PI 3.141592653589793

using namespace std;

int main()
{
    Pose p = {0, 0, M_PI / 2};
    double radius = 10;
    int N = 50;
    double L = .5;

    double dt = 5;

    double Vr_cmd = 2;
    double Vl_cmd = 2;

    cout << "Initial Pose: (x=" << p.x << ", y=" << p.y << ", theta=" << p.theta * 180 / M_PI << " deg)" << endl;

    // Step 1: Create fake ticks from intended speeds
    double ticksL = ticksL_calc_assuming_Vl(Vl_cmd, dt, radius, N);
    double ticksR = ticksR_calc_assuming_Vr(Vr_cmd, dt, radius, N);
    cout << "Generated Ticks : Left=" << ticksL << ", Right=" << ticksR << endl;

    // Step 2: Ignore Vr_cmd & Vl_cmd — pretend we don't know them anymore
    double Vl_meas = Vl_calc_from_ticks(ticksL, N, radius, dt);
    double Vr_meas = Vr_calc_from_ticks(ticksR, N, radius, dt);
    cout << "Measured Speeds from Ticks : Vl=" << Vl_meas << " m/s, Vr=" << Vr_meas << " m/s" << endl;

    // Step 3: Update pose using v and ω computed inside update_pose()
    update_pose(p, Vr_meas, Vl_meas, L, dt);
    double deg = fmod(p.theta * 180.0 / M_PI, 360.0);
    if (deg < 0)
        deg += 360.0;
    cout << "Updated Pose After " << dt << "s : (x=" << p.x << ", y=" << p.y << ", theta=" << deg << " deg)" << endl;

    // ================= SECOND TEST (ONLY CHANGING SPEEDS & dt) =================

    Vr_cmd = 4.0;
    Vl_cmd = 1.5;
    dt = 3;

    cout << "\n---- NEW MOVEMENT COMMAND ----" << endl;
    cout << "Commanded Speeds: Vl=" << Vl_cmd << " m/s, Vr=" << Vr_cmd << " m/s, dt=" << dt << " s" << endl;

    // 1) Generate ticks (simulation)
    ticksL = ticksL_calc_assuming_Vl(Vl_cmd, dt, radius, N);
    ticksR = ticksR_calc_assuming_Vr(Vr_cmd, dt, radius, N);
    cout << "Generated Ticks : Left=" << ticksL << ", Right=" << ticksR << endl;

    // 2) Calculate measured speeds from ticks
    Vl_meas = Vl_calc_from_ticks(ticksL, N, radius, dt);
    Vr_meas = Vr_calc_from_ticks(ticksR, N, radius, dt);
    cout << "Measured Speeds from Ticks : Vl=" << Vl_meas << " m/s, Vr=" << Vr_meas << " m/s" << endl;

    // 3) Update pose
    update_pose(p, Vr_meas, Vl_meas, L, dt);
    deg = fmod(p.theta * 180.0 / M_PI, 360.0);
    if (deg < 0)
        deg += 360.0;
    cout << "Updated Pose After " << dt << "s : (x=" << p.x
         << ", y=" << p.y
         << ", theta=" << deg << " deg)" << endl;

    return 0;
}

/*
    My code till day-4 (Before implementation of Encoder calcuating Vl and Vr for us)

    Pose p = {0, 0, M_PI / 2};
    double Vr = 2, Vl = 2, L = 0.5, dt = 1;

    cout << "x=0,y=0,theta=90" << endl;
    cout << "V for Vr=" << Vr << ",Vl=" << Vl<< " is " << compute_v(Vr, Vl) << endl;

    cout << "Omega for Vr=" << Vr << ",Vl=" << Vl<< " and L=" << L<< " is " << compute_omega(Vr, Vl, L) << endl;

    update_pose(p, Vr, Vl, L, dt);

    cout << "Updated Position is x=" << p.x<< "  y=" << p.y<< "   theta=" << p.theta / M_PI * 180<< " in degrees" << endl;
*/