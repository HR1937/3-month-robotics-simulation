#include <iostream>
#include "kinematics.h"
#include <cmath>
#define M_PI 3.141592653589793

using namespace std;
double findDegree(double theta);
void printPose(Pose p, double dt, double deg,int steps);

int main()
{
    Pose p = {0, 0, M_PI / 2};
    double radius = 10;
    int N = 50;
    double L = .5;
    double dt = 1;
    double Vr_cmd = 2;
    double Vl_cmd = 2;

    // -------------------------------
    // Test 1: Straight line (Vr = Vl)
    // -------------------------------
    cout << "==== Test 1: Straight Line (Vr = Vl) ====" << endl;
    cout << "Initial Pose: (x=" << p.x << ", y=" << p.y << ", theta=" << p.theta * 180 / M_PI << " deg)\n" << endl;

    for (int steps = 0; steps < 5; steps++)
    {
        double ticksL = ticksL_calc_assuming_Vl(Vl_cmd, dt, radius, N);
        double ticksR = ticksR_calc_assuming_Vr(Vr_cmd, dt, radius, N);
        cout << "Generated Ticks : Left=" << ticksL << ", Right=" << ticksR << endl;

        double Vl_meas = Vl_calc_from_ticks(ticksL, N, radius, dt);
        double Vr_meas = Vr_calc_from_ticks(ticksR, N, radius, dt);
        cout << "Measured Speeds from Ticks : Vl=" << Vl_meas << " m/s, Vr=" << Vr_meas << " m/s\n" << endl;

        update_pose(p, Vr_meas, Vl_meas, L, dt);
        double deg = findDegree(p.theta);
        printPose(p, dt, deg,steps);
    }

    // -----------------------------------------
    // Test 2: Circular motion (Vr > Vl, left turn)
    // -----------------------------------------
    cout << "\n==== Test 2: Circular Motion (Vr > Vl) ====" << endl;
    p = {0, 0, M_PI / 2};   // reset pose
    Vr_cmd = 2;
    Vl_cmd = 1;             // slower left wheel → turn left

    cout << "Initial Pose: (x=" << p.x << ", y=" << p.y << ", theta=" << p.theta * 180 / M_PI << " deg)\n" << endl;

    for (int steps = 0; steps < 5; steps++)
    {
        double ticksL = ticksL_calc_assuming_Vl(Vl_cmd, dt, radius, N);
        double ticksR = ticksR_calc_assuming_Vr(Vr_cmd, dt, radius, N);
        cout << "Generated Ticks : Left=" << ticksL << ", Right=" << ticksR << endl;

        double Vl_meas = Vl_calc_from_ticks(ticksL, N, radius, dt);
        double Vr_meas = Vr_calc_from_ticks(ticksR, N, radius, dt);
        cout << "Measured Speeds from Ticks : Vl=" << Vl_meas << " m/s, Vr=" << Vr_meas << " m/s\n" << endl;

        update_pose(p, Vr_meas, Vl_meas, L, dt);
        double deg = findDegree(p.theta);
        printPose(p, dt, deg,steps);
    }

    // -----------------------------------------
    // Test 3: Rotation in place (Vr = -Vl)
    // -----------------------------------------
    cout << "\n==== Test 3: Rotation in Place (Vr = -Vl) ====" << endl;
    p = {0, 0, M_PI / 2};   // reset pose
    Vr_cmd = 2;
    Vl_cmd = -2;            // opposite directions → spin in place

    cout << "Initial Pose: (x=" << p.x << ", y=" << p.y << ", theta=" << p.theta * 180 / M_PI << " deg)\n" << endl;

    for (int steps = 0; steps < 5; steps++)
    {
        double ticksL = ticksL_calc_assuming_Vl(Vl_cmd, dt, radius, N);
        double ticksR = ticksR_calc_assuming_Vr(Vr_cmd, dt, radius, N);
        cout << "Generated Ticks : Left=" << ticksL << ", Right=" << ticksR << endl;

        double Vl_meas = Vl_calc_from_ticks(ticksL, N, radius, dt);
        double Vr_meas = Vr_calc_from_ticks(ticksR, N, radius, dt);
        cout << "Measured Speeds from Ticks : Vl=" << Vl_meas << " m/s, Vr=" << Vr_meas << " m/s\n" << endl;

        update_pose(p, Vr_meas, Vl_meas, L, dt);
        double deg = findDegree(p.theta);
        printPose(p, dt, deg,steps);
    }

    return 0;
}

double findDegree(double theta)
{
    double deg = fmod(theta * 180.0 / M_PI, 360.0);
    if (deg < 0)
        deg += 360.0;
    return deg;
}

void printPose(Pose p, double dt, double deg,int steps)
{
    // treat very small values as 0 for readability
    if (fabs(p.x) < 1e-6) p.x = 0;
    if (fabs(p.y) < 1e-6) p.y = 0;
    if (fabs(deg) < 1e-6) deg = 0;

    cout << "Updated Pose After " << (steps+1)*dt << "s : (x=" << p.x << ", y=" << p.y << ", theta=" << deg << " deg)\n" << endl;
}
