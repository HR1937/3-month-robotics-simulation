# Week 1 – Differential Drive Kinematics & Odometry (Simulation Only)

## 1. What is a differential drive robot?

- A differential drive robot has:
  - Two independently driven main wheels: left and right.
  - Often one or more passive caster wheels for support.
- The wheels can only roll forward/backward; they cannot slide sideways without slipping.
- The robot moves by:
  - driving both wheels at the same speed → straight,
  - or different speeds → turning.

**Why is it called "differential" drive?**

- The motion of the robot depends on the **difference** between the left and right wheel speeds.
- If the wheel speeds are equal, there is no difference → no rotation.
- If the wheel speeds are different, the robot rotates.
- So the “differential” refers to **differential wheel speeds** determining the robot’s rotation.

---

## 2. Robot pose and geometry

We describe the robot’s pose in a fixed world frame as:

- \( x \) [m] – position along world X-axis  
- \( y \) [m] – position along world Y-axis  
- \( \theta \) [rad] – heading angle of the robot body
  - \( \theta = 0 \): facing along +X direction
  - \( \theta = \pi/2 \): facing along +Y, etc.

Wheel-related quantities:

- \( v_L \) [m/s] – linear speed of the left wheel at the ground  
- \( v_R \) [m/s] – linear speed of the right wheel at the ground  
- \( L \) [m] – distance between the two wheels (wheelbase / track width)

The pose at time \( t \) is written as \( (x(t), y(t), \theta(t)) \).

---

## 3. Body velocities: linear \( v \) and angular \( \omega \)

From the wheel speeds, we define the robot body velocities:

- **Forward linear velocity** (through the center of the axle):
  \[
  v = \frac{v_R + v_L}{2}
  \]

- **Angular velocity** (turning rate around the center):
  \[
  \omega = \frac{v_R - v_L}{L}
  \]

**Intuition:**

- If \( v_R = v_L \):
  - \( v = v_R = v_L \)
  - \( \omega = 0 \)
  - Robot moves straight.

- If \( v_R \neq v_L \) (both constant):
  - \( v \) is constant
  - \( \omega \) is constant and non-zero
  - Robot follows a **circular arc** (a full circle if run long enough).

- The bigger \( |v_R - v_L| \), the bigger \( |\omega| \) → tighter/faster turns.

Summary:
- Average of wheel speeds → forward motion \( v \).  
- Difference of wheel speeds → rotation \( \omega \).

---

## 4. Why it can’t move sideways

- Each wheel strongly resists sideways sliding (pure rolling assumption).
- Because of that, the robot **cannot** move directly sideways in the world frame.
- It is **non-holonomic**:
  - It has constraints on its motion: only forward/backward along its heading, plus rotation.
- To move “at 45°” in the world, it must:
  1. Rotate its whole body until \( \theta \approx 45^\circ \).
  2. Then move forward.

(Omni/mecanum robots can move sideways, but they use different wheel types and different kinematics; here we focus on the simpler, very common differential drive.)

---

## 5. From body velocity \( v \) to world velocities \( V_x \), \( V_y \)

The robot moves **forward** along its heading \( \theta \) with speed \( v \).

In the world coordinate frame, this forward motion has components:

- \( V_x = v \cos(\theta) \)  
- \( V_y = v \sin(\theta) \)

These are the **velocity components** of the robot’s center in the global X and Y directions.

Interpretation:

- If \( \theta = 0 \): \( V_x = v \), \( V_y = 0 \) → pure +X motion.  
- If \( \theta = \pi/2 \): \( V_x = 0 \), \( V_y = v \) → pure +Y motion.  
- Other angles: motion is split between X and Y.

---

## 6. Odometry update (discrete-time integration)

Given current pose \( (x, y, \theta) \) and body velocities \( v \) and \( \omega \), over a small time step \( \Delta t \) we update:

\[
x_{\text{new}} = x_{\text{old}} + v \cos(\theta) \cdot \Delta t
\]

\[
y_{\text{new}} = y_{\text{old}} + v \sin(\theta) \cdot \Delta t
\]

\[
\theta_{\text{new}} = \theta_{\text{old}} + \omega \cdot \Delta t
\]

Explanation:

- Distance moved along the robot’s forward direction in this step:
  \[
  d = v \cdot \Delta t
  \]
- Project this distance onto X and Y using \(\cos(\theta)\) and \(\sin(\theta)\).
- Update the heading by turning rate \(\omega\) for time \(\Delta t\).

This is called **odometry** (dead reckoning): estimating pose by integrating motion over time.  
It’s accurate short-term but drifts long-term due to wheel slip, encoder errors, etc. Later, it can be corrected with IMU/SLAM.

---

## 7. Simple motion patterns

1. **Straight line**
   - \( v_L = v_R \)
   - \( \omega = 0 \)
   - Pose update:
     - \( x \) and \( y \) change according to heading \( \theta \),
     - \( \theta \) stays constant.
   - Path: straight line.

2. **Pure rotation (spin in place)**
   - \( v_L = -v_R \)
   - \( v = 0 \)
   - \( \omega \neq 0 \)
   - Robot spins around its center; ideally \( x, y \) stay constant, \( \theta \) changes.

3. **Circular arc**
   - \( v_L \neq v_R \), both constant.
   - \( v \) constant, \( \omega \) constant \( \neq 0 \).
   - Robot traces an arc of a circle (full circle if run long enough).

If \( v_L(t) \), \( v_R(t) \) change over time, then \( v(t) \), \( \omega(t) \) change, and the path becomes a more general smooth curve.

---

## 8. Relation to sensors (short note)

- **Wheel encoders** measure how much each wheel has rotated.
  - From encoder ticks, we compute approximate \( v_L, v_R \) and then use the kinematic equations to update \( x, y, \theta \).
  - This alone is theoretical and drifts if there is slip.

- **IMU** (Inertial Measurement Unit) provides:
  - Gyroscope: angular velocity (how fast the robot is turning).
  - Accelerometer: linear acceleration.
  - These can be fused with odometry to get a better estimate of orientation and motion.

- **SLAM** (with LiDAR or camera) can correct long-term drift by matching what the robot sees to a map.

For Week 1, I focus only on the **pure kinematic odometry equations** in simulation.

---

## 9. Code plan for Week 1 (what I implement)


### 9.1 Types
```
struct Pose {
  double x; // position in meters
  double y; // position in meters
  double theta; // heading in radians
};
```

### 9.2 Functions

Compute body velocities from wheel speeds:

```
double compute_v(double v_L, double v_R) {
  return (v_L + v_R) / 2.0;
}

double compute_omega(double v_L, double v_R, double L) {
  return (v_R - v_L) / L;
}
```

Update pose using odometry equations:

```
void update_pose(Pose &p, double v, double omega, double dt) {
  p.x += v * std::cos(p.theta) * dt;
  p.y += v * std::sin(p.theta) * dt;
  p.theta += omega * dt;
}
```

### 9.3 Test ideas for this week

1. **Straight motion test**
   - Parameters:
     - \( L = 0.2 \) m, \( dt = 0.1 \) s.
     - \( v_L = v_R = 0.3 \) m/s.
   - Start:
     ```
     Pose p{0.0, 0.0, 0.0};
     ```
   - Simulate multiple steps with `update_pose` and print \( x, y, \theta \).
   - Expect motion along +X.

2. **Turning test**
   - Parameters:
     - \( v_L = 0.1 \), \( v_R = 0.3 \) m/s (constant).
   - Simulate many steps, observe how \( x, y, \theta \) change.
   - Later:
     - Log (x, y) points to a file.
     - Plot X vs Y to visualize the circular arc (saved as `week1_path_plot.png`).

---
## 10. One-step odometry story (how encoders update pose)

I imagine odometry as a repeated “small step” story.

### 10.1 Start of the step

At some time t, I already have an estimated pose:
- x(t), y(t), theta(t)

This is the pose at the beginning of the step.

### 10.2 During a short time dt

I let the robot move for a short time dt (for example, 0.1 s).

During this exact interval:
- The left wheel rotates some amount.
- The right wheel rotates some amount.
- The encoders count how many ticks happened during this interval:
  - ticks_L for the left wheel,
  - ticks_R for the right wheel.

These tick counts are only for this step, not total since startup.

### 10.3 Ticks → wheel distances in this step

I know hardware parameters:
- wheel radius R,
- encoder ticks per revolution N.

For this step:

- Angle turned by the left wheel:
  delta_phi_L = (ticks_L / N) * 2 * pi

- Distance travelled by the left wheel along the floor in this step:
  d_L = R * delta_phi_L

Similarly for the right wheel:
- delta_phi_R = (ticks_R / N) * 2 * pi
- d_R = R * delta_phi_R

So d_L and d_R are the distances each wheel moved between time t and t + dt, not since the beginning of the run.

### 10.4 Wheel distances → body motion in this step

From these two distances for this step:

- Forward distance of the robot’s center:
  d_center = (d_L + d_R) / 2

- Change in robot heading during this step:
  delta_theta = (d_R - d_L) / L

If I divide by dt, I get approximate velocities for this step:
- v ≈ d_center / dt
- omega ≈ delta_theta / dt

### 10.5 Body motion → new global pose

Using the pose at the start of the step, (x(t), y(t), theta(t)), I update:

- x_new = x(t) + d_center * cos(theta(t))
- y_new = y(t) + d_center * sin(theta(t))
- theta_new = theta(t) + delta_theta

Now the time is t + dt, and the new pose is:
- (x_new, y_new, theta_new)

This becomes the starting pose for the next step.

### 10.6 Summary of the chain each step

For every small time step, the data flow is:

previous pose (x, y, theta)
→ encoder ticks in this step
→ wheel distances in this step (d_L, d_R)
→ body motion in this step (d_center, delta_theta)
→ new pose (x_new, y_new, theta_new)

Repeating this many times gives the odometry estimate over time.


This pose is only an estimate (it can drift due to slip and noise), so in real systems it is often corrected or fused with IMU and SLAM.  
For Week 1, I am focusing only on this pure encoder + kinematics odometry pipeline in simulation.

---
## 11. How control and odometry use v, omega, vL, vR

There are two opposite directions in a differential-drive robot:

### 1) Control side (AI → wheels)

- The planner/controller decides how the robot body should move:
  - desired forward speed v_cmd,
  - desired turn rate omega_cmd.
- Inverse kinematics converts body velocities to wheel speeds:
  - vL_cmd = v_cmd − (omega_cmd * L / 2)
  - vR_cmd = v_cmd + (omega_cmd * L / 2)
- These vL_cmd and vR_cmd are what we try to send to the motors (e.g., using PWM).
- This direction answers: “Given how I want the robot to move (v, omega), what should each wheel do (vL, vR)?”

### 2) Odometry side (wheels → pose)

- Encoders measure what actually happened: ticks_L and ticks_R in the last dt.
- From ticks and robot parameters, we compute actual wheel speeds:
  - vL_meas, vR_meas.
- Forward kinematics converts wheel speeds to body velocities:
  - v = (vR_meas + vL_meas) / 2
  - omega = (vR_meas − vL_meas) / L
- Using v and omega, we update pose:
  - x_new = x_old + v * cos(theta_old) * dt
  - y_new = y_old + v * sin(theta_old) * dt
  - theta_new = theta_old + omega * dt
- This direction answers: “Given what the wheels actually did (vL, vR), how did the robot move (v, omega) and what is the new pose (x, y, theta)?”

In a running robot, these two directions work together in a loop:
- Control uses the current pose estimate to choose v_cmd, omega_cmd.
- Odometry uses encoder feedback to update the pose estimate after each time step.

