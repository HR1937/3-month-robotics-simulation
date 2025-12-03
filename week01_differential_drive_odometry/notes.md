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
## 10. Encoder-based odometry summary (real robot intuition)

On a real differential-drive robot, wheel encoders and kinematic equations work together to estimate pose.

Each control cycle:

1. We start from the previous pose estimate:
   - \(x(t), y(t), \theta(t)\).

2. Encoders on each wheel give **tick counts** since the last cycle:
   - `ticks_L`, `ticks_R`.

3. Using known hardware parameters:
   - wheel radius \(R\),
   - encoder ticks per revolution \(N\),
   - time step \(\Delta t\),
   we convert ticks → wheel linear speeds:
   - \(v_L\) and \(v_R\).

4. From wheel speeds, we compute **body velocities** (forward kinematics):
   - \(v = \dfrac{v_R + v_L}{2}\) (forward speed),
   - \(\omega = \dfrac{v_R - v_L}{L}\) (turn rate).

5. From \(v\) and \(\omega\), we update the pose by odometry:
   - \(x_{\text{new}} = x + v \cos(\theta) \cdot \Delta t\),
   - \(y_{\text{new}} = y + v \sin(\theta) \cdot \Delta t\),
   - \(\theta_{\text{new}} = \theta + \omega \cdot \Delta t\).

So the data flow in a real robot is:

- **Encoders → \(v_L, v_R\) → \(v, \omega\) → new \(x,y,\theta\)**

This pose is only an estimate (it can drift due to slip and noise), so in real systems it is often corrected or fused with IMU and SLAM.  
For Week 1, I am focusing only on this pure encoder + kinematics odometry pipeline in simulation.
