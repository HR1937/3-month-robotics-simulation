# Week 1 – Differential Drive Kinematics & Odometry (Simulation Only)

## 1. What is a differential drive robot?

- A differential drive robot has:
  - Two independently driven main wheels: left and right.
  - One passive caster wheel (a small free-rotating support wheel for balance).
    - (Some large/heavy robots may use two, but only for extra weight support, not for movement.)
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

## 2. Robot Position and Geometry

To predict how a robot moves, we must know **where it is now** and **how its wheels are moving**.  
A robot needs:

- **Position (x, y)** – where it is in the world  
- **Orientation (θ)** – which direction it is facing  
- **Wheel speeds (Vl, Vr)** – how fast the wheels are moving  

*Position answers “Where am I?” and wheel speeds answer “How am I moving?”*

### World Frame (fixed reference)

The robot’s position is measured in a **world frame**, which is a fixed X–Y coordinate system in the environment (like a grid drawn on the floor).  
This frame does **not move with the robot**. The robot moves **inside** it.

*Think of a map that stays still while you move on it.*

### Robot Position Variables

- **x (meters)** – position along the world X-axis  
- **y (meters)** – position along the world Y-axis  
- **θ (theta, radians)** – robot’s heading (orientation)  
  - θ = 0     → robot faces the +X direction  
  - θ = π/2   → robot faces the +Y direction  

The robot’s state at time **t** is written as:
```
(x(t), y(t), θ(t))
```

### Wheel Quantities

- **Vl (m/s)** – linear speed of the left wheel  
- **Vr (m/s)** – linear speed of the right wheel  
- **L (meters)** – distance between the wheels (wheelbase / track width)

These wheel speeds determine how the robot moves and turns.


---

## 3. Body Velocities: Linear (v) and Angular (ω)

To update the robot’s position, we must know **how fast the robot moves forward** and **how fast it rotates**.  
These two values are computed from the wheel speeds **Vl** and **Vr** *(measured using wheel encoders)*.

- **Linear velocity (v)** – how fast the robot moves forward  
- **Angular velocity (ω)** – how fast the robot turns

### Formulas

- **Forward linear velocity**
  
  It is the average of the two wheel speeds:
```
v = (Vr + Vl) / 2
```

- **Angular velocity (turning rate)**

It depends on the difference between wheel speeds:
```
ω = (Vr - Vl) / L
```
*(L is the distance between the two wheels.)*

### Intuition

- If **Vr = Vl**  
→ robot moves straight (no rotation)

- If **Vr ≠ Vl**  
→ robot rotates and follows a curved path

- Larger difference **(Vr − Vl)**  
→ robot turns faster and in a tighter curve

### Summary

- **Average of wheel speeds → forward motion (v)**  
- **Difference of wheel speeds → rotation (ω)**  

---

### What is a wheel encoder?

A **wheel encoder** is a sensor connected to each wheel that **counts how much the wheel has rotated**.

- It does **not directly measure speed**
- Instead, it counts **tiny steps** of rotation called **ticks**
- From ticks, we calculate:
- how far the wheel has moved
- how fast it is spinning → gives **Vl** and **Vr**

**Why the name "encoder"?**  
Because it **encodes rotation into numbers** (tick counts) that a computer can understand.

**Very simple example**  
- If the wheel rotates a little → encoder count increases  
- More counts in less time → wheel is moving faster

Encoders are the reason the robot knows:
> “How much did each wheel actually move?”

Which is why they are essential for odometry (position estimation).

---

## 4. Why a Differential Drive Robot Cannot Move Sideways

The wheels of a differential drive robot are designed to **roll forward/backward**, not slide sideways.  
So it can only:
- move straight  
- rotate (turn)

This limitation is called **non-holonomic** — the robot **cannot move freely in all directions**.

### How does it move diagonally?
It cannot slide diagonally. Instead, it:
1. rotates to the desired angle  
2. then moves forward

### What if we used different wheels?
If we use **omni wheels** or **mecanum wheels**, the robot *can* move sideways.  
But then the **kinematics equations change completely**, because those wheels allow motion in multiple directions.

### Which type is used more in real life?
- **Differential drive (normal wheels)** → most common (low cost, simple control). Used in: home robots, warehouse robots, small research robots.
- **Mecanum/omni wheels** → used where sideways motion is needed (expensive, complex). Used in: industry robots that need tight-space maneuvering.

---

## 5. From Body Velocity to World Velocities (Vx, Vy)

The robot moves forward with speed v, but the world uses a fixed X–Y grid. So we convert the robot's current forward speed into current world velocities:

* **Vx** → velocity along the world X-axis (right now)
* **Vy** → velocity along the world Y-axis (right now)

Based on the robot's current heading θ:
```
Vx = v * cos(θ)
Vy = v * sin(θ)
```

These do not give the new position. They only tell how the robot is moving at this moment, which we then use to update its position.

## 6. Odometry Update (Position Over Time)

Using the current velocities (Vx, Vy) and rotation rate (ω), we update the robot's position after a small time step Δt:
```
x_new = x_old + Vx * Δt
y_new = y_old + Vy * Δt
θ_new = θ_old + ω  * Δt
```

This step-by-step update is called **odometry**. It estimates the robot's path over time, though small wheel/encoder errors cause slow drift (fixed later with IMU/SLAM).

## 7. Common Motion Cases (Quick Reference)

> Vr, Vl = right and left wheel speeds


| Wheel Speeds (Vr, Vl) | Motion Type | Result |
|------------------------|-------------|---------|
| Vr = Vl | Straight | Moves forward, no rotation |
| Vr = -Vl | Spin in place | Rotates around its center |
| Vr ≠ Vl | Curved path | Follows a circular arc |

Almost any complex robot path is built by combining these basic motion types.

---

### IMPORTANT NOTE

> Vx and Vy describe the robot's motion right now.<br>
> Odometry uses them to compute where the robot will be after Δt.


---

## 8. Relation to Sensors (How real robots estimate motion)

Odometry uses only wheel rotation to estimate how the robot moves. To do that, real robots rely on sensors that measure wheel motion and body rotation.

### Wheel Encoders (for Vr and Vl)

Wheel encoders count how much each wheel has rotated. From these rotation counts (ticks), we estimate the wheel speeds:
```
Vr, Vl → computed from encoder ticks
```

We then use our odometry formulas to update:
```
x, y, θ → robot's estimated position and heading
```

**Limitation:** If the wheels slip, the encoder values become wrong, causing drift (small errors that grow over time).

### IMU (Inertial Measurement Unit)

To reduce drift, robots use an IMU:

* **Gyroscope** → measures turning speed (angular velocity)
* **Accelerometer** → measures linear acceleration

This helps correct encoder errors and gives a more stable motion estimate.

### SLAM (Simultaneous Localization and Mapping)

Even with IMU + encoders, errors keep increasing slowly. SLAM fixes this by comparing what the robot sees to a map:

* Builds a map of the environment while locating itself inside that map
* Uses a LiDAR or camera to detect surroundings
* Adjusts position errors based on observed landmarks

### Note for Week 1

In Week 1, we focus only on ideal odometry in simulation, so we assume:

> No wheel slip<br>
> Perfect encoders<br>
> No IMU or SLAM corrections


First we learn the math. Later we add real-world sensors to correct it.

---

## 9. Code Plan (What the program will do)

| Step | What we will build | What we will get |
|------|-------------------|------------------|
| Basic Movement Math | Code that calculates how the robot moves when wheels rotate | New robot position (x, y, direction) |
| Fake Sensor Input | Generate pretend encoder ticks instead of giving speeds directly | Data that looks like real robot sensors |
| Using Ticks to Move | Use only these ticks to update the robot's position | Position calculated from "sensor" data |
| Saving the Path | Store every position the robot reaches | A list of points showing robot's travel path |
| Drawing the Path | Plot the travel path on an XY graph | Image file: `week1_path_plot.png` |

### End Result in One Line

The program will take fake sensor data, estimate the robot's movement, and show the path as a picture.

---
## 10. How Encoders Update the Robot's Pose

Odometry works by repeating the same small update many times. In each small time step Δt, the robot moves a bit, and we update its pose.

### Step 1: Encoders measure wheel rotation

During Δt, each wheel produces tick counts:
```
ticks_L , ticks_R
```

These ticks represent only the movement in this small interval, not from the start of the robot.

### Step 2: Ticks → wheel distances

Knowing:

* wheel radius R
* ticks per revolution N

The wheel travel distances in this step:
```
d_L = R * (ticks_L / N) * 2π
d_R = R * (ticks_R / N) * 2π
```

### Step 3: Wheel distances → robot motion in this step
```
Forward movement:   d_center = (d_L + d_R) / 2
Rotation change:    Δθ = (d_R - d_L) / L
```

### Step 4: Update global position using current heading
```
x_new = x_old + d_center * cos(θ_old)
y_new = y_old + d_center * sin(θ_old)
θ_new = θ_old + Δθ
```

### Final idea

We repeat this step many times:
```
( old pose ) → ticks → ( d_L, d_R ) → ( d_center, Δθ ) → ( new pose )
```

Doing this continuously gives the full path estimate.

For Week 1, we simulate ticks and apply this pipeline without IMU or SLAM corrections.

---
## 11. How Control and Odometry Use v, ω, vL, vR

A differential-drive robot has two opposite directions of calculation:

### (A) Control Side: AI → Wheels

The controller decides how the robot should move:
```
v_cmd     = desired forward speed
ω_cmd     = desired turn rate
```

To make this happen, we convert body motion into wheel speeds (inverse kinematics):
```
vL_cmd = v_cmd - (ω_cmd * L / 2)
vR_cmd = v_cmd + (ω_cmd * L / 2)
```

These values are sent to the motors (via PWM, etc.).

**Question answered here:** "To move like this (v, ω), how fast should each wheel rotate (vL, vR)?"

### (B) Odometry Side: Wheels → Pose

Encoders measure what actually happened and give wheel speeds:
```
vL_meas , vR_meas
```

Using these measured speeds, we compute body motion:
```
v     = (vR_meas + vL_meas) / 2
ω     = (vR_meas - vL_meas) / L
```

Then update position:
```
x_new     = x_old + v * cos(θ_old) * dt
y_new     = y_old + v * sin(θ_old) * dt
θ_new     = θ_old + ω * dt
```

**Question answered here:** "Given what the wheels actually did, how did the robot move and where is it now?"

### Both run together in a loop

> Control uses current pose → chooses v_cmd, ω_cmd<br>
> Odometry uses encoder feedback → updates the pose


This loop runs continuously in real robots.

---

## 12. Headers, Source Files, Linking, `#pragma once`

### `.h` (Header) vs `.cpp` (Source)

* `.h` → declarations only (function signatures, structs)
* `.cpp` → definitions (actual code)
* `main.cpp` includes `.h` to know the function exists

### Compilation vs Linking

> Each .cpp → compiled separately → creates .o (object files)<br>
> Linker joins them → matches function calls with definitions


That's how `update_pose()` in `main.cpp` connects to its real code in `kinematics.cpp`.

### Why not put definitions in `.h`?

If `.h` contains full function bodies and is included in multiple files:

> multiple copies → linker duplicates → error: multiple definition


### What does `#pragma once` do?

* It is a compiler directive.
* It tells the compiler to include a header only one time, even if included many times.
* Prevents duplicate inclusion and errors.

### Final Summary

> Header (.h) tells what exists,<br>
> Source (.cpp) provides the code,<br>
> Linker connects them,<br>
> #pragma once prevents duplicate inclusion.

---

# 13. Simulating Wheel Encoders (Day 5)

## Why we need encoder simulation

Until now, we directly used wheel speeds `Vl` and `Vr` to compute how the robot moves. But real robots never get `Vl` and `Vr` directly. They only get encoder tick counts (rotation steps). So, to act like a real robot, we must:

* assume wheel motion → generate ticks
* forget what we assumed
* recalculate speed from ticks
* use those speeds to update pose

> This makes our simulation behave like a real physical robot with sensors.

## 13.1 What does an encoder measure?

A wheel encoder does not measure speed or angle directly. It only counts tiny rotations:
```
tick tick tick tick…
```

More ticks in less time → wheel moved faster. Fewer ticks → wheel moved slower.

### How does it count ticks?

* Optical sensors (LED + slotted disk)
* Magnetic Hall-effect sensors

> Encoder = a sensor that converts wheel rotation into digital tick counts.


## 13.2 Simulating fake ticks (since we don't have a real robot)

To generate ticks, we assume that the ideal commanded wheel speeds actually happened:
```
ticksL = (Vl * dt / (2πR)) * N
ticksR = (Vr * dt / (2πR)) * N
```

Where:

| Symbol | Meaning |
|--------|---------|
| `Vl, Vr` | commanded wheel speeds |
| `dt` | small time interval |
| `R` | wheel radius |
| `N` | ticks per full revolution |
| `2πR` | wheel circumference |

> We allow fractional ticks in simulation → real robots get integers, but allowing decimals prevents sudden jumps during learning.


## 13.3 Re-computing speed from ticks (like a real robot would)

Now we pretend we do not know `Vl` and `Vr`. We compute them again from ticks:
```
Vl = (2πR * ticksL) / (N * dt)
Vr = (2πR * ticksR) / (N * dt)
```

> This step converts sensor ticks → physical speed.


## 13.4 Updating pose from measured speeds

After recalculating `Vl` and `Vr`, we compute:
```
v = (Vr + Vl) / 2
ω = (Vr - Vl) / L
```

Then integrate:
```
x_new = x_old + v * cos(θ_old) * dt
y_new = y_old + v * sin(θ_old) * dt
θ_new = θ_old + ω * dt
```

> IMPORTANT: We are now using measured motion, not commanded motion.


## 13.5 Why we wrap θ (heading) using floating-point modulo

As the robot rotates many times, the angle `θ` may go far beyond 360°. To keep it meaningful:
```
deg = fmod(theta * 180/π, 360)
if (deg < 0) deg += 360;
```

### Why not `%`?

* `%` works only on integers
* it loses decimals
* fails for negative angles

So, `fmod()` (in `<cmath>`) is used.

## 13.6 Summary of the "realistic odometry loop"

| Step | Action | Source |
|------|--------|--------|
| 1 | Controller gives `Vl_cmd` & `Vr_cmd` | AI / user input |
| 2 | We assume those speeds occurred & generate ticks | Fake encoders (simulation) |
| 3 | We ignore the commanded values | Just like real robots |
| 4 | Compute `Vl_meas` & `Vr_meas` from ticks | Sensor decoding |
| 5 | Compute `v` & `ω` | Forward kinematics |
| 6 | Update `(x, y, θ)` | Odometry |

> This is exactly how real robots estimate their position.


## 13.7 Industry note

Real robots:

* never trust commanded speeds
* only trust sensors (encoders, IMU, SLAM)
* combine them to reduce drift

> Our simulation is now doing the same → Good enough to extend into ROS2 + Gazebo + SLAM later.
---
# 14. Testing Odometry on Virtual Robot Paths

## Why are we doing this today?

Till now, we calculated how a robot moves using wheel data and odometry formulas. But we never checked if the robot actually behaves like a real one when it moves straight, turns, or rotates.

So Day 6 is simply about checking the behaviour:

* If we give certain wheel speeds,
* Does the robot update its pose in a sensible way?

> We are not testing accuracy today.<br>
> We are only checking correct motion shape (straight, circle, rotation).

---

## What is happening in Day-6 code?

We:

1. Give a fixed wheel command (like `Vr=2`, `Vl=2`).
2. We convert those speeds to fake encoder ticks.
3. From ticks, we again recalculate `Vr` and `Vl` (like a real robot).
4. Then we update the pose using those measured values.

Because there is no error or noise yet, the measured speeds are the same every loop, so:

* Ticks remain same every step,
* `Vl` and `Vr` remain same every step.

> Even though speeds are fixed, the robot's position keeps changing because the pose we update each time becomes the "starting point" for the next step.

---

## Understanding the 3 Motions We Tested

### 1. Straight Line

We set:
```
Vr = Vl
```

Meaning both wheels move equally, so the robot cannot rotate. Since our robot is facing 90° initially (towards +Y axis), its:

* x stays 0
* y keeps increasing
* θ stays constant (90°)

So the pose moves in a straight upward line.

---

### 2. Circle / Curved Motion

We set:
```
Vr > Vl
```

The right wheel moves faster than the left wheel, so the robot must turn left. Now at each step:

* x and y both change
* θ keeps increasing
* The robot slowly curves

This is how differential drive robots follow curved paths (one wheel faster creates rotation around a circle).

---

### 3. Rotation in Place

We set:
```
Vr = -Vl
```

The wheels rotate in opposite directions, so the robot spins around its own center. Now:

* x and y do not change
* θ changes a lot
* Robot stays at same place but rotates

This shows differential drive can rotate without moving forward.

---

## Why were we getting tiny weird numbers like `1.2e−14`?

Those tiny values are mistakes from floating-point math, not from our logic. Example: `0.00000000000001` is shown as `1e-14`. They are practically zero, so we replaced anything very small with 0 while printing.

---

## Simple Summary

* Day 6 does not measure accuracy.
* It checks if different wheel speeds create correct movement shapes.
* Since we did not add any real-world errors, ticks and speeds remain the same, and only pose keeps changing.

### One-Line Understanding

> Same speeds → different pose each step because the robot keeps moving forward in time.

# 15. Visualising Robot Path with Python (Day 7)

## 15.1 Why are we doing this?

Till now, we only printed numbers for pose:

* x (position in X)
* y (position in Y)
* θ (heading angle)

Numbers are hard to "see".

For understanding motion, it is much easier if we can:

* see the path of the robot on an XY graph
* see how θ changes during rotation

> Day 7 is about turning odometry output into a picture, not just trusting the console.

---

## 15.2 What we log from C++ (CSV files)

From `main.cpp`, after each pose update, we save:

* x
* y
* theta_deg (angle in degrees)

into `.csv` files:

* `Straight_path.csv`
* `Circular_path.csv`
* `Rotated_path.csv`

Each CSV has a simple format:
```
x,y,theta_deg
0,0,90
0,2,90
...
```

### Why CSV?

* It is just plain text.
* Each row = one time step.
* Columns are separated by commas → easy for Python to read.

We also overwrite files (using truncate mode) so that each C++ run gives fresh data, not mixed with old values.

---

## 15.3 Why we use Python + matplotlib

We use Python only as a tool to plot.

> The main logic stays in C++, Python is for drawing.

### What is a library?

A library is ready-made code written by others that we can reuse instead of writing plotting logic from scratch.

### What is matplotlib?

`matplotlib` is a plotting library for Python.

It can draw:

* line plots
* scatter plots
* bar graphs
* etc.

### What is pyplot and plt?

We write:
```python
import matplotlib.pyplot as plt
```

* `matplotlib` → main library
* `pyplot` → a part of matplotlib that gives simple plotting functions
* `plt` → just a short nickname we use in our code

The dot `.` means "inside":

* `matplotlib.pyplot` → pyplot module inside matplotlib
* `plt.plot(...)` → use the plot function from pyplot

So when we call:
```python
plt.plot(xs, ys)
```

we are saying:

> "Ask the plotting library to draw a curve through these points."

---

## 15.4 Reading CSV and plotting paths

In `graph-plot.py`, we:

### Read CSV files
```python
def read_csv(filename):
    xs = []
    ys = []
    thetas_deg = []
    with open(filename, "r") as f:
        next(f)          # skip header line
        for line in f:
            line = line.strip()
            if not line:
                continue
            x, y, theta = line.split(",")
            xs.append(float(x))
            ys.append(float(y))
            thetas_deg.append(float(theta))
    return xs, ys, thetas_deg
```

* `open(filename, "r")` → open file for reading.
* `next(f)` → skip first line (`x,y,theta_deg`).
* `line.split(",")` → split `"0,2,90"` into `"0"`, `"2"`, `"90"`.
* `float(...)` → convert strings into numbers.

### Get data for all 3 motions
```python
sx, sy, sth = read_csv("Straight_path.csv")
cx, cy, cth = read_csv("Circular_path.csv")
rx, ry, rth = read_csv("Rotated_path.csv")
```

### Create one figure with two plots side by side
```python
plt.figure(figsize=(10,4))
```

This will give one PNG file with two subplots.

---

## 15.5 Left side graph – XY paths
```python
plt.subplot(1, 2, 1)
plt.plot(sx, sy, label="Straight")
plt.plot(cx, cy, label="Circular")
plt.plot(rx, ry, label="Rotated", marker="o")

plt.title("Week 1 - Robot Paths")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.grid(True)
plt.axis("equal")
plt.legend()
```

* `subplot(1, 2, 1)` → 1 row, 2 columns, we are now drawing in left panel.
* Straight path → almost a vertical line (since robot faces +Y).
* Circular path → curved line (because `Vr > Vl`).
* Rotated path → points almost at same (x, y), robot spins in place.
* `axis("equal")` → same scale on X and Y, so circles do not look stretched.

---

## 15.6 Right side graph – θ vs step (rotation test)
```python
steps = list(range(len(rth)))
plt.subplot(1, 2, 2)
plt.plot(steps, rth)

plt.title("Rotation Test - Heading vs Step")
plt.xlabel("Step")
plt.ylabel("theta (deg)")
plt.grid(True)
```

* X-axis = step number (0, 1, 2, …).
* Y-axis = `theta_deg`.
* For rotation test, θ increases quickly → shows how fast robot turns.

> This makes orientation (θ) clearly visible without needing arrows.

---

## 15.7 Flow of running everything

### Run C++ program

Compile and run `main.cpp`.

It generates:

* `Straight_path.csv`
* `Circular_path.csv`
* `Rotated_path.csv`

### Run Python plotting script

Run `graph-plot.py`:
```bash
python graph-plot.py
```

This script:

* reads all three CSVs,
* creates a combined figure with:
  * Left: XY paths
  * Right: θ vs step for rotation
* saves the image as:
  * `week1_path_plot.png`

### Result

We now have a visual summary of:

* how the robot moved in X–Y,
* how its orientation changed when rotating.

---

## 15.8 Quick Summary

Day 7 converts our odometry output into a graphical path.

> C++ generates pose data → Python turns it into a picture.

We now understand robot motion by seeing straight, circular, and rotating behaviour in a single PNG file.
