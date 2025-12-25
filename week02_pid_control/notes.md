# Week 2 – PID Control Math (Theory)

## 1. The Problem: Motors Don't Behave Perfectly

Last week we commanded: "left wheel spin at 2 m/s" and assumed it happened.

**Real motors don't work that way.**

When we send a PWM signal:
- Motor might be **too slow** (weak power)
- Motor might **overshoot** (too fast, then settles)
- Motor might **oscillate** (speed goes up, down, up, down)
- **Load changes** (carrying weight) → speed drops

### Example

We want motor at **200 RPM**. What we actually measure:
```
Time 0s:   50 RPM    (too slow)
Time 1s:   180 RPM   (getting better)
Time 2s:   250 RPM   (overshot!)
Time 3s:   190 RPM   (oscillating)
Time 4s:   200 RPM   (finally)
```

Motor overshoots, oscillates, and takes time to settle.

### Why It Matters

If motors don't reach target speeds:
- Robot won't move straight (one wheel slow, one fast)
- Robot won't turn at right angles
- Odometry gets wrong data
- Navigation fails

**Solution:** Use **PID Control** to automatically adjust motor power.

---

## 2. What is PID Control?

PID is a **feedback system** that:
1. Measures actual speed
2. Compares it to target speed
3. Automatically adjusts motor power (PWM) to fix the difference
4. Repeats this continuously

### The Loop

```
Goal: 200 RPM

Each iteration (100 times/second):
  1. Read actual speed → 180 RPM
  2. Calculate error = 200 - 180 = 20 RPM
  3. Adjust PWM based on error
  4. Motor spins faster
  5. Repeat
```

**PID = Proportional + Integral + Derivative**

Three different ways of looking at error, together giving smooth control.

---

## 3. The Three Terms

### P – Proportional (React Now)

**Bigger error → bigger correction**

```
correction = Kp × error
```

If error = 20 RPM → correction = 1.0 × 20 = 20  
If error = 5 RPM → correction = 1.0 × 5 = 5

**Problem:** Small errors never fully correct. Motor gets stuck at 198 instead of 200.

### I – Integral (Fix Accumulated Error)

**If small error persists, keep pushing**

```
accumulated_error = sum of all errors over time
correction += Ki × accumulated_error
```

If we have 2 RPM error for 5 seconds:
```
Time 1s: accumulated = 2
Time 2s: accumulated = 4
Time 3s: accumulated = 6
...
```

As it grows, controller pushes harder until target is reached.

**Problem:** Too much I causes overshoot and oscillation.

### D – Derivative (Prevent Overshoot)

**Predict if we're approaching target too fast**

```
error_rate = (error_now - error_before) / dt
correction -= Kd × error_rate
```

When error is decreasing fast (we're improving), D term says "slow down, you'll overshoot."

Prevents the motor from shooting past 200 to 220 and coming back.

---

## 4. The PID Formula

```
error = target - actual
accumulated_error += error × dt
error_rate = (error - previous_error) / dt

PWM = Kp × error + Ki × accumulated_error + Kd × error_rate
```

**What happens:**
- **P term** → immediate reaction
- **I term** → removes steady-state error
- **D term** → smooth approach, no overshoot

Result: Motor reaches target **fast, smooth, accurate.**

---

## 5. Tuning: Kp, Ki, Kd Values

We need to pick three numbers that control how aggressive each term is.

| Gain | Too Low | Too High |
|------|---------|----------|
| **Kp** | Slow response | Oscillates, unstable |
| **Ki** | Final error remains | Causes oscillation |
| **Kd** | Overshoot happens | Response too sluggish |

**Typical ranges:**
- Kp = 0.5 to 2.0
- Ki = 0.05 to 0.2
- Kd = 0.1 to 1.0

(Exact values depend on motor characteristics)

---

## 6. PID vs No Control (Motor Speed Example)

### Without PID
Motor commanded to 200 RPM, actual speed wobbles:
```
0s: 100 RPM  →  1s: 180  →  2s: 240  →  3s: 210  →  4s: 205  →  5s: 200
```
Takes 5 seconds, overshoots to 240, oscillates.

### With PID (Kp=1.0, Ki=0.1, Kd=0.5)
```
0s: 100 RPM  →  1s: 185  →  2s: 198  →  3s: 200  →  3.5s: 200 (stable)
```
Reaches target in 3.5 seconds, no overshoot, smooth.

---

## 7. Why We Need PID for Our Robot

In Week 1, we computed:
```
v = (Vr + Vl) / 2
```

`Vr` and `Vl` come from **encoders** (actual wheel speeds).

**Without PID:**
- We command left motor 2 m/s
- Left motor spins at 1.8 m/s (non-ideal motor)
- Odometry gets wrong speed
- Robot drifts sideways instead of going straight

**With PID:**
- We command left motor 2 m/s (target)
- PID controller adjusts PWM continuously
- Left motor actually spins at ~2.0 m/s
- Odometry is accurate
- Robot moves straight

For our household robot startup, we need:
- Straight lines (no drifting)
- Precise turns
- No wobbling

**All depend on accurate motor speed control via PID.**

---

## 8. Quick Troubleshooting

| Symptom | Fix |
|---------|-----|
| Motor responds too slowly | Increase Kp |
| Speed oscillates (up-down-up) | Decrease Kp, increase Kd |
| Final speed is slightly off | Increase Ki |
| Motor overshoots target | Increase Kd |
| Response feels sluggish | Decrease Kd, increase Kp |

---

## 9. Summary

**PID Control:**
1. Reads actual motor speed (encoder)
2. Calculates error from target
3. Adjusts motor power based on three feedback terms:
   - **P:** Immediate reaction
   - **I:** Removes steady-state error
   - **D:** Prevents overshoot
4. Repeats 100+ times per second

**Result:** Motor reaches and maintains target speed smoothly, accurately, without oscillation.

---

## 10. Next Steps (Week 2 Coding)

This week we understood PID conceptually. Next we will:
1. Write a PID class in C++
2. Simulate it controlling virtual motor speed
3. Plot response (speed vs time)
4. Tune Kp, Ki, Kd for best behavior
5. Test with sudden speed changes and load changes

---

## One-Line Memory

> **P + I + D feedback terms automatically adjust motor power to reach and maintain target speed smoothly and accurately.**

---

May Shree_Krsna bless Week 2 🙏
