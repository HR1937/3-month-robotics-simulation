# Week 1 – Differential Drive Kinematics & Odometry

### Status: 🟩 Completed / Learned (Theory) — Now implementing simulation

This week focuses on the fundamentals of **how a 2-wheel differential drive robot moves**, and how to mathematically track its position (odometry) using wheel speeds.

---

## Goals of Week 1

By the end of this week I will:

- understand differential drive robots (2 driven wheels, 1 caster)
- define robot pose: `x, y, theta`
- compute:
  - forward velocity `v`
  - angular velocity `ω`
- update robot pose using odometry equations
- simulate:
  - straight-line motion
  - rotation in place
  - circular motion (different wheel speeds)
- generate XY path & save as `week1_path_plot.png`

---

## Files in this folder

```text
week01_differential_drive_odometry/
│
├─ README.md     → this file
├─ notes.md      → theory, formulas, explanations
└─ src/          → simulation code (C++)
```
