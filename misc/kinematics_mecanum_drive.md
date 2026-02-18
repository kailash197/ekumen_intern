# Mecanum drive kinematics
- [Source: Section 11.6](https://file.tavsys.net/control/controls-engineering-in-frc.pdf)
- A mecanum drive has four wheels, one on each corner of a rectangular chassis.
- The wheels have rollers offset at 45 degrees (whether it’s clockwise or not varies per wheel).
---

## Assumptions
*   Let:
    *   $v_x$ : robot's linear velocity along x-axis
    *   $v_y$ : robot's linear velocity along y-axis
    *   $\omega$ : robot's counterclockwise angular velocity about z-axis
    ---
    *   ${v}_{fl}$ : velocity vector of the front-left wheel contact point relative to the ground
    *   $(r_{fl_x},\ r_{fl_y})$ : position of the front-left wheel relative to the robot’s center.
    ---
## Equations
Tangential velocity if robot is turning with some angular velcity is given by:

$$ \vec{v} = \vec{\omega} \times \vec{r} $$

***



## Mecanum Wheel Inverse Kinematics

For a **mecanum wheeled robot** (4 wheels, each with rollers at 45°), the inverse kinematics relate **global velocity commands** to **individual wheel linear velocities**.
$$
\begin{bmatrix}
v_{fl} \\
v_{fr} \\
v_{rl} \\
v_{rr} \\
\end{bmatrix}
= \frac{1}{\sqrt{2}}
\begin{bmatrix}
1 & -1 & -(r_{fl_{x}} +r_{fl_{y}}) \\
1 & 1 &  (r_{fr_{x}} -r_{fr_{y}}) \\
1 & 1 & (r_{rl_{x}} -r_{rl_{y}}) \\
1 & -1 &  -(r_{rr_{x}} +r_{rr_{y}}) \\
\end{bmatrix}
\begin{bmatrix}
v_x \\
v_y \\
\omega \end{bmatrix}
$$

Where:
- $r_{fl_x}, r_{fl_y} \quad\text{– } x,y\text{ coordinates of front-left wheel relative to robot center}$
- $r_{fr_x}, r_{fr_y} \quad\text{– } x,y\text{ coordinates of front-right wheel relative to robot center}$
- $r_{rl_x}, r_{rl_y} \quad\text{– } x,y\text{ coordinates of rear-left wheel relative to robot center}$
- $r_{rr_x}, r_{rr_y} \quad\text{– } x,y\text{ coordinates of rear-right wheel relative to robot center}$

**Special notes**
1. This matrix assumes the **roller angle is 45°** (most common).
2. If rollers are at another angle $ \alpha $, the 1’s and -1’s become $ \cot\alpha $ etc.
3. The matrix is **constant** — independent of robot orientation $\theta$ — because mecanum can move in any global $v_x, v_y$ without changing heading. This is the big advantage over differential drive.

---
