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

### Inverse Kinematics
Robots is moving at following velocity:
- Translation or linear velocity, $\vec{v}_l = v_x\hat{i} + v_y\hat{j}$
- Angular velocity, $\vec{\omega} = \omega\hat{k}$
- rotational contribution to the wheel’s velocity due to the robot spinning.

The velocity vector of the front-left wheel contact point relative to the ground is the vector sum of:  
- the translational velocity of the robot’s center projected onto the wheel.
- tangential velocity at wheel due to robot's rotation  

$$
\begin{aligned}
\vec{v}_{fl} &= \vec{v}_{linear} + \vec{v}_{tangent}\\
             &= v_x\hat{i} + v_y\hat{j} + \vec{\omega} \times \vec{r}_{fl}
\end{aligned}
$$



$$
\begin{equation}
\begin{aligned}
    x &= 1 \times 2 + 3 \\
      &= 2 + 3 \\
      &= 5 
\end{aligned}
\end{equation}
$$

$$
\begin{equation}
\begin{aligned}
    x &= 1 \times 2 + 3 \\
      &= 2 + 3 \\
      &= 5 
\end{aligned}
\end{equation}
$$

