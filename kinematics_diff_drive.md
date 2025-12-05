# Differential drive

## Differential drive algorithm
Differential drive algorithm computes:
1. wheel velocities from desired linear and angular velocity (IK)
2. Robot pose update (Odometry)

## Kinematics for Differential Drive Robot
*   Let:
    *   $v_c$ = velocity of the robot's center
    *   $v_l$ = velocity of the left wheel
    *   $v_r$ = velocity of the right wheel  </p>
    *   $\omega$ = counterclockwise turning rate around the center of rotation
    *   $\omega_l$ = angular velocity of left wheel
    *   $\omega_r$ = angular velocity of right wheel </p>
    *   $r_b$ = distance from the center to each wheel
    *   $2r_b$ = wheel separation

---

### **Inverse Kinematics for Differential Drive Robot**
---

- map the **linear velocity** $v_c$ and **angular velocity** $\omega$ of the robot to the **left** and **right wheel velocities** $v_l$ and $v_r$



#### **Left Wheel Velocity**
$$ \vec{v}_l = (v_c - {\omega}r_b)\hat{i} $$

Project onto wheel direction, $\hat{i}$ direction:

$$ v_l = \vec{v}_l.\hat{i} $$
$$ v_l = (v_c - {\omega}r_b)\hat{i}.\hat{i} $$
$$ v_l = v_c - {\omega}r_b $$

Thus $eq. (1)$,

$$ v_l = v_c - {\omega}r_b $$

#### **Right Wheel Velocity**
Similarly $eq. (2)$,

$$ v_r = v_c + {\omega}r_b $$

***
### **Forward Kinematics for Differential Drive Robot**
---
Substituting for $v_c$ in $eq(1)$, we get $eq. (3)$,

$$
\omega = \frac{v_r-v_l}{2r_b}
$$

Substituting $eq(3)$ back for $v_c$ in $eq(2)$, we get $eq. (4)$,

$$
v_c = \frac{v_r+v_l}{2}
$$ 

These equations, $eq.\ (3)$ and  $eq.\ (4)$,  map wheel velocities back to robot linear and angular velocities.

---
### **Inverse Kinematics for Differential Drive Robot**
$$v_l = v_c - {\omega}r_b$$
$$v_r = v_c + {\omega}r_b$$

These equations show how the robot’s linear and angular velocities translate into individual wheel speeds.
### **Forward Kinematics for Differential Drive Robot**
$$ \omega = \frac{v_r-v_l}{2r_b}$$
$$ v_c = \frac{v_r+v_l}{2}$$

These equations map wheel velocities back to robot linear and angular velocities.

---
