# Equations for differential drive odometry

## Kinematic Parameterss for Differential Drive Robot
- Assume
    *   $v_c$ = velocity of the robot's center
    *   $v_l$ = velocity of the left wheel
    *   $v_r$ = velocity of the right wheel  </p>
    *   $\omega$ = counterclockwise turning rate around the center of rotation
    *   $\omega_l$ = angular velocity of left wheel
    *   $\omega_r$ = angular velocity of right wheel </p>
    *   $r$ = radius of wheel
    *   $r_b$ = distance from the body center to each wheel
    *   $2r_b$ = wheel separation

## Source: `/joint_states`
- $k$: represents $k^{th}$ iteration ie for current timestamp
- $ \phi_{l_k}$: total rotation of left wheel (in rad)
- $ \phi_{r_k}$: total rotation of left wheel (in rad)
- $t_{current}$: current timestamp

    If $f$ is control loop frequency:
    $$t =\Delta t = \frac{1}{f}$$
    or using timestamp
    $$t=\Delta t = t_{current}​−t_{previous​}$$

## Calculations
### 1. Robot body motion
#### Option 1: Use calculated tangential displacements using wheel rotations from `\joint_states`

1. Differential wheel rotations in radians
$$\Delta \phi_l = \phi_{l_k} - \phi_{l_{k-1}}\\
\Delta \phi_r =  \phi_{r_k} - \phi_{r_{k-1}}$$

2. Wheel displacements: tangential distance moved by wheel by virtue of its rotation
$$\Delta s_l = r \Delta \phi_l\\
\Delta s_r = r \Delta \phi_r
$$

3. Robot delta pose
$$\Delta x = \frac {\Delta s_l + \Delta s_r} {2} = \frac{r}{2}(\Delta \phi_l+\Delta \phi_r)$$
$$\Delta y = 0$$
$$\Delta \theta = \frac {\Delta s_r - \Delta s_l} {r_b}= \frac{r}{2r_b}(\Delta \phi_r-\Delta \phi_l)$$
#### Option 2: Use angular velocities from `\joint_states` directly
1. Robot velocities (Forward Kinematics)
$$v_x=\frac{r}{2}(\omega_l + \omega_r)$$
$$v_y=0$$
$$\omega_c=\frac{r}{2r_b}(\omega_r - \omega_l)$$
2. Robot delta pose
$$\Delta x = v_xt = v_x\Delta t$$
$$\Delta y = 0$$
$$\Delta \theta =\omega t =\omega \Delta t$$
---
### 2. Pose integration
Given:
$v_x:$ robot velocity in forward direction
$v_y:$ robot velocity sideways
$\omega:$ robot's angular velocity $CCW: +ve$
$t:$ is the total integration time, $t \approx \Delta t$
$G$ denotes global frame and $R$ denotes Robot frame.
$\theta$ represents starting angle in global frame. (CCW +ve)


```math
\begin{bmatrix}\Delta x \\
\Delta y \\
\Delta \theta
\end{bmatrix}_G
=
\begin{bmatrix}
cos{\theta} & -sin{\theta} & 0 \\
sin{\theta} &  cos{\theta} & 0 \\
0 & 0 & 1
\end{bmatrix}

\begin{bmatrix}
\frac{sin{\omega}t}{\omega} & \frac{cos{\omega t}-1}{\omega} & 0 \\
\frac{1-cos{\omega t}}{\omega} & \frac{sin{\omega}t}{\omega} & 0 \\
0 & 0 & 1
\end{bmatrix}_R

\begin{bmatrix}
v_x \\
v_y \\
\omega
\end{bmatrix}_R
```
```math
\begin{bmatrix}\Delta x \\
\Delta y \\
\Delta \theta
\end{bmatrix}_G
=
\begin{bmatrix}
cos{\theta} & -sin{\theta} & 0 \\
sin{\theta} &  cos{\theta} & 0 \\
0 & 0 & 1
\end{bmatrix}

\begin{bmatrix}
\frac{sin{\omega}t}{\omega t} & \frac{cos{\omega t}-1}{\omega t} & 0 \\
\frac{1-cos{\omega t}}{\omega t} & \frac{sin{\omega}t}{\omega t} & 0 \\
0 & 0 & 1
\end{bmatrix}_R

\begin{bmatrix}
v_x  t\\
v_y t\\
\omega t
\end{bmatrix}_R
```

Also, if change in pose between updates is available:

```math
\begin{bmatrix}\Delta x \\
\Delta y \\
\Delta \theta
\end{bmatrix}_G
=
\begin{bmatrix}
cos{\theta} & -sin{\theta} & 0 \\
sin{\theta} &  cos{\theta} & 0 \\
0 & 0 & 1
\end{bmatrix}

\begin{bmatrix}
\frac{sin{\Delta\theta}}{\Delta \theta} & \frac{cos{\Delta\theta}-1}{\Delta \theta} & 0 \\
\frac{1-cos{\Delta\theta}}{\Delta \theta} & \frac{sin{\Delta\theta}}{\Delta \theta} & 0 \\
0 & 0 & 1
\end{bmatrix}_R

\begin{bmatrix}
\Delta x \\
\Delta y \\
\Delta \theta
\end{bmatrix}_R
```

For very small $\Delta \theta \lt \epsilon$, (for practical implementations, $\epsilon \approx 1E-9$ ):

$$\frac{sin{\Delta\theta}}{\Delta \theta} \approx 1 - \frac{{\Delta \theta}^2}{6}$$
$$\frac{cos{\Delta\theta}-1}{\Delta \theta} \approx -\frac{\Delta \theta}{2}$$
$$\frac{1-cos{\Delta\theta}}{\Delta \theta} \approx \frac{\Delta \theta}{2}$$
---

### 3. Estimated Current Pose

```math
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}_k
=
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}_{k-1} +
\begin{bmatrix}
\Delta x \\
\Delta y \\
\Delta \theta
\end{bmatrix}_G
```

---
## Summary:
The incremental pose update in the global frame given robot-frame velocities can be expressed as:

```math
\begin{bmatrix}\Delta x \\
\Delta y \\
\Delta \theta
\end{bmatrix}_G
=
\begin{bmatrix}
cos{\theta} & -sin{\theta} & 0 \\
sin{\theta} &  cos{\theta} & 0 \\
0 & 0 & 1
\end{bmatrix}
.
\begin{bmatrix}
\frac{sin{\Delta\theta}}{\Delta \theta} & \frac{cos{\Delta\theta}-1}{\Delta \theta} & 0 \\
\frac{1-cos{\Delta\theta}}{\Delta \theta} & \frac{sin{\Delta\theta}}{\Delta \theta} & 0 \\
0 & 0 & 1
\end{bmatrix}_R
.
\begin{bmatrix}
v_x \\
v_y \\
\omega
\end{bmatrix}_R

\Delta t
```
or, in short,

$$
\begin{bmatrix} \Delta x \\ \Delta y \\ \Delta \theta \end{bmatrix}_G
= R(\theta) \cdot M(\Delta\theta) \cdot \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}_R \Delta t
$$
where:
- $R(\theta)$ = rotation from robot to global frame (correct)
- $M(\Delta\theta)$ = a matrix that handles **nonholonomic integration effects** when $\omega \neq 0$

---
