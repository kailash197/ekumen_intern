# Definitions
Timestamps
$$
t_k: \text{current timestamp} \\
t_{k-1}: \text{previous timestamp} \\
\Delta t = t_k  - t_{k-1}
$$

If $f$ is control loop frequency:
$$t =\Delta t = \frac{1}{f}$$
---
Angular wheel positions
$$
\phi_{l_k}: \text{left wheel position at current timestamp, }t_k \\
\phi_{l_{k-1}}: \text{left wheel position at current timestamp, }t_{k-1}
$$
for right wheels:
$$
\phi_{r_k},\ \phi_{r_{k-1}}
$$
---

Angular wheel displacement vector
$$
\overrightarrow{\Delta \phi} =

\begin{bmatrix}
\Delta\phi_l \\
\Delta\phi_r
\end{bmatrix} =

\begin{bmatrix}
\phi_{l_k} - \phi_{l_{k-1}} \\
\phi_{r_k} - \phi_{r_{k-1}}
\end{bmatrix}
$$
---

Angular wheel velocity vector
$$
\boldsymbol{\omega}_w =

\begin{bmatrix}
\omega_l \\
\omega_r
\end{bmatrix}$$
---

Poses
$$
\mathbf{p} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
$$
---
Wheel radius matrix (diagonal)
$$
R_w = \begin{bmatrix} R & 0 \\ 0 & R \end{bmatrix}
$$
---
Sign correction matrix (from URDF axes)
$$
S = \text{diag}(s_l, s_r)
$$
---

**Robot twist** in robot body frame
$$
\boldsymbol{\xi}_b = \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}_R
$$
---
**Linear wheel velocity vector**
$$
\mathbf{v}_w = \begin{bmatrix} v_l \\ v_r \end{bmatrix}
$$
---

**Forward kinematics Jacobian**: (wheel → robot twist), $H$
$$ \boldsymbol{\xi} = H \mathbf{v}_w $$

**Inverse kinematics Jacobian**: (robot twist → wheel velocities), $H^+$:
$$ \mathbf{v}_w = H^+ \boldsymbol{\xi} $$
$$ H^+ = (H^T H)^{-1} H^T $$
---
Compute $ H^+ $:
1. Product: $H^T H$
2. Inverse of product: $(H^T H)^{-1}$
3. Multiply by $ H^T $: $(H^T H)^{-1} H^T$
---

**Motion integration matrix** (for rotating motion)
$$
M(\Delta\theta) = \begin{bmatrix}
\frac{\sin\Delta\theta}{\Delta\theta} & \frac{\cos\Delta\theta-1}{\Delta\theta} & 0 \\
\frac{1-\cos\Delta\theta}{\Delta\theta} & \frac{\sin\Delta\theta}{\Delta\theta} & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

  Also called **infinitesimal motion matrix** or **SE(2) displacement matrix**.

---

**Rotation/Transformation**
- $ R(\theta) $ — **Rotation matrix** (SO(2), robot → global)
  $$
  R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
  $$
- $ T(\theta) $ — **Homogeneous transformation** (SE(2))
  $$
  T(\theta) = \begin{bmatrix} R(\theta) & \mathbf{0} \\ \mathbf{0}^T & 1 \end{bmatrix}
  $$
- $ \text{diag}(R(\theta), 1) $ — **Block-diagonal pose rotation matrix**
---

**Pose Variables**
- $ \mathbf{p} $ — **Robot pose** in global frame
  $$
  \mathbf{p} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
  $$
- $ \Delta\mathbf{p}_R $ — **Pose displacement in robot frame**
  $$
  \Delta\mathbf{p}_R = \begin{bmatrix} \Delta x \\ \Delta y \\ \Delta\theta \end{bmatrix}_R
  $$
- $ \Delta\mathbf{p}_G $ — **Pose displacement in global frame**
$$
  \Delta\mathbf{p}_G = \begin{bmatrix} \Delta x \\ \Delta y \\ \Delta\theta \end{bmatrix}_G
  $$

---

### **Notation**
| Symbol | Name | Meaning |
|--------|------|---------|
| $$ \boldsymbol{\omega}_w $$ | Wheel velocity vector | Input from `/joint_states` |
| $$ H $$ | Forward kinematics matrix | Maps wheel → robot twist |
| $$ H^+ $$ | Inverse kinematics matrix | Maps robot twist → wheel |
| $$ \boldsymbol{\xi}_b $$ | Body twist | Robot velocity in body frame |
| $$ M(\Delta\theta) $$ | Motion integration matrix | Converts twist → displacement |
| $$ R(\theta) $$ | Rotation matrix | Robot → global rotation |
| $$ \mathbf{p} $$ | Pose vector | $$[x, y, \theta]^T$$ |

---
