# Algorithms
**Velocity Pipeline**
$$
\boxed{
\mathbf{p}_{\text{new}} = \mathbf{p}_{\text{prev}} + T(\theta_{\text{prev}}) \; M(\Delta\theta) \; H \; R_w \; S \; \overrightarrow{\omega}_w \; \Delta t
}
$$

**Displacement Pipeline**
$$
\boxed{
\mathbf{p}_{\text{new}} = \mathbf{p}_{\text{prev}} + T(\theta_{\text{prev}}) \, M(\Delta\theta) \,H\, R_w \, S \, \overrightarrow{\Delta\boldsymbol{\phi}}
}
$$

**Where:**
- $ \Delta\theta = \omega \Delta t$, where $\omega$ is robot angular velocity <font color=red>not wheel angular velocity</font>
- $\Delta t = t_k  - t_{k-1}$
- $T(\theta)$ — Pose rotation matrix
- $M(\Delta\theta)$ — Motion integration matrix
- $ H $ — Forward kinematics Jacobian
- $ R_w $ — Wheel radius matrix
- $ S $ — Sign correction matrix
- $ \overrightarrow{\omega}_w $ — Wheel angular velocity vector
---

## Alogrithm based on angular wheel positions

**Step 1:** Get angular wheel positions from `/joint_states/position` for current timestamp $t_k$:
$$
\phi_{l_k}, \phi_{l_{k-1}}, \phi_{r_k},\ \phi_{r_{k-1}}
$$
---

**Step 2:** Calculate angular wheel displacement vector
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

**Step 3:** Calculate angular wheel velocities
$$
\overrightarrow{\omega}_w =
\overrightarrow{\dot{\phi}} =
\frac{\overrightarrow{\Delta \phi}}{\Delta t},\ \ \Delta t = t_k  - t_{k-1}
$$
---

Then resume from **Step 2** of angular wheel velocity based method.

---

## Alogrithm based on angular wheel velocity

**Step 1:** Get angular wheel velocities from `/joint_states/velocity`
$$
\overrightarrow{\omega}_w =
\overrightarrow{\dot{\phi}} =
\begin{bmatrix} \omega_l \\ \omega_r \end{bmatrix}
$$
---

**Step 2:** Correct signs and convert to linear velocities
$$
\overrightarrow{\mathbf{v}_w} = R_w \cdot S \cdot \overrightarrow{\boldsymbol{\omega}_w}
$$
where,
$ R_w = \text{diag}(R, R) $
$ S = \text{diag}(s_l, s_r) $

---

**Step 3:** Robot twist in robot robot frame
$$
\overrightarrow{\boldsymbol{\xi}_R} = H \overrightarrow{\mathbf{v}_w}
$$
---

**Step 4:** Integration in robot robot frame
First calculate: $$ \Delta\theta = \omega \Delta t $$
where $$ \omega = [0\;0\;1] \cdot \overrightarrow{\boldsymbol{\xi}_R} $$
then, integration in robot frame to find $\Delta \mathbf{p}_R$
$$ \Delta\mathbf{p}_R = M(\Delta\theta) \, \overrightarrow{\boldsymbol{\xi}_R} \, \Delta t $$
where
$$
M(\Delta\theta) = \begin{bmatrix}
\frac{\sin\Delta\theta}{\Delta\theta} & \frac{\cos\Delta\theta-1}{\Delta\theta} & 0 \\
\frac{1-\cos\Delta\theta}{\Delta\theta} & \frac{\sin\Delta\theta}{\Delta\theta} & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

For very small $\Delta \theta \lt \epsilon$, (for practical implementations, $\epsilon \approx 1E-9$ ):

$$\frac{sin{\Delta\theta}}{\Delta \theta} \approx 1 - \frac{{\Delta \theta}^2}{6}$$
$$\frac{cos{\Delta\theta}-1}{\Delta \theta} \approx -\frac{\Delta \theta}{2}$$
$$\frac{1-cos{\Delta\theta}}{\Delta \theta} \approx \frac{\Delta \theta}{2}$$
---
**Step 5:** Rotate to global frame
$$
\Delta\mathbf{p}_G = T(\theta_{\text{prev}}) \, \Delta\mathbf{p}_R
$$
with
$$
T(\theta) = \begin{bmatrix} R(\theta) & \mathbf{0} \\ \mathbf{0}^T & 1 \end{bmatrix}
$$
---

**Step 6:** Update
$$
\mathbf{p}_{\text{new}} = \mathbf{p}_{\text{prev}} + \Delta\mathbf{p}_G
$$
---
