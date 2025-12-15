### Forward Euler Integration
Pose estimation: integrate the velocity in each orthogonal direction
over time

$$ x_{k+1}=x_k + v_k cos{\theta}_kT  $$
$$ y_{k=1}=y_k+v_ksin{\theta}_kT $$
$$ {\theta}_{k+1}={\theta}_{gyro,k+1} $$

where $T$ is the sample period and,  

**Assumption**: 
- Robot moves in a straight path between samples.  
- $\omega=0$ at all but sample times

### Pose exponential
- more accurate than Forward Euler Integration
- map `twist` to a `pose` using `exp` function, hence the name pose exponential, $$pose = e^{twist}$$

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
where:  
$v_x:$ robot velocity in forward direction  
$v_y:$ robot velocity sideways  
$\omega:$ robot's angular velocity $CCW: +ve$  
$t:$ is the total integration time, $t \approx \Delta t$ 

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

where:  
$G$ denotes global frame and $R$ denotes Robot frame.  
$\theta$ represents starting angle in global frame. (CCW +ve)

For very small $\Delta \theta \lt \epsilon$, (for practical implementations, $\epsilon \approx 1E-9$ ):

$$\frac{sin{\Delta\theta}}{\Delta \theta} \approx 1 - \frac{{\Delta \theta}^2}{6}$$
$$\frac{cos{\Delta\theta}-1}{\Delta \theta} \approx -\frac{\Delta \theta}{2}$$
$$\frac{1-cos{\Delta\theta}}{\Delta \theta} \approx \frac{\Delta \theta}{2}$$

