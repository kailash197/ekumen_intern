### Stochastic
The term **stochastic** refers to something that involves randomness or uncertainty. In mathematics and engineering, a stochastic process is one that evolves over time in a way that is not completely deterministic, meaning its future states depend on both predictable factors and random variables.

For example:

*   **Deterministic system:** If you know the initial conditions, you can predict the exact outcome.
*   **Stochastic system:** Even with initial conditions known, the outcome has some randomness (e.g., stock prices, weather patterns).

It’s widely used in fields like probability theory, statistics, machine learning, and control systems to model real-world phenomena where uncertainty plays a role.

***

### Stochastic control theory

Stochastic control theory is a branch of control theory that addresses uncertainty in system dynamics or observations by modeling random noise with probability distributions. The goal is to achieve desired control objectives despite this uncertainty, often minimizing a cost function (as in LQR). It involves representing linear stochastic systems in state-space form, understanding probability fundamentals, and deriving optimal estimators like the Kalman filter, which combines predictions and noisy measurements. This introduction is math-intensive, and prior reading of Kalman and Bayesian Filters in Python by Roger Labbe is recommended.

---


### Terminology
> A `causal system` is one that uses only past information. A noncausal system also uses information from the future.
---


### A complete state-space model
A **complete state-space model** refers to the full mathematical description of a system in state-space form, which includes **both**:

1.  **State Equation** (Internal Behavior: how the internal states evolve):
    $$
    \dot{x} = Ax + Bu \tag{1}
    $$
    *   Describes the dynamics of the system (rate of change of states).

    *   **State equation** models robot physics (position, velocity, etc.).

2.  **Output Equation** (External Behavior: how outputs relate to states and inputs):
    $$
    y = Cx + Du \tag{2}
    $$
    *   Describes what you can measure (sensor outputs) based on states and inputs.
    *   **Output equation** models sensors (encoders, gyros) and how they read those states.

---
Together:
$$
\begin{cases}
\dot{x} = Ax + Bu 
\\
y = Cx + Du
\end{cases}
$$

This is the standard representation for **linear time-invariant (LTI) systems** in control theory.
***

The equation (1):
$$\dot{x} = Ax + Bu$$

is the **state-space representation** of a **linear time-invariant (LTI) system** in control theory, where:

*   $\dot{x}$ is the derivative of the state vector (x) with respect to time (i.e., how the state changes).
*   $x$ is the state vector (e.g., position, velocity, angle).
*  $u$ is the input vector (e.g., motor voltage or torque).
*   $A$ is the **system matrix** that describes how the current state influences its rate of change.
*   $B$ is the **input matrix** that describes how the control input affects the state.

***

### **Interpretation**

> The rate of change of the system’s state $\dot{x}$ depends on:
> * The current state $x$ scaled by $A$
> * The control input $u$ scaled by $B$

***

### **Example in FRC**

For a simple drivetrain:

*   $x = \begin{bmatrix} \text{position} \\ \text{velocity} \end{bmatrix}$
*   $u = \text{motor voltage}$
*   $A$ and $B$ come from the robot’s physics (mass, friction, gear ratio).

So:
$$
\begin{bmatrix}
\dot{\text{position}} \\
\dot{\text{velocity}}
\end{bmatrix}

=

\begin{bmatrix}
0 & 1 \\
0 & -\frac{b}{m}
\end{bmatrix}

\begin{bmatrix}
\text{position} \\
\text{velocity}
\end{bmatrix}

+

\begin{bmatrix}
0 \\
\frac{k}{m}
\end{bmatrix}
\text{voltage}
$$

***

The equation (2):
$$ y = Cx + Du \tag{2} $$

is the **output equation** in the state-space representation, where:

*   $y$ is the Output vector (what you measure, e.g., sensor readings)
*   $x$ is the State vector (internal states like position, velocity)
*   $u$ is the Input vector (control inputs like motor voltage)
*   $C$ is the Output matrix (maps states to outputs)
*   $D$ is the Feedthrough matrix (maps inputs directly to outputs)


This equation describes how the **measured outputs** relate to:

*   The internal states $x$ via $C$ 
*   The control inputs $y$ via $D$

If $D = 0$, the output depends only on the states.  
If $D \neq 0$, the input has an immediate effect on the output.

**Example: For a drivetrain:**

*   Sensors measure **position** and **velocity**.
*   So:
    $$
    y =
    \begin{bmatrix}
    \text{position sensor} \\
    \text{velocity sensor}
    \end{bmatrix}
    =
    \begin{bmatrix}
    1 & 0 \\
    0 & 1
    \end{bmatrix}
    
    \begin{bmatrix}
    \text{position} \\
    \text{velocity}
    \end{bmatrix}

    + 
    \begin{bmatrix}
    0 \\
    0
    \end{bmatrix}
    \text{voltage}
    $$
***

## State Observers
### Luenberger observer
### Luenberger observer with separate predict/update.
Eigenvalues of closed-loop observer
### Separation principle


## Introduction to probability
### Random variables
### Expected value
### Variance
### Joint PDF
### Covariance
### Correlation
### Independence
### Marginal probability density functions
### Conditional probability density functions
### Relations for independent random vectors
### Gaussian random variables
Definition 9.3.1 — Central limit theorem

## Linear stochastic systems




