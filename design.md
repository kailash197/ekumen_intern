# Considerations

## Scope of Design
- What does my design need to cover? 
- Which is the scope?
- What are input and output of the system?
- Which are the inputs of the system?
---

- Design should cover generalized IK based driving algorithms like differential drive  algorithm. Various drives to be considered are:
    1. Differential drive
    2. Ackermann drive
    3. Omnidirectional drive
    4. Swerve drive
- Input and Output
    - Twist (linear and angular robot velocities)
    - Wheel odometry
---
## Block Diagram
![System Diagram](./WheelOdom.svg)

---

## Components
- Which are the individual components that I need to implement?
- What are the responsibilities? 
- How do they talk to each other?
- Is my design synchronous or asynchronous?
- States
    - How do we keep state?
    - Is the algorithm stateful or stateless?
---
The overall design is asynchronous and distributed by virtue of ROS2. There may be synchronous components like ROS2 service calls, Parameter server, etc. which are blocking.

#### <u> Non-ROS Components:</u>
The design of non-ROS components are asynchronous in nature.
1. **IK implementation**
    - converts required robot linear and angular velocity into wheel velocities which are input to motor control hardware.
2. **Wheel Odometry**
    - includes implementation of **pose exponential** for pose estimation
3. **FK implementation** <font color='red'>is this required?</font>

#### <u> ROS components</u>:
There may be synchronous components like service calls.
1. Kinematic Topic Publisher/Subscriber node:
    - Asynchronous
    - reads twist from `\cmd_vel` topic
    - call back **IK implementation**
    - publishes wheel velocities to topic which motor driver listens to
2. Odometry Publisher Node:
    - Asynchronous
    - read wheel ticks from encoder (is it from some topic??)
    - call **pose exponential**
    - publish odometry to `\odom` topic

#### <u> Statefulness of Algorithms </u>
1. **Pose estimation** is inherently stateful as the algorithm tries to determine the robot's position and orientation over time by using past states, as 
$$pose_t​=pose_{t−1}​+\Delta pose$$
2. **Inverse kinematics (IK)** and **Forward kinematics (FK)** equations are generally stateless unless numerical iterative methods (which use intermediate states) are employed to achieve solution.


    In a ROS node, the state is maintained by storing current pose, velocity, cov, etc as class member variables.

---

## Testing
- How can I test every line of code?
---
1. **Static Analysis Tools** helps test every line of code at least for:
    - Formatting
    - Styling: check code for style violations and probelematic patterns
    - Common bugs: find potential bugs and errors like null ptr dereferences, memory leaks, undefined behavior, etc.

2. **Unit Testing**
    - Use GTest to test every single functions/methods for expected outcomes and expected errors/exceptions.

3. **ROS Unit Testing**
    - To test whether ROS interfaces are communicating or connected as expected
    - might need to check in insolation as well

---
## Dependencies

- Which are my dependencies?  
- Why do I depend on them?  
- Is it worth implementing them on my own?

---

Following are dependencies:
1. `Eigen3` library
    - a header-only library, so no linking hassle: just #include the headers
    - handles vectors, matrices, linear algebra, transformations, and even quaternions, which makes it perfect for FK, IK, and pose exponential calculations in robotics.
    - Why: 
        - manual implementation in C++ is error-prone
        - `Eigen3` is heavily optmized
        - Compatibility to ROS/Gazebo: messages, TFs, gazebo expect Eigen3 types, so need to put effort on conversion to Eigen3 types as well.
    - No manual implementation:
        - Eigen3 is faster, safer and future-proof.
        - Save extra effort on developing and testing own math library.
2. ROS ??
***

## Bottlenecks
- Which part is the riskiest part in the entire design?  
`you can think of the riskiest as the part of the system that you know less about.`

- Which is the complexity of the algorithm? Time and memory.
---
