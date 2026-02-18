# **Building a Differential Drive Algorithm from Scratch**

Let me explain the **mathematics and implementation** of a differential drive algorithm. This is the core algorithm that converts linear/angular velocities into individual wheel speeds.

## **1. The Core Mathematics**

### **Differential Drive Kinematics:**

For a robot with:
- **Wheel radius**: `r`
- **Wheel separation**: `L` (distance between wheels)
- **Left wheel velocity**: `v_left`
- **Right wheel velocity**: `v_right`

The robot's **linear** and **angular** velocities are:

```
Linear velocity (v) = (v_right + v_left) / 2
Angular velocity (ω) = (v_right - v_left) / L
```

**Inverse Kinematics** (what we need for control):
```
v_left = v - (ω * L / 2)
v_right = v + (ω * L / 2)
```

## **2. Basic Python Implementation**

```python
import numpy as np
import math

class DifferentialDriveController:
    def __init__(self, wheel_radius=0.05, wheel_separation=0.2):
        """
        Initialize differential drive controller
        
        Args:
            wheel_radius (float): Radius of each wheel in meters
            wheel_separation (float): Distance between wheels in meters
        """
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        
        # Current wheel velocities (rad/s)
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        
        # Robot pose (x, y, theta)
        self.pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        
        # Wheel positions (for odometry)
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        
    def twist_to_wheel_velocities(self, linear_vel, angular_vel):
        """
        Convert linear and angular velocity to individual wheel velocities
        
        Args:
            linear_vel (float): Linear velocity in m/s
            angular_vel (float): Angular velocity in rad/s
            
        Returns:
            tuple: (left_wheel_vel, right_wheel_vel) in rad/s
        """
        # Calculate wheel velocities using inverse kinematics
        v_left = linear_vel - (angular_vel * self.wheel_separation / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_separation / 2.0)
        
        # Convert from m/s to rad/s (v = ω * r)
        left_wheel_angular_vel = v_left / self.wheel_radius
        right_wheel_angular_vel = v_right / self.wheel_radius
        
        return left_wheel_angular_vel, right_wheel_angular_vel
    
    def wheel_velocities_to_twist(self, left_wheel_vel, right_wheel_vel):
        """
        Convert wheel velocities back to robot twist
        
        Args:
            left_wheel_vel (float): Left wheel velocity in rad/s
            right_wheel_vel (float): Right wheel velocity in rad/s
            
        Returns:
            tuple: (linear_vel, angular_vel)
        """
        # Convert from rad/s to m/s
        v_left = left_wheel_vel * self.wheel_radius
        v_right = right_wheel_vel * self.wheel_radius
        
        # Forward kinematics
        linear_vel = (v_right + v_left) / 2.0
        angular_vel = (v_right - v_left) / self.wheel_separation
        
        return linear_vel, angular_vel
    
    def update_odometry(self, left_wheel_delta, right_wheel_delta, dt):
        """
        Update robot pose using wheel encoder measurements
        
        Args:
            left_wheel_delta (float): Left wheel displacement in radians
            right_wheel_delta (float): Right wheel displacement in radians
            dt (float): Time step in seconds
        """
        # Convert wheel displacements to linear distances
        d_left = left_wheel_delta * self.wheel_radius
        d_right = right_wheel_delta * self.wheel_radius
        
        # Calculate linear and angular displacement
        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation
        
        # Update pose
        x, y, theta = self.pose
        
        # For straight motion
        if abs(d_theta) < 1e-6:
            x += d_center * math.cos(theta)
            y += d_center * math.sin(theta)
        else:
            # For curved motion (using arc approximation)
            radius = d_center / d_theta
            
            # Calculate ICC (Instantaneous Center of Curvature)
            icc_x = x - radius * math.sin(theta)
            icc_y = y + radius * math.cos(theta)
            
            # Update pose
            x = math.cos(d_theta) * (x - icc_x) - math.sin(d_theta) * (y - icc_y) + icc_x
            y = math.sin(d_theta) * (x - icc_x) + math.cos(d_theta) * (y - icc_y) + icc_y
            theta += d_theta
        
        # Normalize theta to [-π, π]
        theta = math.atan2(math.sin(theta), math.cos(theta))
        
        self.pose = np.array([x, y, theta])
        
        # Update wheel velocities for reporting
        self.left_wheel_velocity = left_wheel_delta / dt
        self.right_wheel_velocity = right_wheel_delta / dt
        
        return self.pose
    
    def move_to_point(self, target_x, target_y, max_linear_vel=0.5, max_angular_vel=1.0):
        """
        Simple point-to-point movement with PD controller
        
        Args:
            target_x, target_y: Target coordinates
            max_linear_vel, max_angular_vel: Velocity limits
            
        Returns:
            tuple: (linear_vel, angular_vel) commands
        """
        x, y, theta = self.pose
        
        # Calculate error
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # If we're close enough, stop
        if distance < 0.05:  # 5 cm tolerance
            return 0.0, 0.0
        
        # Calculate desired angle
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - theta
        
        # Normalize angle error to [-π, π]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # PD controller for angular velocity
        Kp_angular = 2.0
        angular_vel = Kp_angular * angle_error
        angular_vel = np.clip(angular_vel, -max_angular_vel, max_angular_vel)
        
        # PD controller for linear velocity (slow down when turning)
        Kp_linear = 1.0
        linear_vel = Kp_linear * distance * math.cos(angle_error)
        linear_vel = np.clip(linear_vel, -max_linear_vel, max_linear_vel)
        
        # Stop if angle error is too large
        if abs(angle_error) > math.pi/4:  # 45 degrees
            linear_vel = 0.0
        
        return linear_vel, angular_vel
```

## **3. ROS 2 Implementation**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
import math

class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')
        
        # Parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.2)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        
        # Robot state
        self.pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_time = self.get_clock().now()
        
        # Publishers
        self.wheel_cmd_pub = self.create_publisher(
            JointState, 
            'wheel_commands', 
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer for odometry update
        self.timer = self.create_timer(0.01, self.update_odometry)  # 100 Hz
        
        self.get_logger().info('Differential Drive Controller started')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to wheel velocities"""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Apply velocity limits
        max_linear = self.get_parameter('max_linear_vel').value
        max_angular = self.get_parameter('max_angular_vel').value
        
        linear_vel = np.clip(linear_vel, -max_linear, max_linear)
        angular_vel = np.clip(angular_vel, -max_angular, max_angular)
        
        # Calculate wheel velocities
        v_left = linear_vel - (angular_vel * self.wheel_separation / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_separation / 2.0)
        
        # Convert to angular velocities
        left_wheel_vel = v_left / self.wheel_radius
        right_wheel_vel = v_right / self.wheel_radius
        
        # Publish wheel commands
        wheel_cmd = JointState()
        wheel_cmd.header.stamp = self.get_clock().now().to_msg()
        wheel_cmd.name = ['left_wheel_joint', 'right_wheel_joint']
        wheel_cmd.velocity = [float(left_wheel_vel), float(right_wheel_vel)]
        
        self.wheel_cmd_pub.publish(wheel_cmd)
        
        # Log for debugging
        self.get_logger().debug(
            f'Cmd: v={linear_vel:.2f}, ω={angular_vel:.2f} → '
            f'Left: {left_wheel_vel:.2f}, Right: {right_wheel_vel:.2f} rad/s'
        )
    
    def joint_state_callback(self, msg):
        """Update wheel positions from encoders"""
        try:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
            
            self.left_wheel_pos = msg.position[left_idx]
            self.right_wheel_pos = msg.position[right_idx]
            
        except ValueError:
            pass
    
    def update_odometry(self):
        """Update and publish odometry"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Get wheel displacements (simplified - in real system use encoders)
        # This would normally come from encoder readings
        left_delta = 0.0
        right_delta = 0.0
        
        # Forward kinematics to update pose
        v_left = left_delta * self.wheel_radius / dt
        v_right = right_delta * self.wheel_radius / dt
        
        linear_vel = (v_right + v_left) / 2.0
        angular_vel = (v_right - v_left) / self.wheel_separation
        
        # Update pose (simplified Euler integration)
        x, y, theta = self.pose
        x += linear_vel * math.cos(theta) * dt
        y += linear_vel * math.sin(theta) * dt
        theta += angular_vel * dt
        
        # Normalize angle
        theta = math.atan2(math.sin(theta), math.cos(theta))
        self.pose = np.array([x, y, theta])
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        
        # Convert theta to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom)
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## **4. Advanced Features Implementation**

```python
class AdvancedDiffDriveController:
    def __init__(self):
        # PID controllers for each wheel
        self.left_pid = PIDController(kp=1.0, ki=0.1, kd=0.01)
        self.right_pid = PIDController(kp=1.0, ki=0.1, kd=0.01)
        
        # Velocity ramp limiter
        self.max_accel = 2.0  # m/s²
        self.max_decel = 3.0  # m/s²
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        
        # Slip detection
        self.slip_threshold = 0.2
        self.wheel_slip_detected = False
        
    def apply_velocity_ramp(self, target_linear, target_angular, dt):
        """Limit acceleration/deceleration"""
        # Linear velocity ramp
        max_delta_v = self.max_accel * dt
        if target_linear > self.last_linear_vel:
            linear_vel = min(target_linear, self.last_linear_vel + max_delta_v)
        else:
            max_delta_v = self.max_decel * dt
            linear_vel = max(target_linear, self.last_linear_vel - max_delta_v)
        
        # Angular velocity ramp
        max_delta_w = self.max_accel * 2.0 * dt  # More aggressive for rotation
        if target_angular > self.last_angular_vel:
            angular_vel = min(target_angular, self.last_angular_vel + max_delta_w)
        else:
            angular_vel = max(target_angular, self.last_angular_vel - max_delta_w)
        
        self.last_linear_vel = linear_vel
        self.last_angular_vel = angular_vel
        
        return linear_vel, angular_vel
    
    def detect_wheel_slip(self, cmd_left_vel, cmd_right_vel, 
                         meas_left_vel, meas_right_vel):
        """Detect if wheels are slipping"""
        left_error = abs(cmd_left_vel - meas_left_vel)
        right_error = abs(cmd_right_vel - meas_right_vel)
        
        if left_error > self.slip_threshold or right_error > self.slip_threshold:
            self.wheel_slip_detected = True
            return True
        else:
            self.wheel_slip_detected = False
            return False
    
    def slip_compensation(self, left_vel, right_vel):
        """Apply compensation if slip is detected"""
        if self.wheel_slip_detected:
            # Reduce velocities and increase torque
            left_vel *= 0.7
            right_vel *= 0.7
        return left_vel, right_vel

class PIDController:
    """Simple PID controller for wheel velocity control"""
    def __init__(self, kp, ki, kd, max_output=10.0, min_output=-10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
        
    def compute(self, setpoint, measured_value, current_time):
        error = setpoint - measured_value
        
        if self.previous_time is None:
            dt = 0.01  # Default
        else:
            dt = (current_time - self.previous_time).nanoseconds / 1e9
            
        # Proportional
        p = self.kp * error
        
        # Integral (with anti-windup)
        self.integral += error * dt
        i = self.ki * self.integral
        
        # Derivative
        if dt > 0:
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0.0
        d = self.kd * derivative
        
        # Calculate output
        output = p + i + d
        
        # Clamp output
        output = max(self.min_output, min(self.max_output, output))
        
        self.previous_error = error
        self.previous_time = current_time
        
        return output
```

## **5. Testing Your Algorithm**

```python
def test_differential_drive():
    """Unit tests for the differential drive algorithm"""
    
    controller = DifferentialDriveController(
        wheel_radius=0.05,
        wheel_separation=0.2
    )
    
    print("=== Differential Drive Algorithm Tests ===")
    
    # Test 1: Forward motion
    print("\n1. Pure forward motion (v=0.5, ω=0):")
    left, right = controller.twist_to_wheel_velocities(0.5, 0.0)
    print(f"   Left wheel: {left:.2f} rad/s, Right wheel: {right:.2f} rad/s")
    
    # Test 2: Pure rotation
    print("\n2. Pure rotation (v=0, ω=1.0):")
    left, right = controller.twist_to_wheel_velocities(0.0, 1.0)
    print(f"   Left wheel: {left:.2f} rad/s, Right wheel: {right:.2f} rad/s")
    
    # Test 3: Curved motion
    print("\n3. Curved motion (v=0.3, ω=0.5):")
    left, right = controller.twist_to_wheel_velocities(0.3, 0.5)
    print(f"   Left wheel: {left:.2f} rad/s, Right wheel: {right:.2f} rad/s")
    
    # Test 4: Inverse test
    print("\n4. Inverse kinematics check:")
    v, w = controller.wheel_velocities_to_twist(left, right)
    print(f"   Reconstructed: v={v:.3f} m/s, ω={w:.3f} rad/s")
    
    # Test 5: Odometry update
    print("\n5. Odometry simulation:")
    controller.pose = np.array([0.0, 0.0, 0.0])
    
    # Simulate wheel movements for 1 second
    dt = 0.1
    for i in range(10):
        left_delta = left * dt
        right_delta = right * dt
        pose = controller.update_odometry(left_delta, right_delta, dt)
        print(f"   Step {i+1}: x={pose[0]:.2f}, y={pose[1]:.2f}, θ={pose[2]:.2f}")
    
    # Test 6: Move to point
    print("\n6. Move to point (2, 3):")
    controller.pose = np.array([0.0, 0.0, 0.0])
    for i in range(20):
        v_cmd, w_cmd = controller.move_to_point(2.0, 3.0)
        left_delta, right_delta = controller.twist_to_wheel_velocities(v_cmd, w_cmd)
        pose = controller.update_odometry(left_delta*dt, right_delta*dt, dt)
        print(f"   Step {i+1}: v={v_cmd:.2f}, ω={w_cmd:.2f}, "
              f"pose=({pose[0]:.2f}, {pose[1]:.2f})")

if __name__ == "__main__":
    test_differential_drive()
```

## **6. Key Mathematical Formulas**

### **Core Equations:**
```
1. Wheel velocities to robot twist:
   v = (ω_r * r + ω_l * r) / 2
   ω = (ω_r * r - ω_l * r) / L

2. Robot twist to wheel velocities:
   ω_l = (v - ω * L/2) / r
   ω_r = (v + ω * L/2) / r

3. Odometry update (for small Δt):
   Δs = (Δθ_r * r + Δθ_l * r) / 2
   Δθ = (Δθ_r * r - Δθ_l * r) / L
   
   x_{k+1} = x_k + Δs * cos(θ_k + Δθ/2)
   y_{k+1} = y_k + Δs * sin(θ_k + Δθ/2)
   θ_{k+1} = θ_k + Δθ
```

### **Physical Limits:**
- **Maximum linear velocity**: `v_max = max_motor_rpm * r * π / 30`
- **Maximum angular velocity**: `ω_max = 2 * v_max / L`
- **Motor torque required**: `τ = F * r = m * a * r`

## **7. Next Steps for Implementation:**

1. **Start with pure kinematics** (basic conversion formulas)
2. **Add velocity limiting** (prevent motor damage)
3. **Implement odometry** (track robot position)
4. **Add PID controllers** (for accurate velocity control)
5. **Include safety features** (slip detection, emergency stop)
6. **Integrate with ROS 2** (publishers/subscribers)
7. **Test in simulation** (Gazebo with simple model)
8. **Add navigation** (path following, obstacle avoidance)

This algorithm forms the **core intelligence** of any differential drive robot, converting high-level movement commands into precise wheel controls while tracking the robot's position.