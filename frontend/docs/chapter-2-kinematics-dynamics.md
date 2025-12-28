---
id: chapter-2-kinematics-dynamics
title: "Chapter 2: Robot Kinematics and Dynamics"
---

## 2.1 Introduction to Robot Kinematics

Robot kinematics is the study of robot motion without considering the forces or torques that cause the motion. It primarily deals with the geometric relationships between the joint angles of a robot and the position and orientation of its end-effector (e.g., a gripper or tool). Kinematics is fundamental for understanding how to command a robot to reach a specific point in space.

**Key Concepts:**
*   **Forward Kinematics:** Calculating the end-effector's position and orientation given the robot's joint angles.
*   **Inverse Kinematics:** Calculating the required joint angles to achieve a desired end-effector position and orientation.
*   **Degrees of Freedom (DoF):** The number of independent parameters that define the configuration of a robot.

**Diagram: 2-DOF Robotic Arm**

```mermaid
graph TD
    A[Base] -->|Joint 1 (θ1)| B(Link 1)
    B -->|Joint 2 (θ2)| C(Link 2)
    C --> D[End-Effector (X, Y)]
```

## 2.2 Forward Kinematics

Forward kinematics involves using a robot's geometric parameters (link lengths, joint offsets) and its current joint angles to determine the spatial position and orientation of its end-effector. The Denavit-Hartenberg (DH) convention is a widely used method for systematically assigning coordinate frames to robot links and deriving kinematic equations.

**Mathematical Representation (Simplified 2D Example):**
For a 2-DOF planar arm with link lengths L1 and L2, and joint angles θ1 and θ2:

```latex
X = L1 \cos(\theta_1) + L2 \cos(\theta_1 + \theta_2)
Y = L1 \sin(\theta_1) + L2 \sin(\theta_1 + \theta_2)
```

## 2.3 Inverse Kinematics

Inverse kinematics is more challenging than forward kinematics as it often involves solving non-linear equations, potentially leading to multiple solutions, no solutions, or singularities. It's crucial for path planning, where the robot needs to follow a trajectory defined in Cartesian space.

**Common Approaches:**
*   **Analytical Solutions:** Closed-form expressions for simple manipulators (rare for complex robots).
*   **Numerical Solutions:** Iterative methods (e.g., Jacobian transpose, pseudo-inverse) for complex robots.
*   **Look-up Tables/Machine Learning:** Pre-computed solutions or learned mappings.

## 2.4 Robot Dynamics

Robot dynamics deals with the relationship between the forces and torques acting on a robot and the resulting motion. It considers mass, inertia, gravity, and external forces. Dynamics is essential for designing controllers that can move the robot precisely and efficiently, especially in high-speed or high-payload applications.

**Key Equations (Conceptual):**

```latex
\tau = M(\theta) \ddot{\theta} + C(\theta, \dot{\theta}) \dot{\theta} + G(\theta)
```
Where:
*   `$\tau$`: Joint torques
*   `$M(\theta)$`: Mass matrix (inertia matrix)
*   `$C(\theta, \dot{\theta})$`: Coriolis and centrifugal forces
*   `$G(\theta)$`: Gravitational forces
*   `$\theta, \dot{\theta}, \ddot{\theta}$`: Joint position, velocity, and acceleration

## 2.5 Simulation Steps: Manipulator Control

Simulating a robot manipulator often involves:

1.  **URDF/SDF Model:** Defining the robot's physical properties (mass, inertia) and collision geometry.
2.  **Inverse Kinematics Solver:** Implementing an IK solver to translate desired end-effector poses into joint commands.
3.  **Joint Controllers:** Using PID (Proportional-Integral-Derivative) controllers for each joint to track desired positions/velocities.
4.  **Force/Torque Application:** Applying calculated torques to joints in the simulator.

**ROS2 Code Example: Inverse Kinematics with MoveIt**

MoveIt is a powerful ROS2 package for robotic manipulation, including IK. This example shows a high-level approach.

```python
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

class MoveItIKClient(Node):
    def __init__(self):
        super().__init__('moveit_ik_client')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, target_pose: PoseStamped):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "[your_robot_arm_group_name]" # e.g., "panda_arm"
        req.ik_request.pose_stamped = target_pose
        req.ik_request.timeout.sec = 1.0

        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    ik_client = MoveItIKClient()

    target_pose = PoseStamped()
    target_pose.header.frame_id = "[your_robot_base_frame]" # e.g., "panda_link0"
    target_pose.pose.position.x = 0.5
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.5
    target_pose.pose.orientation.w = 1.0 # Identity orientation

    response = ik_client.send_request(target_pose)
    if response.error_code.val == response.error_code.SUCCESS:
        ik_client.get_logger().info(f'IK successful: {response.solution.joint_state.position}')
    else:
        ik_client.get_logger().warn(f'IK failed with error code: {response.error_code.val}')

    ik_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Explanation*: This ROS2 client calls the `/compute_ik` service provided by MoveIt to find joint configurations (`JointState`) that correspond to a desired end-effector `PoseStamped`. You would replace `[your_robot_arm_group_name]` and `[your_robot_base_frame]` with your robot's specific configuration.

## Chapter Summary

Chapter 2 delved into the fundamental concepts of robot kinematics and dynamics. We explored forward and inverse kinematics for determining robot poses and joint angles, respectively, and introduced the governing equations of robot dynamics. The chapter also provided insights into simulating manipulators and a high-level ROS2 example demonstrating inverse kinematics with MoveIt, crucial for programming robot movements in complex environments.
