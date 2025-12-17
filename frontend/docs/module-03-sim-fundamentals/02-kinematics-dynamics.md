# Chapter 2: Kinematics & Dynamics for Humanoids

## Introduction

In Chapter 1, you learned about robot description models (URDF and SDF) that define the physical structure of robots. Now, let's explore how these robots move and the physical forces that govern their motion. Understanding kinematics and dynamics is crucial for controlling humanoid robots, as it allows us to predict where robot parts will be in space and how they'll respond to forces and torques.

Kinematics is the study of motion without considering the forces that cause it - essentially answering "where will the robot's foot be if I move its joints to these angles?" Dynamics, on the other hand, deals with the forces and torques that cause motion - answering questions like "what forces are needed to make the robot take a step?"

For humanoid robots, these concepts become particularly important because of their complex structure with multiple limbs and the need to maintain balance while moving. Unlike wheeled robots that can be controlled with simple forward and turn commands, humanoid robots require sophisticated understanding of how joint movements affect the position of each body part and the overall system.

These kinematic and dynamic principles connect directly to the ROS 2 concepts you learned in Module 2. In ROS 2, the `tf2` (Transform Library) system uses forward kinematics to publish coordinate transforms between different parts of the robot, allowing different nodes to understand where sensors and effectors are located in space. When you worked with topics, services, and actions in Module 2, those communication mechanisms often carry the kinematic and dynamic data needed to control robot movement. The robot description models you learned about in Chapter 1 provide the structural foundation for these kinematic calculations.

By the end of this chapter, you'll understand the fundamental concepts of forward kinematics, inverse kinematics, and dynamics as they apply to humanoid robots. You'll learn about links and joints, how to think about kinematic chains, and get a conceptual understanding of how robots solve the complex problems of movement and balance without getting bogged down in heavy mathematics.

## Core Concepts

### Links and Joints

In robotics, a robot is modeled as a collection of rigid bodies called "links" connected by "joints." Think of your own body: your upper arm, forearm, and hand are links, while your shoulder, elbow, and wrist are joints. In a humanoid robot, the torso might be one link, connected to arm links through joint actuators.

**Links** are the rigid parts of the robot - they don't change shape during normal operation. Each link has physical properties like mass, center of mass, and inertia that affect how it responds to forces.

**Joints** are the connections between links that allow relative motion. Common joint types include:
- **Revolute joints**: Like your elbow, allowing rotation around a single axis
- **Prismatic joints**: Like a sliding drawer, allowing linear motion
- **Ball joints**: Like your shoulder, allowing rotation around multiple axes

In humanoid robots, most joints are revolute, allowing the robot to bend at knee, elbow, and hip positions. The combination of links and joints creates what we call a "kinematic chain."

### Forward Kinematics

Forward kinematics answers the question: "Given the current joint angles, where are all the parts of the robot?" It's like knowing that your shoulder is at 30 degrees, your elbow at 90 degrees, and being able to calculate where your hand is positioned in 3D space.

For a humanoid robot, forward kinematics allows us to:
- Visualize the robot's pose in simulation
- Understand where each part of the robot is located
- Plan movements by predicting where the robot will be after joint changes
- Detect potential collisions or self-collisions

The process is deterministic: if you know the joint angles and the robot's structure (from its URDF/SDF), you can precisely calculate where every point on the robot is located. This is why forward kinematics is generally straightforward to compute.

### Inverse Kinematics (IK)

Inverse kinematics is the reverse problem: "Where should I move my joints to place my hand at this specific location?" This is much more complex than forward kinematics because there may be multiple (or infinite) ways to achieve the same end position, or sometimes no solution at all.

For humanoid robots, inverse kinematics is essential for:
- Reaching for objects at specific locations
- Maintaining balance by adjusting the center of mass
- Planning walking patterns where feet need to land at specific positions
- Coordinating multiple limbs to work together

A simple example: if a humanoid robot wants to touch a ball floating in front of it, inverse kinematics calculates how to position all the joints in the arm to make that contact happen. The solution isn't always unique - the robot could reach with a bent elbow or a straight arm, for instance.

### Dynamics Concepts: Mass, Inertia, and Center of Gravity

While kinematics deals with motion, dynamics deals with the forces that cause motion. For humanoid robots, understanding dynamics is crucial for stable movement and balance.

**Mass** is straightforward - it's how much matter is in each robot part. Heavier parts require more force to move.

**Inertia** describes how resistant a part is to changes in rotation. A long robot arm has higher inertia than a short one, meaning it requires more torque to start or stop its rotation.

**Center of Gravity (CoG)** is the average location of all the mass in the robot. For a humanoid robot to maintain balance, its center of gravity must remain within its support base (typically the area between its feet). This is why humanoid robots need to carefully control their movements to avoid falling over.

Dynamics also involves understanding how forces propagate through the robot's structure. When a humanoid robot takes a step, the impact force travels up through its leg, affecting the entire body. Control systems must account for these dynamic effects to maintain stability.

## Examples

### Simple Kinematic Chain Diagram

```
    Hip (Base)
       |
       | (Thigh Link)
       |
    Knee Joint
       |
       | (Shin Link)
       |
    Ankle Joint
       |
       | (Foot Link)
       |
    Ground Contact
```

This simple leg model shows a kinematic chain: a series of links connected by joints. In forward kinematics, if we know the angles at the hip and knee, we can calculate where the foot is positioned. In inverse kinematics, if we want the foot at a specific location, we can calculate the required hip and knee angles.

For humanoid robots, multiple kinematic chains work together - two legs for walking, two arms for manipulation, all connected through the torso. The challenge is coordinating these chains while maintaining overall balance.

### Python-Based Conceptual IK Explanation

Here's a conceptual understanding of how inverse kinematics works in Python-like pseudocode:

```python
def inverse_kinematics(target_position, current_joint_angles):
    """
    Conceptual IK algorithm for a simple 2-joint arm
    This is pseudocode to demonstrate the concept, not actual implementation
    """
    # Start with current joint angles
    current_angles = current_joint_angles.copy()

    # Calculate current end-effector position (forward kinematics)
    current_position = forward_kinematics(current_angles)

    # Iteratively adjust joint angles to move closer to target
    while distance(current_position, target_position) > tolerance:
        # Calculate how to move each joint to get closer to target
        joint_adjustments = calculate_joint_changes(target_position, current_position)

        # Apply small adjustments to joint angles
        current_angles = adjust_angles(current_angles, joint_adjustments)

        # Recalculate where the end-effector is now
        current_position = forward_kinematics(current_angles)

    return current_angles

# For a humanoid robot, this process would be much more complex
# involving multiple limbs and balance constraints
```

In practice, humanoid robots use sophisticated IK solvers that consider:
- Multiple simultaneous goals (reach with hand, keep head level, maintain balance)
- Joint limits (can't bend joints beyond their physical range)
- Balance constraints (don't fall over while moving)
- Collision avoidance (don't hit other parts of the body)

### Visual Diagrams for Understanding Motion

```
Humanoid Standing Position:
        Head
         |
      Torso
     /     \
   Arm    Arm
    |       |
   Hand   Hand

      Legs
     /    \
   Hip    Hip
    |      |
  Knee   Knee
    |      |
  Ankle  Ankle
    |      |
   Foot   Foot
```

When a humanoid robot moves, these kinematic chains must work in coordination. For example, when reaching forward with one arm, the robot might need to shift its weight or move the other arm to maintain balance - this is the interplay between kinematics and dynamics.

## Summary & Key Takeaways

In this chapter, you've learned about the fundamental concepts that govern how robots move:

- **Links and joints** form the basic structure of robots, with links being rigid parts and joints allowing relative motion between them
- **Forward kinematics** calculates where robot parts are located based on joint angles - this is deterministic and straightforward
- **Inverse kinematics** calculates the joint angles needed to place robot parts at specific locations - this is complex with potentially multiple solutions
- **Dynamics** deals with the forces and torques that cause motion, including mass, inertia, and center of gravity considerations
- For humanoid robots, these concepts become particularly complex due to the need for balance and coordination between multiple limbs

Understanding these concepts is essential for working with humanoid robots because they form the foundation for all movement and control. Whether you're programming a robot to walk, reach for objects, or maintain balance, you're working with the principles of kinematics and dynamics.

In the next chapter, we'll explore physics engines and simulation limits, learning how these motion principles are implemented in simulation environments and what challenges arise when trying to accurately model physical behavior in software.