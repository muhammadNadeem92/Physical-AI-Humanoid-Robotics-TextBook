# Chapter 1: Humanoid Kinematics & Dynamics

## Introduction

This chapter provides the mechanical foundation of humanoid robots. You'll learn about the fundamental concepts that enable humanoid robots to move and balance, including degrees of freedom, forward and inverse kinematics, and center of mass considerations.

Understanding these mechanical principles is essential for comprehending all other aspects of humanoid robotics. Without understanding the mechanical constraints and capabilities, you cannot properly design locomotion, manipulation, or interaction systems.

By the end of this chapter, you'll understand how humanoid robots achieve their human-like form and movement capabilities, preparing you for the more complex topics in subsequent chapters.

## Core Concepts

### Degrees of Freedom (DoF)

Degrees of freedom (DoF) represent the number of independent movements a robot can make. For humanoid robots, DoF determine the range of possible motions and significantly impact the robot's capabilities.

**Key Aspects:**
- Each joint typically contributes 1-3 degrees of freedom depending on its type
- The total DoF of a humanoid robot affects its dexterity and control complexity
- Humanoid robots typically have 30+ DoF to achieve human-like movement capabilities

### Forward & Inverse Kinematics

Kinematics describes the relationship between joint angles and the position of robot parts without considering forces.

**Forward Kinematics:**
- Calculates end-effector position from joint angles
- Used to determine where a robot's hand or foot will be based on current joint configuration
- Provides predictable positioning for planning

**Inverse Kinematics:**
- Calculates joint angles needed to reach a desired position
- Critical for motion planning and task execution
- Often involves solving complex mathematical equations

### Center of Mass (CoM)

The center of mass is the point where the total mass of the robot can be considered to be concentrated. It's critical for balance and stability.

**Key Properties:**
- Affects stability during locomotion and manipulation
- Must be kept within the support polygon for static balance
- Continuously shifts during dynamic movements

### Dynamics vs Statics

Dynamics involves the study of forces and their effects on motion, while statics deals with equilibrium of bodies at rest.

**Statics:**
- Relevant for stationary poses and slow movements
- Focuses on maintaining balance without motion
- Simpler to analyze but less realistic

**Dynamics:**
- Accounts for forces, torques, and accelerations
- Critical for fast movements and dynamic locomotion
- More complex but more realistic

## Examples

### Example: Kinematic Chain of a Humanoid Leg

The kinematic chain of a humanoid leg typically follows this structure:
```
Hip Joint → Thigh → Knee Joint → Shin → Ankle Joint → Foot
```

Each joint contributes specific degrees of freedom:
- Hip: 3 DoF (flexion/extension, abduction/adduction, rotation)
- Knee: 1 DoF (flexion/extension)
- Ankle: 2 DoF (dorsiflexion/plantarflexion, inversion/eversion)

### Example: Visual Diagram Showing Joint Hierarchy

```
                    [HEAD]
                      |
                [TRUNK/TORSO]
                     / \
                   /     \
              [LEFT ARM] [RIGHT ARM]
                 |         |
            [LEFT LEG]  [RIGHT LEG]
                 |
            [LEFT FOOT]
```

### Example: Center of Mass Calculation

For a simplified humanoid model:
- Torso: 40% of total mass
- Head: 7% of total mass
- Arms: 13% each (26% total)
- Legs: 18% each (36% total)

The CoM position is calculated as the weighted average of all body segment positions.

## Summary & Key Takeaways

In this chapter, you learned about the mechanical foundation of humanoid robots:

- **Degrees of freedom** determine the range of possible movements for each joint
- **Forward kinematics** calculates positions from joint angles, while **inverse kinematics** calculates joint angles from desired positions
- **Center of mass** is critical for balance and must be managed during all movements
- **Dynamics vs statics** represents the difference between considering forces versus just equilibrium

These foundational concepts are essential for understanding how humanoid robots move and balance, providing the basis for the locomotion, manipulation, and interaction concepts covered in subsequent chapters.