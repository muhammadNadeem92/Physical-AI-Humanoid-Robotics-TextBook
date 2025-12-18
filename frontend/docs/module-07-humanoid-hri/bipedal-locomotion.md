# Chapter 2: Bipedal Locomotion & Balance

## Introduction

This chapter explores how humanoid robots walk without falling. You'll learn about gait cycles, the Zero Moment Point (ZMP) concept, static vs dynamic walking approaches, and fall detection and recovery strategies.

Bipedal locomotion is the most complex and distinctive aspect of humanoid robotics that differentiates it from other robot types. Understanding ZMP and balance control is essential for humanoid-specific applications and represents one of the most challenging problems in robotics.

By the end of this chapter, you'll understand why bipedal walking is challenging and how it's achieved, preparing you for the manipulation and interaction concepts in subsequent chapters.

## Core Concepts

### Gait Cycles

A gait cycle represents the sequence of movements that constitute a complete walking step, including stance and swing phases.

**Key Phases:**
- **Stance Phase**: When the foot is in contact with the ground (about 60% of gait cycle)
- **Swing Phase**: When the foot is off the ground moving forward (about 40% of gait cycle)
- **Double Support Phase**: When both feet are in contact with the ground (occurs twice per cycle)
- **Single Support Phase**: When only one foot is in contact with the ground

**Gait Parameters:**
- Step length: Distance between consecutive foot placements
- Stride length: Distance between successive placements of the same foot
- Cadence: Steps per unit time

### Zero Moment Point (ZMP)

The Zero Moment Point is a point where the net moment of the ground reaction force is zero. It's crucial for maintaining balance during locomotion.

**ZMP Properties:**
- Must remain within the support polygon for stable walking
- Represents the point where all forces and moments balance to zero
- Used as a control reference for walking robots

**Support Polygon:**
- The convex hull of all ground contact points
- For bipedal walking, typically defined by the feet's contact area
- ZMP must stay within this polygon for static stability

### Static vs Dynamic Walking

Different approaches to achieving bipedal locomotion with different stability characteristics.

**Static Walking:**
- Center of mass always kept within support polygon
- More stable but slower
- Typically uses tripod gait patterns
- Suitable for rough terrain

**Dynamic Walking:**
- Allows CoM to move outside support polygon temporarily
- More human-like and efficient
- Requires active balance control
- Faster but more complex control

### Fall Detection and Recovery

Strategies to detect when a robot is losing balance and methods to recover or minimize damage.

**Detection Methods:**
- Monitoring ZMP position relative to support polygon
- Analyzing joint angles and torques
- Using inertial measurement units (IMUs)
- Monitoring foot pressure sensors

**Recovery Strategies:**
- Step adjustment to expand support polygon
- Hip and trunk movements to shift CoM
- Arm movements for momentum control
- Controlled fall minimization if recovery fails

## Examples

### Example: Conceptual Walking Pipeline

```
Planning Phase:
1. Path planning to destination
2. Footstep planning considering terrain
3. Trajectory generation for CoM and ZMP

Execution Phase:
4. Balance control during stance
5. Swing leg trajectory control
6. Foot placement adjustment
7. Feedback control based on sensor data
```

### Example: Balance Control Loop Diagram

```
Sensors (IMU, joint encoders, force sensors)
         ↓
State Estimation (CoM position, ZMP, etc.)
         ↓
ZMP Controller (compares desired vs actual ZMP)
         ↓
Trajectory Generator (adjusts CoM, foot placement)
         ↓
Motor Commands (joint position/force control)
         ↓
Humanoid Robot (physical response)
         ↓
Sensors (feedback loop)
```

### Example: Locomotion State Transitions

```
Standing → Single Support → Double Support → Single Support → Standing
  ↑              ↓              ↓              ↓              ↓
Initial    Stance Phase   Double Support  Stance Phase   Final
State                     Phase                        State
```

## Summary & Key Takeaways

In this chapter, you learned about bipedal locomotion and balance:

- **Gait cycles** consist of stance and swing phases that repeat in a coordinated pattern
- **Zero Moment Point (ZMP)** is crucial for balance and must remain within the support polygon
- **Static walking** keeps CoM within support polygon (stable but slow) vs. **dynamic walking** (human-like but complex)
- **Fall detection and recovery** strategies are essential for safe operation

These concepts explain the fundamental challenges of bipedal locomotion and how humanoid robots achieve stable walking, building on the kinematic foundations from Chapter 1 and preparing for manipulation concepts in Chapter 3.