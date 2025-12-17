# Chapter 1: Robot Description Models (URDF vs SDF)

## Introduction

Welcome to the world of robot modeling and simulation fundamentals! In Module 2, you learned how ROS 2 nodes communicate with each other through topics, services, and actions, and how to describe robot structures using URDF. Now we'll explore how robots themselves are represented in simulation environments - the foundation for all digital twins and virtual testing environments. These robot description formats connect directly to the ROS 2 communication framework you learned in Module 2.

Robot description models are essential for robotics development because they provide a standardized way to describe a robot's physical properties: its links (rigid parts), joints (connections between parts), sensors, and other components. These descriptions are crucial for simulation, visualization, motion planning, and control. In the context of Physical AI and humanoid robotics, accurate robot models enable us to test and validate our control algorithms in safe, virtual environments before deploying them on real hardware.

By the end of this chapter, you'll understand the two primary formats for describing robots in simulation: URDF (Unified Robot Description Format) and SDF (Simulation Description Format). You'll learn their strengths and limitations, when to use each format, and how they relate to ROS 2. You'll also see practical examples comparing these formats and a simple humanoid limb model.

## Core Concepts

### URDF (Unified Robot Description Format)

URDF (Unified Robot Description Format) is an XML-based format that has become the standard for representing robot models in the ROS ecosystem. It was designed primarily for kinematic and dynamic properties of robots, making it ideal for:

- **Kinematic chains**: Describing how links are connected through joints
- **Dynamic properties**: Specifying mass, center of mass, and inertia tensors
- **Visual and collision geometry**: Defining how the robot looks and how collisions are detected
- **Transmission information**: Describing how actuators connect to joints

**Strengths of URDF:**
- Deep integration with ROS and ROS 2 tools
- Extensive community support and documentation
- Standard format for robot models in ROS distributions
- Direct integration with robot_state_publisher and other ROS tools

**Limitations of URDF:**
- Limited simulation-specific features
- No support for multiple robots in a single file
- No world description capabilities
- Less flexibility for complex simulation scenarios

### SDF (Simulation Description Format)

SDF (Simulation Description Format) is also XML-based but was designed specifically for simulation environments, particularly Gazebo. It extends the capabilities of robot description by including:

- **World description**: Ability to describe entire environments, not just robots
- **Multiple robots**: Support for multiple robots in a single file
- **Sensor specifications**: Detailed sensor models including noise characteristics
- **Simulation parameters**: Physics engine settings, lighting, and other simulation-specific properties

**SDF Features and Extensions:**
- More comprehensive simulation support than URDF
- Can describe entire scenes with robots, objects, and environments
- Better support for complex sensor models
- Physics engine configuration parameters

### When to Use URDF vs SDF

**Use URDF when:**
- Working primarily within the ROS/ROS 2 ecosystem
- Need tight integration with ROS tools (robot_state_publisher, moveit, etc.)
- Creating robot models for kinematic and dynamic analysis
- Developing for real robot deployment (URDF is more standard for this)

**Use SDF when:**
- Creating complex simulation scenarios with multiple robots
- Need advanced sensor simulation capabilities
- Working extensively with Gazebo or other simulators that prefer SDF
- Describing entire environments with robots and objects
- Need simulation-specific features not available in URDF

### Relationship with ROS 2

URDF has deep integration with ROS 2 through several packages:
- `robot_state_publisher`: Publishes transforms based on joint states and URDF
- `joint_state_publisher`: Publishes joint states for visualization
- `rviz2`: Uses URDF for robot visualization
- `moveit`: Uses URDF for motion planning

SDF can be used with ROS 2 through Gazebo integration, but requires additional tools to bridge the gap between simulation and ROS 2 control systems.

## Examples

### Side-by-Side URDF and SDF Comparison

Here's a simple robot model example in both formats:

**URDF Example:**
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- First joint -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0025"/>
    </inertial>
  </link>
</robot>
```

**SDF Example:**
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="simple_robot">
    <!-- Base link -->
    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.2</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.2</length>
          </cylinder>
        </geometry>
      </collision>
      <inertial>
        <mass>1</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.02</izz>
        </inertia>
      </inertial>
    </link>

    <!-- First joint -->
    <joint name="joint1" type="revolute">
      <parent>base_link</parent>
      <child>arm_link</child>
      <pose>0 0 0.1 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>100</effort>
          <velocity>1</velocity>
        </limit>
      </axis>
    </joint>

    <!-- Arm link -->
    <link name="arm_link">
      <pose>0 0 0.2 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.05 0.05 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0 0 1</ambient>
          <diffuse>0.8 0 0 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.05 0.05 0.2</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.0025</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

### Simple Humanoid Limb Model

Here's a simple humanoid leg model in URDF format:

```xml
<?xml version="1.0"?>
<robot name="humanoid_leg">
  <!-- Hip (pelvis) link -->
  <link name="hip">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.15"/>
    </inertial>
  </link>

  <!-- Hip joint (3 DOF for more realistic movement) -->
  <joint name="hip_yaw" type="revolute">
    <parent link="hip"/>
    <child link="thigh"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.78" upper="0.78" effort="200" velocity="2"/>
  </joint>

  <joint name="hip_pitch" type="revolute">
    <parent link="hip"/>
    <child link="thigh"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.5" effort="200" velocity="2"/>
  </joint>

  <joint name="hip_roll" type="revolute">
    <parent link="hip"/>
    <child link="thigh"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="200" velocity="2"/>
  </joint>

  <!-- Thigh link -->
  <link name="thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.08"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Knee joint -->
  <joint name="knee" type="revolute">
    <parent link="thigh"/>
    <child link="shank"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="200" velocity="2"/>
  </joint>

  <!-- Shank (lower leg) link -->
  <link name="shank">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.07"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.07"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.008"/>
    </inertial>
  </link>

  <!-- Ankle joint -->
  <joint name="ankle" type="revolute">
    <parent link="shank"/>
    <child link="foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="100" velocity="1"/>
  </joint>

  <!-- Foot link -->
  <link name="foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.006"/>
    </inertial>
  </link>
</robot>
```

### Diagrams for URDF/SDF Comparison

```mermaid
graph TB
    A[URDF: Robot-centric] --> B[ROS Ecosystem Integration]
    A --> C[Simple Kinematic Chains]
    A --> D[Standard in ROS Distributions]

    E[SDF: Simulation-centric] --> F[World Description]
    E --> G[Advanced Sensor Models]
    E --> H[Multiple Robots Support]

    I[Choice Factors] --> J{Application Type}
    J -->|ROS Integration| A
    J -->|Simulation Focus| E
    J -->|Real Robot Control| A
    J -->|Complex Simulation| E

    style A fill:#e1f5fe
    style E fill:#f3e5f5
    style B fill:#e8f5e8
    style F fill:#e8f5e8

## Summary & Key Takeaways

In this chapter, you've learned about the fundamental formats for describing robots in simulation:

- **URDF (Unified Robot Description Format)** is ideal for ROS/ROS 2 integration and real robot deployment
- **SDF (Simulation Description Format)** provides enhanced simulation capabilities for complex scenarios
- Both formats describe robot links, joints, and physical properties but serve different purposes
- URDF excels in ROS ecosystem integration while SDF excels in simulation-specific features
- The choice between URDF and SDF depends on your application: use URDF for ROS integration and real robot control, SDF for complex simulation scenarios

You've seen practical examples of both formats and learned when to use each one. These robot description models form the foundation for all simulation work, enabling you to create accurate digital twins of your robotic systems.

In the next chapter, we'll explore how these robot models move through kinematics and dynamics, learning how to calculate robot motion and understand the physical forces that affect robotic systems.