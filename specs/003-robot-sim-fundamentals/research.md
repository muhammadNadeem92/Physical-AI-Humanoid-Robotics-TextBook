# Research Summary: Module 3: Robot Modeling & Simulation Fundamentals

## Decision: Focus on conceptual understanding with math-light approach
**Rationale**: The specification explicitly states "Math-light: Intuition first, formulas optional" and "Focus: Understanding behavior, not configuration". This approach makes the content accessible to beginners while still providing valuable knowledge about simulation concepts.

**Alternatives considered**:
- Detailed mathematical approach with full kinematics equations - rejected as it would make the content inaccessible to beginners
- Pure conceptual approach with no equations - rejected as some basic understanding requires minimal mathematical concepts

## Decision: Python for conceptual examples
**Rationale**: Since Module 2 used Python rclpy, maintaining consistency with Python for conceptual examples will help learners build on their existing knowledge. Python is also widely used in robotics and is beginner-friendly.

**Alternatives considered**:
- Using C++ examples - rejected as it would introduce a new programming language unnecessarily
- Using pseudocode only - rejected as concrete examples help understanding

## Decision: Include multiple physics engines (ODE, Bullet, PhysX)
**Rationale**: The specification specifically mentions these three physics engines. Understanding the differences between them is important for learners to make informed decisions when working with different simulation environments.

**Alternatives considered**:
- Focusing on only one physics engine - rejected as it would limit learner knowledge
- Including additional engines like Mujoco - rejected as it would exceed scope and introduce proprietary software

## Decision: Visual diagrams over equations
**Rationale**: The specification explicitly states "Visuals: Diagrams preferred over equations". This approach helps learners visualize concepts without getting bogged down in mathematical details.

**Examples to include**:
- Kinematic chain diagrams
- URDF/SDF comparison visuals
- Sensor model illustrations
- Physics simulation failure examples

## Decision: Bridge between ROS 2 and full simulators
**Rationale**: The specification clearly states this module "bridges the gap between ROS 2 control (Module 2) and full simulation environments (Module 4)". This positioning is essential for the learning progression.

## Decision: Emphasis on simulation stability and debugging
**Rationale**: The specification includes a "Debug checklist for unstable simulations as mandatory example", highlighting the importance of understanding common simulation issues.

**Key topics to cover**:
- Common simulation failures (falling robots, jitter, tunneling)
- Best practices for stable simulations
- Understanding time-step limitations
- Collision mesh considerations

## Decision: Sensor modeling with noise characteristics
**Rationale**: The specification emphasizes understanding sensor noise, latency, and drift, which are crucial for realistic simulation and perception pipeline preparation.

**Key concepts to explain**:
- Camera models in simulation
- LiDAR and depth sensor simulation
- IMU and encoder simulation
- Noise modeling approaches
- Impact of sensor noise on perception systems