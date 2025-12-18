# Style Guide: Module 7 - Humanoid Systems & Human–Robot Interaction (HRI)

## Terminology Standards

### Core Terms
- **Humanoid Robot**: A robot with human-like form factor including legs for bipedal locomotion, arms for manipulation, and head for interaction
- **Kinematic Chain**: A series of rigid bodies connected by joints that define the movement relationships between different parts of the robot
- **Center of Mass (CoM)**: The point where the total mass of the robot can be considered to be concentrated, critical for balance and stability
- **Zero Moment Point (ZMP)**: A point where the net moment of the ground reaction force is zero, used for maintaining balance during locomotion
- **Gait Cycle**: The sequence of movements that constitute a complete walking step, including stance and swing phases
- **Grasp Type**: Classification of how a robot hand grips an object (power grasp for stability, precision grasp for fine control)
- **End-Effector**: The tool or device at the end of a robot arm designed to interact with the environment
- **Proxemics**: The study of personal space and distance in human interactions, applied to robot safety zones

### Technical Concepts
- **Degrees of Freedom (DoF)**: The number of independent movements a robot can make
- **Forward Kinematics**: Calculating end-effector position from joint angles
- **Inverse Kinematics**: Calculating joint angles needed to reach a position
- **Static Walking**: Walking where the center of mass is always within the support polygon
- **Dynamic Walking**: Walking that involves periods where the center of mass moves outside the support polygon
- **Power Grasp**: A grasp type focused on stability and force application
- **Precision Grasp**: A grasp type focused on fine control and dexterity

## Writing Style Guidelines

### Educational Focus
- Emphasize conceptual understanding over implementation details
- Focus on safety, predictability, and human comfort throughout all content
- Use diagrams for motion and interaction concepts rather than text descriptions
- Avoid low-level motor controller implementations
- Connect perception to manipulation in practical examples

### Content Structure
- Follow the standard template: Introduction → Concepts → Examples → Summary
- Explain technical concepts before introducing terminology
- Use beginner-friendly language while maintaining technical accuracy
- Ensure each chapter can be understood independently
- Maintain concept-first approach with diagrams preferred over text

### Safety Integration
- Integrate safety considerations throughout all content rather than as a separate section
- Emphasize risk assessment in locomotion and manipulation discussions
- Prioritize human comfort in interaction design
- Address ethical considerations in human-robot interaction

### Diagram and Visualization Standards
- Use text-based diagrams (ASCII art, mermaid diagrams) that can be embedded in markdown
- Ensure diagrams clearly illustrate motion and interaction concepts
- Include kinematic chain diagrams showing joint hierarchy
- Provide conceptual pipeline diagrams for locomotion and manipulation
- Use flow diagrams for human-robot interaction scenarios

## Cross-Module References
- Connect to Module 5 (VLA Systems) concepts where relevant
- Reference proxy robot trade-offs (90% of concepts transfer, except bipedal balance)
- Link to capstone autonomous humanoid system where appropriate
- Maintain consistency with previous module terminology

## RAG System Alignment
- Structure content with semantic chunking by concept
- Ensure content can be used by chatbot to answer specific questions
- Include key questions that the chatbot should be able to answer:
  - How do humanoids walk?
  - Why are humanoids unstable?
  - What is ZMP?
  - How do robots interact safely with humans?