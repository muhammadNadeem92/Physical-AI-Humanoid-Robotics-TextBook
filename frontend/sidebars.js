// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Introduction to Physical AI & Humanoid Robotics',
      items: [
        'module-01-introduction/what-is-physical-ai',
        'module-01-introduction/embodied-intelligence',
        'module-01-introduction/introduction-to-humanoid-robotics',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: ROS 2 — The Robotic Nervous System',
      items: [
        'module-02-ros2/architecture-setup',
        'module-02-ros2/nodes-topics-messages',
        'module-02-ros2/services-actions-launch',
        'module-02-ros2/robot-description-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Robot Modeling & Simulation Fundamentals',
      items: [
        'module-03-sim-fundamentals/robot-description-models',
        'module-03-sim-fundamentals/kinematics-dynamics',
        'module-03-sim-fundamentals/physics-engines-simulation',
        'module-03-sim-fundamentals/sensor-modeling-noise',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: The Digital Twin: Gazebo & Unity Simulation',
      items: [
        'module-04-digital-twin/digital-twin-concepts',
        'module-04-digital-twin/gazebo-ros2',
        'module-04-digital-twin/unity-hri',
        'module-04-digital-twin/sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: The AI Robot Brain: NVIDIA Isaac Platform',
      items: [
        'module-05-isaac-ai-brain/isaac-architecture',
        'module-05-isaac-ai-brain/synthetic-data',
        'module-05-isaac-ai-brain/perception-navigation',
        'module-05-isaac-ai-brain/learning-sim2real',
      ],
    },
    {
      type: 'category',
      label: 'Module 6: Vision–Language–Action (VLA) Systems',
      items: [
        'module-06-vla-systems/vla-foundations',
        'module-06-vla-systems/voice-language',
        'module-06-vla-systems/llm-planning',
        'module-06-vla-systems/action-safety',
      ],
    },
    {
      type: 'category',
      label: 'Module 7: Humanoid Systems & Human–Robot Interaction (HRI)',
      items: [
        'module-07-humanoid-hri/kinematics-dynamics',
        'module-07-humanoid-hri/bipedal-locomotion',
        'module-07-humanoid-hri/manipulation-grasping',
        'module-07-humanoid-hri/human-robot-interaction',
      ],
    },
    {
      type: 'category',
      label: 'Module 8: Capstone - The Autonomous Humanoid System',
      items: [
        'module-08-capstone-autonomous-humanoid/system-architecture',
        'module-08-capstone-autonomous-humanoid/voice-to-plan',
        'module-08-capstone-autonomous-humanoid/perception-grounding',
        'module-08-capstone-autonomous-humanoid/action-navigation',
        'module-08-capstone-autonomous-humanoid/deployment-evaluation',
      ],
    },
    // Add more modules as they are created
  ],
};

module.exports = sidebars;