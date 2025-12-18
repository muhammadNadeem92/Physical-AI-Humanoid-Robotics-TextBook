# Chapter 2: Gazebo Simulation with ROS 2

## Introduction

In Chapter 1, you learned about digital twin concepts and their importance in humanoid robotics. Now we'll explore how to implement these concepts using Gazebo, the primary physics-based simulation environment for robotics. Gazebo provides realistic physics simulation, sensor modeling, and environment representation that makes it ideal for creating digital twins of humanoid robots.

Gazebo is widely used in the robotics community because of its excellent integration with ROS 2, realistic physics engines (ODE, Bullet, and DART), and comprehensive sensor simulation capabilities. When combined with ROS 2, Gazebo enables you to create complete digital twin environments where you can test control algorithms, sensor processing, and robot behaviors in a safe, virtual setting.

This chapter builds on the digital twin concepts you learned in Chapter 1 and shows you how to create practical simulation environments using Gazebo with ROS 2 integration. You'll learn about Gazebo's architecture, how to configure physics engines, and how to connect simulated sensors to ROS 2 topics for realistic testing.

By the end of this chapter, you'll be able to launch humanoid robot models in Gazebo, configure physics parameters, connect simulated sensors to ROS 2, and control robot joints using ROS 2 nodes. You'll also learn how to visualize sensor output in RViz to validate your simulation setup.

## Core Concepts

### Gazebo Architecture

Gazebo's architecture consists of several key components that work together to create realistic simulation environments:

**Gazebo Server**: The core physics simulation engine that handles physics calculations, collision detection, and sensor simulation. It runs in the background and can be controlled through various interfaces.

**Gazebo Client**: The graphical user interface that allows you to visualize the simulation, inspect robot states, and interact with the environment. Multiple clients can connect to the same server.

**Model Database**: A collection of pre-built robot models, objects, and environments that can be easily incorporated into simulations. This includes humanoid robots, sensors, and common objects.

**Plugin System**: Gazebo's extensible architecture allows custom plugins for sensors, controllers, and other functionality. This is how ROS 2 integration is achieved.

### Physics Engine Configuration

Gazebo supports multiple physics engines, each with different strengths:

**ODE (Open Dynamics Engine)**: The most commonly used engine, optimized for rigid body dynamics. It's stable and well-tested, making it ideal for most humanoid robotics applications.

**Bullet**: Offers more advanced contact handling and is better for complex collision scenarios. It's particularly useful when simulating interactions with deformable objects.

**DART (Dynamic Animation and Robotics Toolkit)**: Provides advanced features for articulated body simulation and is excellent for complex humanoid models.

Configuration parameters include:
- **Gravity**: Typically set to -9.8 m/s² for Earth-like conditions
- **Real Time Factor (RTF)**: Controls simulation speed relative to real time
- **Max Step Size**: Determines physics update frequency
- **Solver Iterations**: Affects physics stability and accuracy

### ROS 2 + Gazebo Bridges

The integration between ROS 2 and Gazebo is facilitated through several bridge components:

**gazebo_ros_pkgs**: A collection of ROS 2 packages that provide the interface between ROS 2 and Gazebo. These include plugins for spawning robots, publishing sensor data, and controlling joints.

**Robot State Publisher**: Publishes joint states and transforms that allow RViz to visualize the robot correctly in the simulation.

**Sensor Bridges**: Convert Gazebo sensor data to ROS 2 messages (sensor_msgs) for processing by ROS 2 nodes.

**Controller Bridges**: Allow ROS 2 controllers to send commands to simulated joints in Gazebo.

### Simulated Sensors and Actuators

Gazebo provides realistic simulation of various sensors and actuators:

**Camera Sensors**: Simulate RGB, depth, and stereo cameras with realistic noise and distortion models.

**IMU Sensors**: Model inertial measurement units with configurable noise parameters and bias drift.

**Force/Torque Sensors**: Simulate force and torque measurements at joints and end-effectors.

**LIDAR and Range Sensors**: Model various types of distance sensors with realistic beam patterns and noise.

**Joint Actuators**: Simulate motor dynamics, including position, velocity, and effort control with realistic response characteristics.

## Examples

### Example: Launching a Humanoid Proxy in Gazebo

Here's how to launch a humanoid robot model in Gazebo with ROS 2 integration:

```xml
<!-- launch_humanoid_gazebo.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name', default='humanoid_robot')

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        spawn_entity
    ])
```

### Example: Controlling Joints Using ROS 2 Topics

Once your humanoid robot is in Gazebo, you can control its joints using ROS 2 topics:

```python
#!/usr/bin/env python3
# joint_controller_example.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        # Timer to send commands at 50Hz
        self.timer = self.create_timer(0.02, self.send_joint_commands)
        self.time_counter = 0.0

    def send_joint_commands(self):
        msg = Float64MultiArray()

        # Create oscillating joint commands for demonstration
        joint_positions = []
        for i in range(6):  # Example: 6 joints
            position = 0.5 * math.sin(self.time_counter + i * 0.5)
            joint_positions.append(position)

        msg.data = joint_positions
        self.joint_pub.publish(msg)
        self.time_counter += 0.02

def main(args=None):
    rclpy.init(args=args)
    controller = JointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: Visualizing Sensor Output in RViz

To visualize sensor data from your simulated robot in RViz:

```yaml
# config/sensor_visualization.rviz
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Topic: /robot_description
    - Class: rviz_default_plugins/Image
      Name: CameraFeed
      Topic: /camera/image_raw
    - Class: rviz_default_plugins/PointCloud2
      Name: PointCloud
      Topic: /lidar/points
    - Class: rviz_default_plugins/TF
      Name: TF
      Show Arrows: true
      Show Names: true

Global Options:
  Fixed Frame: odom
  Frame Rate: 30
```

## Summary & Key Takeaways

In this chapter, you've learned how to implement digital twin concepts using Gazebo simulation with ROS 2 integration:

- **Gazebo architecture** includes server, client, model database, and plugin system components that work together to create realistic simulations
- **Physics engine configuration** allows you to tune simulation parameters for different use cases and accuracy requirements
- **ROS 2 bridges** connect Gazebo to the ROS 2 ecosystem, enabling seamless integration of simulated and real-world components
- **Simulated sensors and actuators** provide realistic modeling of physical hardware for comprehensive testing

You've seen practical examples of launching humanoid robots in Gazebo, controlling joints through ROS 2 topics, and visualizing sensor output in RViz. These capabilities form the foundation for creating comprehensive digital twin environments that enable safe, effective testing of humanoid robot behaviors.

In the next chapter, we'll explore Unity as an alternative simulation environment focused on advanced visualization and human-robot interaction, complementing the physics-focused approach of Gazebo.