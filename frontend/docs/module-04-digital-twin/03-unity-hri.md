# Chapter 3: Unity for Visualization & Human–Robot Interaction

## Introduction

In the previous chapters, you learned about digital twin concepts and how to implement physics-based simulation using Gazebo with ROS 2 integration. Now we'll explore Unity, a powerful visualization platform that excels in creating engaging, realistic environments for human-robot interaction (HRI) scenarios.

While Gazebo focuses on physics accuracy and realistic simulation of robot behaviors, Unity specializes in high-quality graphics, immersive environments, and intuitive human-robot interaction design. Unity provides an excellent complement to physics-focused simulators by offering advanced visualization capabilities and user-friendly interfaces for testing interaction scenarios.

Unity's strength lies in its ability to create photorealistic environments and intuitive interfaces that help researchers and developers visualize robot behaviors in contexts that closely resemble real-world deployment scenarios. This is particularly valuable for humanoid robots, which are designed to operate in human environments and interact with humans.

This chapter explores why Unity is used in robotics, the trade-offs between graphics and physics accuracy, ROS-Unity communication approaches, and how to design effective human-robot interaction scenarios. You'll learn how Unity can serve as a visualization-only digital twin or be integrated with physics engines for more comprehensive simulation.

By the end of this chapter, you'll understand when to use Unity for robotics applications, how to create engaging visualization environments, and how to implement human-robot interaction scenarios that complement physics-based simulation approaches.

## Core Concepts

### Why Unity is Used in Robotics

Unity has become increasingly popular in robotics for several key reasons:

**High-Quality Graphics**: Unity's rendering pipeline produces photorealistic visuals that help researchers visualize robot behaviors in realistic environments. This is crucial for testing perception algorithms and creating compelling demonstrations.

**Asset Ecosystem**: Unity's Asset Store provides thousands of pre-built models, environments, and tools that can accelerate robotics development. This includes furniture, buildings, robots, and specialized robotics tools.

**Cross-Platform Deployment**: Unity allows deployment to multiple platforms including desktop, mobile, VR, and AR, enabling diverse testing and interaction scenarios.

**User Interface Design**: Unity's UI system makes it easy to create intuitive interfaces for robot teleoperation, monitoring, and control.

**Real-Time Performance**: Unity is optimized for real-time rendering, making it suitable for interactive applications and real-time visualization of robot data.

### Graphics vs Physics Accuracy Trade-offs

When choosing between Unity and physics-focused simulators like Gazebo, consider these important trade-offs:

**Unity Strengths**:
- Superior visual quality and rendering
- Better user interface and interaction design capabilities
- Faster rendering of complex scenes
- More intuitive environment creation tools
- Better integration with VR/AR platforms

**Unity Limitations**:
- Less accurate physics simulation compared to dedicated physics engines
- More complex ROS integration requiring specialized middleware
- Higher computational requirements for high-quality graphics
- Steeper learning curve for robotics-specific applications

The choice between Unity and Gazebo often depends on the specific requirements of your application:
- Use Gazebo for physics-accurate simulation and control algorithm testing
- Use Unity for visualization, perception testing, and human-robot interaction
- Use both in combination for comprehensive digital twin environments

### ROS–Unity Communication Approaches

Connecting Unity with ROS 2 requires specialized middleware that bridges the two systems:

**Unity Robotics Hub**: Provides official packages for ROS-Unity integration, including:
- ROS TCP Connector: Enables communication between ROS and Unity via TCP/IP
- URDF Importer: Allows importing robot models directly from URDF files
- Robotics Demo Framework: Example implementations of common robotics patterns

**Message Translation**: Converting between ROS message types and Unity data structures, including:
- Transform synchronization between ROS TF tree and Unity coordinate system
- Sensor data translation from ROS sensor_msgs to Unity representations
- Control command translation from Unity to ROS joint commands

**Communication Patterns**: Implementing publisher-subscriber and service-client patterns between ROS and Unity systems.

### Human–Robot Interaction (HRI) in Unity

Unity excels at creating intuitive interfaces for human-robot interaction:

**Teleoperation Interfaces**: Creating user-friendly controls for remote robot operation, including:
- Joystick and keyboard interfaces
- VR/AR teleoperation systems
- Gesture-based control interfaces
- Touch-screen interfaces for mobile platforms

**Visualization Systems**: Displaying robot state, sensor data, and environmental information in intuitive ways:
- 3D visualization of sensor data (point clouds, camera feeds, etc.)
- Robot state displays (battery, joint angles, system status)
- Path planning visualization
- Collision detection and safety zones

**Interaction Design**: Creating natural ways for humans to interact with robots:
- Voice command interfaces
- Gesture recognition systems
- Social robot interaction scenarios
- Collaborative task interfaces

## Examples

### Example: Unity Scene with Robot Avatar

Creating a basic Unity scene with a robot avatar involves several key components:

```csharp
// RobotAvatarController.cs
using UnityEngine;
using System.Collections;

public class RobotAvatarController : MonoBehaviour
{
    [Header("Robot Configuration")]
    public string robotName = "HumanoidRobot";
    public float movementSpeed = 2.0f;
    public float rotationSpeed = 100.0f;

    [Header("Joint References")]
    public Transform head;
    public Transform leftArm;
    public Transform rightArm;
    public Transform leftLeg;
    public Transform rightLeg;

    [Header("ROS Connection")]
    public bool isConnected = false;
    public string rosEndpoint = "ws://localhost:9090";

    private Vector3 targetPosition;
    private Quaternion targetRotation;
    private bool useROSMovement = false;

    void Start()
    {
        targetPosition = transform.position;
        targetRotation = transform.rotation;
        InitializeROSConnection();
    }

    void Update()
    {
        if (useROSMovement)
        {
            // Move based on ROS commands
            UpdateFromROS();
        }
        else
        {
            // Move based on keyboard input for demonstration
            HandleKeyboardInput();
        }

        // Smoothly interpolate to target position and rotation
        transform.position = Vector3.Lerp(transform.position, targetPosition, Time.deltaTime * movementSpeed);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * rotationSpeed);
    }

    void HandleKeyboardInput()
    {
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");

        Vector3 movement = new Vector3(horizontal, 0, vertical) * movementSpeed * Time.deltaTime;
        targetPosition += transform.TransformDirection(movement);

        if (Input.GetKey(KeyCode.Q))
            targetRotation *= Quaternion.Euler(0, -rotationSpeed * Time.deltaTime, 0);
        if (Input.GetKey(KeyCode.E))
            targetRotation *= Quaternion.Euler(0, rotationSpeed * Time.deltaTime, 0);
    }

    void InitializeROSConnection()
    {
        // Placeholder for ROS connection initialization
        // In practice, you'd use Unity ROS TCP Connector or similar
        Debug.Log($"Connecting to ROS at {rosEndpoint}");
        isConnected = true;
    }

    void UpdateFromROS()
    {
        // This would be populated with actual ROS message data
        // For demonstration, we'll just keep the current position
    }

    // Methods for updating joint positions based on ROS joint state messages
    public void UpdateJointPositions(float[] jointPositions)
    {
        if (jointPositions.Length >= 5)
        {
            // Update joint rotations based on joint position array
            if (head != null)
                head.localRotation = Quaternion.Euler(0, 0, jointPositions[0] * Mathf.Rad2Deg);

            if (leftArm != null)
                leftArm.localRotation = Quaternion.Euler(jointPositions[1] * Mathf.Rad2Deg, 0, 0);

            if (rightArm != null)
                rightArm.localRotation = Quaternion.Euler(jointPositions[2] * Mathf.Rad2Deg, 0, 0);

            if (leftLeg != null)
                leftLeg.localRotation = Quaternion.Euler(jointPositions[3] * Mathf.Rad2Deg, 0, 0);

            if (rightLeg != null)
                rightLeg.localRotation = Quaternion.Euler(jointPositions[4] * Mathf.Rad2Deg, 0, 0);
        }
    }
}
```

### Example: Keyboard or Gesture-Based Robot Control

Implementing intuitive control interfaces in Unity:

```csharp
// HRIController.cs
using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class HRIController : MonoBehaviour
{
    [Header("Control Configuration")]
    public RobotAvatarController robot;
    public Text statusText;
    public Slider speedSlider;
    public Button[] actionButtons;

    [Header("Gesture Recognition")]
    public bool enableGestures = false;
    public Camera gestureCamera;

    void Start()
    {
        SetupUI();
        SetupActionButtons();
    }

    void SetupUI()
    {
        if (speedSlider != null)
        {
            speedSlider.minValue = 0.5f;
            speedSlider.maxValue = 5.0f;
            speedSlider.value = 2.0f;
            speedSlider.onValueChanged.AddListener(OnSpeedChanged);
        }
    }

    void SetupActionButtons()
    {
        if (actionButtons != null)
        {
            for (int i = 0; i < actionButtons.Length; i++)
            {
                int index = i; // Capture for closure
                actionButtons[i].onClick.AddListener(() => OnActionButtonClicked(index));
            }
        }
    }

    void OnSpeedChanged(float value)
    {
        if (robot != null)
            robot.movementSpeed = value;

        if (statusText != null)
            statusText.text = $"Speed: {value:F1}x";
    }

    void OnActionButtonClicked(int actionIndex)
    {
        switch (actionIndex)
        {
            case 0: // Wave gesture
                StartCoroutine(PerformWaveGesture());
                break;
            case 1: // Point gesture
                StartCoroutine(PerformPointGesture());
                break;
            case 2: // Come here gesture
                StartCoroutine(PerformComeGesture());
                break;
        }
    }

    IEnumerator PerformWaveGesture()
    {
        if (robot != null && robot.rightArm != null)
        {
            Quaternion originalRotation = robot.rightArm.localRotation;
            Quaternion targetRotation = Quaternion.Euler(30, 0, 0);

            for (float t = 0; t < 1; t += Time.deltaTime * 4)
            {
                robot.rightArm.localRotation = Quaternion.Slerp(originalRotation, targetRotation, t);
                yield return null;
            }

            yield return new WaitForSeconds(0.5f);

            for (float t = 0; t < 1; t += Time.deltaTime * 4)
            {
                robot.rightArm.localRotation = Quaternion.Slerp(targetRotation, originalRotation, t);
                yield return null;
            }
        }
    }

    IEnumerator PerformPointGesture()
    {
        if (robot != null && robot.rightArm != null)
        {
            Quaternion originalRotation = robot.rightArm.localRotation;
            Quaternion targetRotation = Quaternion.Euler(60, 30, 0);

            for (float t = 0; t < 1; t += Time.deltaTime * 3)
            {
                robot.rightArm.localRotation = Quaternion.Slerp(originalRotation, targetRotation, t);
                yield return null;
            }

            yield return new WaitForSeconds(1.0f);

            for (float t = 0; t < 1; t += Time.deltaTime * 3)
            {
                robot.rightArm.localRotation = Quaternion.Slerp(targetRotation, originalRotation, t);
                yield return null;
            }
        }
    }

    IEnumerator PerformComeGesture()
    {
        if (robot != null)
        {
            // Move robot toward a target position
            Vector3 startPosition = robot.transform.position;
            Vector3 targetPosition = startPosition + robot.transform.forward * 2.0f;

            for (float t = 0; t < 1; t += Time.deltaTime * 0.5f)
            {
                robot.targetPosition = Vector3.Lerp(startPosition, targetPosition, t);
                yield return null;
            }
        }
    }

    void Update()
    {
        if (enableGestures && gestureCamera != null)
        {
            HandleGestureInput();
        }
    }

    void HandleGestureInput()
    {
        // Simple gesture recognition based on mouse movement
        if (Input.GetMouseButtonDown(0))
        {
            StartCoroutine(CaptureGesture());
        }
    }

    IEnumerator CaptureGesture()
    {
        Vector3 gestureStart = Input.mousePosition;
        yield return new WaitForSeconds(0.1f);
        Vector3 gestureEnd = Input.mousePosition;

        Vector2 gestureVector = (gestureEnd - gestureStart);

        if (gestureVector.magnitude > 50) // Minimum gesture size
        {
            if (Mathf.Abs(gestureVector.x) > Mathf.Abs(gestureVector.y))
            {
                // Horizontal gesture - move left/right
                if (gestureVector.x > 0)
                    robot.targetPosition += robot.transform.right * 1.0f;
                else
                    robot.targetPosition -= robot.transform.right * 1.0f;
            }
            else
            {
                // Vertical gesture - move forward/back
                if (gestureVector.y > 0)
                    robot.targetPosition += robot.transform.forward * 1.0f;
                else
                    robot.targetPosition -= robot.transform.forward * 1.0f;
            }
        }
    }
}
```

### Example: Visualization-Only Digital Twin

Creating a visualization-only digital twin that displays real robot data:

```csharp
// VisualizationTwin.cs
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class VisualizationTwin : MonoBehaviour
{
    [Header("Twin Configuration")]
    public RobotAvatarController robotAvatar;
    public bool isLiveConnection = false;
    public float updateRate = 30.0f; // Hz

    [Header("Data Buffers")]
    public int maxBufferSize = 100;
    private Queue<Vector3> positionBuffer;
    private Queue<Quaternion> rotationBuffer;
    private Queue<float[]> jointStateBuffer;

    [Header("Visualization Settings")]
    public bool showTrajectory = true;
    public GameObject trajectoryPrefab;
    public Color trajectoryColor = Color.blue;
    public float trajectoryWidth = 0.1f;

    private LineRenderer trajectoryLine;
    private float lastUpdateTime;

    void Start()
    {
        positionBuffer = new Queue<Vector3>();
        rotationBuffer = new Queue<Quaternion>();
        jointStateBuffer = new Queue<float[]>();

        SetupTrajectoryVisualization();
        lastUpdateTime = Time.time;
    }

    void Update()
    {
        if (isLiveConnection && Time.time - lastUpdateTime > 1.0f / updateRate)
        {
            UpdateFromLiveData();
            lastUpdateTime = Time.time;
        }

        UpdateVisualization();
    }

    void SetupTrajectoryVisualization()
    {
        if (showTrajectory)
        {
            trajectoryLine = gameObject.AddComponent<LineRenderer>();
            trajectoryLine.material = new Material(Shader.Find("Sprites/Default"));
            trajectoryLine.color = trajectoryColor;
            trajectoryLine.startWidth = trajectoryWidth;
            trajectoryLine.endWidth = trajectoryWidth;
        }
    }

    void UpdateFromLiveData()
    {
        // In a real implementation, this would receive data from ROS
        // For demonstration, we'll simulate data updates
        SimulateLiveDataUpdate();
    }

    void SimulateLiveDataUpdate()
    {
        // Simulate receiving position, rotation, and joint state data
        Vector3 newPosition = robotAvatar.transform.position +
                             new Vector3(Random.Range(-0.1f, 0.1f),
                                        0,
                                        Random.Range(-0.1f, 0.1f));

        Quaternion newRotation = robotAvatar.transform.rotation *
                                Quaternion.Euler(0, Random.Range(-5f, 5f), 0);

        float[] jointStates = new float[6];
        for (int i = 0; i < jointStates.Length; i++)
        {
            jointStates[i] = Random.Range(-1.5f, 1.5f); // Simulated joint angles
        }

        // Add to buffers
        AddToBuffer(positionBuffer, newPosition);
        AddToBuffer(rotationBuffer, newRotation);
        AddToBuffer(jointStateBuffer, jointStates);

        // Update robot avatar with new data
        robotAvatar.targetPosition = newPosition;
        robotAvatar.targetRotation = newRotation;
        robotAvatar.UpdateJointPositions(jointStates);

        // Update trajectory visualization
        if (showTrajectory && trajectoryLine != null)
        {
            trajectoryLine.positionCount = positionBuffer.Count;
            trajectoryLine.SetPositions(positionBuffer.ToArray());
        }
    }

    void AddToBuffer<T>(Queue<T> buffer, T item)
    {
        buffer.Enqueue(item);
        if (buffer.Count > maxBufferSize)
            buffer.Dequeue();
    }

    void UpdateVisualization()
    {
        // Update any visualization elements based on current data
        // This could include updating graphs, status displays, etc.
    }

    // Method to manually update with live data from ROS
    public void UpdateWithRealData(Vector3 position, Quaternion rotation, float[] jointStates)
    {
        AddToBuffer(positionBuffer, position);
        AddToBuffer(rotationBuffer, rotation);
        AddToBuffer(jointStateBuffer, jointStates);

        robotAvatar.targetPosition = position;
        robotAvatar.targetRotation = rotation;
        robotAvatar.UpdateJointPositions(jointStates);

        if (showTrajectory && trajectoryLine != null)
        {
            trajectoryLine.positionCount = positionBuffer.Count;
            trajectoryLine.SetPositions(positionBuffer.ToArray());
        }
    }
}
```

## Summary & Key Takeaways

In this chapter, you've learned about Unity's role in robotics and human-robot interaction:

- **Unity's value in robotics** lies in its superior graphics, visualization capabilities, and user interface design tools
- **Graphics vs physics trade-offs** require choosing the right tool for specific application needs
- **ROS-Unity communication** involves specialized middleware and message translation systems
- **Human-robot interaction design** benefits from Unity's intuitive interface creation tools

You've seen practical examples of creating Unity scenes with robot avatars, implementing keyboard and gesture-based controls, and building visualization-only digital twins. Unity complements physics-focused simulators like Gazebo by providing advanced visualization and interaction capabilities that are essential for testing human-robot interaction scenarios and creating compelling demonstrations.

In the next chapter, we'll explore sim-to-real validation strategies that help ensure your simulation results translate effectively to real-world robot deployment.