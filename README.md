# EB15 Robotic Arm - Unity Digital Twin

[![Unity Version](https://img.shields.io/badge/Unity-6000.5.4f1%20(Unity%206)-blue.svg?logo=unity)](https://unity.com/)
[![Render Pipeline](https://img.shields.io/badge/Render%20Pipeline-URP%2017.5-orange.svg)](https://unity.com/srp/universal-render-pipeline)
[![Physics](https://img.shields.io/badge/Physics-PhysX%20ArticulationBody-green.svg)](https://docs.unity3d.com/Manual/class-ArticulationBody.html)
[![URDF](https://img.shields.io/badge/ROS%20URDF-Supported-red.svg)](https://github.com/Unity-Technologies/URDF-Importer)
[![Git LFS](https://img.shields.io/badge/Git%20LFS-Tracked-brightgreen.svg)](https://git-lfs.github.com/)

A high-fidelity, physics-driven **Digital Twin** of the **EB15 5-DOF + Parallel Gripper Robotic Arm** built in Unity. This project bridges physical robotics with virtual simulation, supporting real-time kinematic articulation, joint limit enforcement, interactive slider teleoperation, autonomous pick-and-place trajectory sequencing, and compatibility with the new Unity Input System and OpenXR VR setups.

---

## 📷 Preview

![EB15 Robotic Arm Digital Twin in Unity](Documentation/robot_preview.png)

---

## ✨ Features

* **PhysX ArticulationBody Kinematics**: Accurately simulates closed-loop and multi-body joint dynamics without drift or jitter.
* **Full URDF Hierarchy (11 Joints)**: Converted directly from ROS/ROS2 xacro files with calibrated joint axes, angular limits, and effort ratings.
* **31 High-Resolution Mesh Visuals**: Integrated STL models rendered using Universal Render Pipeline (URP) Lit shaders with physically accurate metallic and smoothness profiles.
* **12 Precision Collision Primitives**: Cylinders, boxes, and spheres mapped to links for fast, accurate collision and self-collision detection.
* **Interactive & Autonomous Controller (`ArmSimulationController.cs`)**:
  * **Interactive Mode**: Real-time on-screen UI sliders for teleoperating each joint within physical safety limits.
  * **Autonomous Demo**: Smooth waypoint interpolation simulating a complete pick-and-place routine.
  * **Input System Ready**: Fully compatible with Unity's new Input System and OpenXR environments.
* **Git LFS Integrated**: All `.stl` binary 3D assets tracked via Git Large File Storage for efficient repository cloning.

---

## 🦾 Joint Specifications & Kinematics

| Joint Name | Type | Axis | Physical Limit Range | Max Effort | Child Link | Description |
| :--- | :---: | :---: | :---: | :---: | :--- | :--- |
| `world_joint` | Fixed | — | — | — | `base_link` | Anchors robot base to the world |
| `joint1` | Revolute | $Z$ | **-180.0° to +180.0°** | 15.0 N·m | `link1` | Base yaw / panning |
| `joint2` | Revolute | $Y$ | **-90.0° to +90.0°** | 15.0 N·m | `link2` | Shoulder pitch |
| `joint3` | Revolute | $Y$ | **-154.7° to +154.7°** | 12.0 N·m | `link3` | Elbow pitch |
| `wrist_base_joint` | Fixed | — | — | — | `wrist_base_link` | Structural mounting fixture |
| `wrist_joint` | Revolute | $Z$ | **-90.0° to +90.0°** | 5.0 N·m | `wrist_link` | Wrist roll / orientation |
| `tool_joint` | Fixed | — | — | — | `tool0` | Tool attachment frame |
| `gripper_base_joint`| Fixed | — | — | — | `gripper_base_link` | End-effector chassis |
| `gripper_left_joint`| Prismatic | $+X$ | **-16.0 mm to +12.0 mm**| 20.0 N | `gripper_left_finger_link` | Left gripping finger |
| `gripper_right_joint`| Prismatic | $-X$ | **-16.0 mm to +12.0 mm**| 20.0 N | `gripper_right_finger_link`| Right gripping finger (mimic) |
| `gripper_tcp_joint` | Fixed | — | — | — | `gripper_tcp` | Tool Center Point indicator |

---

## 🎮 Simulation Controls

When running in Play Mode, the on-screen control panel allows immediate operation:

| Input / Action | Function |
| :--- | :--- |
| **`Space`** | Toggle between **Autonomous Trajectory Demo** and **Interactive Manual Mode** |
| **`Tab`** | Show / Hide the on-screen UI control panel |
| **Joint Sliders** | Teleoperate Base Yaw, Shoulder, Elbow, Wrist, or Gripper in real time |
| **Reset to Home** | Smoothly returns all joints to their upright home position ($0^\circ$) |

---

## 🚀 Getting Started

### Prerequisites
* **Unity Version**: Unity 6 (`6000.5.4f1`) or newer.
* **Render Pipeline**: Universal Render Pipeline (URP).
* **Git LFS**: Installed on your system (`git lfs install`).

### Installation & Setup

1. **Clone the repository with LFS**:
   ```bash
   git lfs install
   git clone https://github.com/StalinoAJ/RoboArm-DigitalTwin.git
   cd RoboArm-DigitalTwin
   ```

2. **Open in Unity**:
   * Launch **Unity Hub**.
   * Click **Add** → **Add project from disk** and select the repository root folder.
   * Open the project with **Unity 6000.5.4f1** (or compatible Unity 6 release).

3. **Run the Simulation**:
   * Open [`Assets/Scenes/SampleScene.unity`](Assets/Scenes/SampleScene.unity).
   * Press the ▶ **Play** button at the top of the Unity Editor.
   * Observe the arm executing the waypoint routine, or uncheck the demo to move the sliders manually!

---

## 📁 Project Structure

```
RoboArm-DigitalTwin/
├── Assets/
│   ├── eb15.prefab                 # Complete pre-configured robot arm prefab
│   ├── eb15.urdf                   # Master URDF definition file
│   ├── eb15_description/           # ROS package source files (meshes, URDF, launch)
│   │   ├── meshes/                 # STL 3D geometry & Unity mesh assets
│   │   └── urdf/                   # Modular xacro components
│   ├── Materials/                  # URP Lit materials (dark_gray, metallic, light_white, etc.)
│   ├── Scenes/
│   │   └── SampleScene.unity       # Active simulation scene with grounded floor & lighting
│   └── Scripts/
│       └── ArmSimulationController.cs  # Physics drive sequencer & UI teleoperation
├── Documentation/                  # Images and architectural diagrams
├── Packages/
│   └── manifest.json               # Package dependencies (URDF Importer, URP, Input System)
├── ProjectSettings/                # Unity project, physics, and quality configuration
├── .gitattributes                  # Git LFS tracking for 3D binary assets
└── .gitignore                      # Standard Unity ignore rules
```

---

## 🛠️ Programmatic C# API

To command the robot arm from external AI agents, state machines, or network receivers:

```csharp
using UnityEngine;
using RoboArm;

public class CustomRobotAgent : MonoBehaviour
{
    public ArmSimulationController arm;

    void Update()
    {
        // Disable automatic demo to take direct control
        arm.autoDemo = false;

        // Command target joint angles (in degrees)
        arm.targetJoint1 = 45.0f;   // Base Yaw (-180° to +180°)
        arm.targetJoint2 = -30.0f;  // Shoulder (-90° to +90°)
        arm.targetJoint3 = 60.0f;   // Elbow (-154.7° to +154.7°)
        arm.targetWrist  = 30.0f;   // Wrist (-90° to +90°)

        // Command gripper (0.0 = fully closed, 1.0 = fully open)
        arm.targetGripper = 1.0f;
    }
}
```

---

## 🔮 Future Roadmap & Applications

- [ ] **VR Teleoperation (Meta Quest / OpenXR)**: Natural 1:1 hand tracking and pinch-to-grip control using XR Interaction Toolkit.
- [ ] **Inverse Kinematics (IK)**: Drag-and-drop 3D end-effector gizmo using Unity Animation Rigging or BioIK.
- [ ] **Sim-to-Real Reinforcement Learning**: Train autonomous pick-and-place policies using **Unity ML-Agents** with domain randomization.
- [ ] **ROS 2 & MoveIt 2 Integration**: Connect via `ROS-TCP-Connector` to execute motion planning pipelines from ROS 2 Humble/Iron.
- [ ] **Bidirectional Telemetry**: Serial/WebSocket link to mirror encoder feedback and torque data from the physical EB15 hardware.

---

## 📄 License & Credits

* Robot CAD and URDF specifications based on the **EB15** robotics platform.
* Unity URDF Importer by [Unity-Technologies](https://github.com/Unity-Technologies/URDF-Importer).
