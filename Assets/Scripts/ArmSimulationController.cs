using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace RoboArm
{
    /// <summary>
    /// Interactive & Autonomous Simulation Controller for the EB15 Robotic Arm.
    /// Fully compatible with both the New Input System and Legacy Input Manager.
    /// Provides On-Screen UI sliders and an automated pick-and-place trajectory demo.
    /// </summary>
    public class ArmSimulationController : MonoBehaviour
    {
        [Header("Simulation Mode")]
        [Tooltip("If enabled, the arm automatically animates through a sequence of waypoints.")]
        public bool autoDemo = true;
        [Tooltip("Speed multiplier for the trajectory demo.")]
        public float demoSpeed = 0.8f;

        [Header("Joint Drive Settings")]
        public float driveStiffness = 15000f;
        public float driveDamping = 300f;
        public float driveForceLimit = 1000f;

        [Header("Joint References")]
        public ArticulationBody joint1;        // Revolute (-180 to 180 deg)
        public ArticulationBody joint2;        // Revolute (-90 to 90 deg)
        public ArticulationBody joint3;        // Revolute (-154.7 to 154.7 deg)
        public ArticulationBody wristJoint;    // Revolute (-90 to 90 deg)
        public ArticulationBody gripperLeft;   // Prismatic (-0.016 to 0.012 m)
        public ArticulationBody gripperRight;  // Prismatic (-0.016 to 0.012 m)

        [Header("Current Target Angles (Deg / Meters)")]
        [Range(-180f, 180f)] public float targetJoint1 = 0f;
        [Range(-90f, 90f)] public float targetJoint2 = 0f;
        [Range(-154.7f, 154.7f)] public float targetJoint3 = 0f;
        [Range(-90f, 90f)] public float targetWrist = 0f;
        [Range(0f, 1f)] public float targetGripper = 0.5f; // 0 = closed, 1 = open

        public bool showUI = true;

        [System.Serializable]
        public struct Waypoint
        {
            public string name;
            public float j1, j2, j3, wrist, grip;
            public Waypoint(string name, float j1, float j2, float j3, float wrist, float grip)
            {
                this.name = name; this.j1 = j1; this.j2 = j2; this.j3 = j3; this.wrist = wrist; this.grip = grip;
            }
        }

        public Waypoint[] waypoints = new Waypoint[]
        {
            new Waypoint("Home (Upright)", 0f, 0f, 0f, 0f, 1f),
            new Waypoint("Reach Forward Right", 45f, -35f, 50f, 25f, 1f),
            new Waypoint("Pick Target", 45f, -55f, 75f, 35f, 0f),
            new Waypoint("Lift Object", 45f, -25f, 35f, 15f, 0f),
            new Waypoint("Swing to Left", -50f, -25f, 35f, -15f, 0f),
            new Waypoint("Place Target", -50f, -55f, 75f, -35f, 1f),
            new Waypoint("Retract", -50f, -10f, 20f, 0f, 1f),
            new Waypoint("Return Home", 0f, 0f, 0f, 0f, 0.5f)
        };

        private int currentWaypointIndex = 0;
        private float waypointProgress = 0f;

        void Awake()
        {
            Application.runInBackground = true;
            FindJoints();
            ConfigureJointDrives();

            // Anchor base link firmly to world
            var baseLink = transform.Find("world/base_link");
            if (baseLink != null)
            {
                var baseAb = baseLink.GetComponent<ArticulationBody>();
                if (baseAb != null) baseAb.immovable = true;
            }
        }

        public void FindJoints()
        {
            var bodies = GetComponentsInChildren<ArticulationBody>();
            foreach (var ab in bodies)
            {
                switch (ab.gameObject.name)
                {
                    case "link1": joint1 = ab; break;
                    case "link2": joint2 = ab; break;
                    case "link3": joint3 = ab; break;
                    case "wrist_link": wristJoint = ab; break;
                    case "gripper_left_finger_link": gripperLeft = ab; break;
                    case "gripper_right_finger_link": gripperRight = ab; break;
                }
            }
        }

        public void ConfigureJointDrives()
        {
            SetDrive(joint1, driveStiffness, driveDamping, driveForceLimit);
            SetDrive(joint2, driveStiffness, driveDamping, driveForceLimit);
            SetDrive(joint3, driveStiffness, driveDamping, driveForceLimit);
            SetDrive(wristJoint, driveStiffness, driveDamping, driveForceLimit);
            SetDrive(gripperLeft, driveStiffness, driveDamping, driveForceLimit);
            SetDrive(gripperRight, driveStiffness, driveDamping, driveForceLimit);
        }

        private void SetDrive(ArticulationBody ab, float stiffness, float damping, float forceLimit)
        {
            if (ab == null || ab.jointType == ArticulationJointType.FixedJoint) return;
            var drive = ab.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            ab.xDrive = drive;
        }

        void Update()
        {
            HandleKeyboardInput();

            if (autoDemo && waypoints != null && waypoints.Length > 1)
            {
                UpdateAutoDemo();
            }

            ApplyJointTargets();
        }

        void FixedUpdate()
        {
            ApplyJointTargets();
        }

        private void HandleKeyboardInput()
        {
#if ENABLE_INPUT_SYSTEM
            try
            {
                if (Keyboard.current != null)
                {
                    if (Keyboard.current.spaceKey.wasPressedThisFrame)
                        autoDemo = !autoDemo;
                    if (Keyboard.current.tabKey.wasPressedThisFrame)
                        showUI = !showUI;
                }
            }
            catch (System.Exception)
            {
                // Suppress any input exception so physics updates never fail
            }
#else
            try
            {
                if (Input.GetKeyDown(KeyCode.Space)) autoDemo = !autoDemo;
                if (Input.GetKeyDown(KeyCode.Tab)) showUI = !showUI;
            }
            catch (System.Exception)
            {
                // Suppress any input exception so physics updates never fail
            }
#endif
        }

        private void UpdateAutoDemo()
        {
            waypointProgress += Time.deltaTime * demoSpeed;
            if (waypointProgress >= 1f)
            {
                waypointProgress = 0f;
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
            }

            int nextIndex = (currentWaypointIndex + 1) % waypoints.Length;
            var from = waypoints[currentWaypointIndex];
            var to = waypoints[nextIndex];
            float t = Mathf.SmoothStep(0f, 1f, waypointProgress);

            targetJoint1 = Mathf.Lerp(from.j1, to.j1, t);
            targetJoint2 = Mathf.Lerp(from.j2, to.j2, t);
            targetJoint3 = Mathf.Lerp(from.j3, to.j3, t);
            targetWrist = Mathf.Lerp(from.wrist, to.wrist, t);
            targetGripper = Mathf.Lerp(from.grip, to.grip, t);
        }

        public void ApplyJointTargets()
        {
            targetJoint1 = Mathf.Clamp(targetJoint1, -180f, 180f);
            targetJoint2 = Mathf.Clamp(targetJoint2, -90f, 90f);
            targetJoint3 = Mathf.Clamp(targetJoint3, -154.7f, 154.7f);
            targetWrist = Mathf.Clamp(targetWrist, -90f, 90f);
            targetGripper = Mathf.Clamp01(targetGripper);

            SetTarget(joint1, targetJoint1);
            SetTarget(joint2, targetJoint2);
            SetTarget(joint3, targetJoint3);
            SetTarget(wristJoint, targetWrist);

            // Gripper: range is [-0.016m, 0.012m]
            float gripPos = Mathf.Lerp(-0.016f, 0.012f, targetGripper);
            SetTarget(gripperLeft, gripPos);
            SetTarget(gripperRight, gripPos);
        }

        private void SetTarget(ArticulationBody ab, float target)
        {
            if (ab == null || ab.jointType == ArticulationJointType.FixedJoint) return;
            var drive = ab.xDrive;
            drive.target = target;
            ab.xDrive = drive;
        }

        void OnGUI()
        {
            // Event-based hotkeys (immune to Input System restrictions)
            if (Event.current.type == EventType.KeyDown)
            {
                if (Event.current.keyCode == KeyCode.Space)
                {
                    autoDemo = !autoDemo;
                    Event.current.Use();
                }
                else if (Event.current.keyCode == KeyCode.Tab)
                {
                    showUI = !showUI;
                    Event.current.Use();
                }
            }

            if (!showUI)
            {
                if (GUI.Button(new Rect(15, 15, 120, 30), "Show Controls"))
                {
                    showUI = true;
                }
                return;
            }

            int panelWidth = 340;
            int panelHeight = 400;
            GUI.Box(new Rect(15, 15, panelWidth, panelHeight), "EB15 Robot Arm Simulation Control");

            GUILayout.BeginArea(new Rect(25, 42, panelWidth - 20, panelHeight - 35));

            // Auto Demo toggle
            bool prevDemo = autoDemo;
            autoDemo = GUILayout.Toggle(autoDemo, " Auto Trajectory Demo (Space to toggle)");
            if (autoDemo && waypoints != null && waypoints.Length > 0)
            {
                var wp = waypoints[currentWaypointIndex];
                GUILayout.Label($"Executing: <b>{wp.name}</b> ({currentWaypointIndex + 1}/{waypoints.Length})");
            }
            else
            {
                GUILayout.Label("<color=#88ccff>Interactive Mode - Drag Sliders Below:</color>");
            }

            GUILayout.Space(6);

            // Joint 1
            float actJ1 = joint1 != null ? joint1.jointPosition[0] * Mathf.Rad2Deg : 0f;
            GUILayout.Label($"Joint 1 (Base Yaw): {targetJoint1:F1}° [Act: {actJ1:F1}°]");
            float newJ1 = GUILayout.HorizontalSlider(targetJoint1, -180f, 180f);
            if (newJ1 != targetJoint1) { targetJoint1 = newJ1; autoDemo = false; ApplyJointTargets(); }

            // Joint 2
            float actJ2 = joint2 != null ? joint2.jointPosition[0] * Mathf.Rad2Deg : 0f;
            GUILayout.Label($"Joint 2 (Shoulder): {targetJoint2:F1}° [Act: {actJ2:F1}°]");
            float newJ2 = GUILayout.HorizontalSlider(targetJoint2, -90f, 90f);
            if (newJ2 != targetJoint2) { targetJoint2 = newJ2; autoDemo = false; ApplyJointTargets(); }

            // Joint 3
            float actJ3 = joint3 != null ? joint3.jointPosition[0] * Mathf.Rad2Deg : 0f;
            GUILayout.Label($"Joint 3 (Elbow): {targetJoint3:F1}° [Act: {actJ3:F1}°]");
            float newJ3 = GUILayout.HorizontalSlider(targetJoint3, -154.7f, 154.7f);
            if (newJ3 != targetJoint3) { targetJoint3 = newJ3; autoDemo = false; ApplyJointTargets(); }

            // Wrist
            float actWrist = wristJoint != null ? wristJoint.jointPosition[0] * Mathf.Rad2Deg : 0f;
            GUILayout.Label($"Wrist (Roll): {targetWrist:F1}° [Act: {actWrist:F1}°]");
            float newWrist = GUILayout.HorizontalSlider(targetWrist, -90f, 90f);
            if (newWrist != targetWrist) { targetWrist = newWrist; autoDemo = false; ApplyJointTargets(); }

            // Gripper
            GUILayout.Label($"Gripper: {(targetGripper < 0.2f ? "Closed" : targetGripper > 0.8f ? "Open" : $"{targetGripper * 100:F0}%")}");
            float newGrip = GUILayout.HorizontalSlider(targetGripper, 0f, 1f);
            if (newGrip != targetGripper) { targetGripper = newGrip; autoDemo = false; ApplyJointTargets(); }

            GUILayout.Space(8);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Reset to Home"))
            {
                targetJoint1 = 0f;
                targetJoint2 = 0f;
                targetJoint3 = 0f;
                targetWrist = 0f;
                targetGripper = 0.5f;
                autoDemo = false;
                ApplyJointTargets();
            }
            if (GUILayout.Button("Hide [Tab]"))
            {
                showUI = false;
            }
            GUILayout.EndHorizontal();

            GUILayout.EndArea();
        }
    }
}
