using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace RoboArm
{
    /// <summary>
    /// Master Digital Twin & Interactive Controller for the EB15 Robotic Arm.
    /// Manages both the Solid Arm (Real-world Physical Twin) and the Ghost Arm (Commanded Target Preview).
    /// Enforces identical physical velocity limits between Unity and the real robot.
    /// </summary>
    public class ArmSimulationController : MonoBehaviour
    {
        [Header("Physical Twin Joints (Solid Arm)")]
        public ArticulationBody joint1;        // Revolute (-180 to 180 deg)
        public ArticulationBody joint2;        // Revolute (-90 to 90 deg)
        public ArticulationBody joint3;        // Revolute (-154.7 to 154.7 deg)
        public ArticulationBody wristJoint;    // Revolute (-90 to 90 deg)
        public ArticulationBody gripperLeft;   // Prismatic (-0.016 to 0.012 m)
        public ArticulationBody gripperRight;  // Prismatic (-0.016 to 0.012 m)

        [Header("Ghost Preview Reference")]
        public GhostArmPreview ghostArm;

        [Header("Commanded Target Angles (Ghost Preview & Real Hardware Goal)")]
        [Range(-180f, 180f)] public float targetJoint1 = 0f;
        [Range(-90f, 90f)] public float targetJoint2 = 0f;
        [Range(-154.7f, 154.7f)] public float targetJoint3 = 0f;
        [Range(-90f, 90f)] public float targetWrist = 0f;
        [Range(0f, 1f)] public float targetGripper = 0.5f; // 0 = closed, 1 = open

        [Header("Physical Arm Measured Angles (Solid Digital Twin)")]
        public float currentJoint1 = 0f;
        public float currentJoint2 = 0f;
        public float currentJoint3 = 0f;
        public float currentWrist = 0f;
        public float currentGripper = 0.5f;

        [Header("Real-World Physical Speed Limits (deg/sec & m/sec)")]
        [Tooltip("Max joint 1 speed: 1.0 rad/s = 57.3 deg/s (matches physical stepper limit)")]
        public float maxSpeedJoint1 = 57.3f;
        [Tooltip("Max joint 2 speed: 0.8 rad/s = 45.8 deg/s (matches shoulder gear limit)")]
        public float maxSpeedJoint2 = 45.8f;
        [Tooltip("Max joint 3 speed: 1.0 rad/s = 57.3 deg/s")]
        public float maxSpeedJoint3 = 57.3f;
        [Tooltip("Max wrist speed: 2.0 rad/s = 114.6 deg/s")]
        public float maxSpeedWrist = 114.6f;
        [Tooltip("Max gripper speed (normalized units / sec)")]
        public float maxSpeedGripper = 1.78f;

        [Header("Drive Dynamics")]
        public float driveStiffness = 20000f;
        public float driveDamping = 350f;
        public float driveForceLimit = 1500f;

        [Header("Operation Mode")]
        public bool autoDemo = false;
        public float demoSpeed = 0.6f;
        public bool showUI = true;

        [Header("Hardware Encoder Status")]
        public bool isHardwareEncoderLive = false;
        private float lastEncoderReceiveTime = -10f;

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
        private RosRoboArmBridge rosBridge;

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

            if (ghostArm == null)
            {
                ghostArm = FindFirstObjectByType<GhostArmPreview>();
            }

            rosBridge = GetComponent<RosRoboArmBridge>();
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

            // Encoder feedback timeout check (1.2 seconds)
            isHardwareEncoderLive = (Time.time - lastEncoderReceiveTime) < 1.2f;

            if (autoDemo && waypoints != null && waypoints.Length > 1)
            {
                UpdateAutoDemo();
            }

            // Update Ghost Arm target preview immediately
            if (ghostArm != null)
            {
                ghostArm.SetPose(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }

            // Update Solid Arm (Physical Twin)
            if (!isHardwareEncoderLive)
            {
                // Velocity-matched simulation: move current positions towards target positions
                // at the EXACT real-world hardware stepper/servo speeds
                float dt = Time.deltaTime;
                currentJoint1 = Mathf.MoveTowards(currentJoint1, targetJoint1, maxSpeedJoint1 * dt);
                currentJoint2 = Mathf.MoveTowards(currentJoint2, targetJoint2, maxSpeedJoint2 * dt);
                currentJoint3 = Mathf.MoveTowards(currentJoint3, targetJoint3, maxSpeedJoint3 * dt);
                currentWrist = Mathf.MoveTowards(currentWrist, targetWrist, maxSpeedWrist * dt);
                currentGripper = Mathf.MoveTowards(currentGripper, targetGripper, maxSpeedGripper * dt);
            }

            ApplyCurrentJointsToPhysicalTwin();
        }

        void FixedUpdate()
        {
            ApplyCurrentJointsToPhysicalTwin();
        }

        /// <summary>
        /// Called by RosRoboArmBridge when real Arduino hardware encoder data is received over UDP.
        /// </summary>
        public void OnHardwareEncoderReceived(float j1Deg, float j2Deg, float j3Deg, float wristDeg, float gripNorm)
        {
            currentJoint1 = Mathf.Clamp(j1Deg, -180f, 180f);
            currentJoint2 = Mathf.Clamp(j2Deg, -90f, 90f);
            currentJoint3 = Mathf.Clamp(j3Deg, -154.7f, 154.7f);
            currentWrist = Mathf.Clamp(wristDeg, -90f, 90f);
            currentGripper = Mathf.Clamp01(gripNorm);

            lastEncoderReceiveTime = Time.time;
            isHardwareEncoderLive = true;
        }

        /// <summary>
        /// Snaps the ghost sliders to the physical arm's current position.
        /// </summary>
        public void SnapGhostToPhysicalPose()
        {
            targetJoint1 = currentJoint1;
            targetJoint2 = currentJoint2;
            targetJoint3 = currentJoint3;
            targetWrist = currentWrist;
            targetGripper = currentGripper;

            if (ghostArm != null)
            {
                ghostArm.SetPose(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }
        }

        private void ApplyCurrentJointsToPhysicalTwin()
        {
            SetTarget(joint1, currentJoint1);
            SetTarget(joint2, currentJoint2);
            SetTarget(joint3, currentJoint3);
            SetTarget(wristJoint, currentWrist);

            float gripM = Mathf.Lerp(-0.016f, 0.012f, currentGripper);
            SetTarget(gripperLeft, gripM);
            SetTarget(gripperRight, gripM);
        }

        private void SetTarget(ArticulationBody ab, float target)
        {
            if (ab == null || ab.jointType == ArticulationJointType.FixedJoint) return;
            var drive = ab.xDrive;
            drive.target = target;
            ab.xDrive = drive;
        }

        private void HandleKeyboardInput()
        {
#if ENABLE_INPUT_SYSTEM
            try
            {
                if (Keyboard.current != null)
                {
                    if (Keyboard.current.spaceKey.wasPressedThisFrame) autoDemo = !autoDemo;
                    if (Keyboard.current.tabKey.wasPressedThisFrame) showUI = !showUI;
                    if (Keyboard.current.gKey.wasPressedThisFrame && ghostArm != null)
                        ghostArm.SetVisible(!ghostArm.isVisible);
                }
            }
            catch (System.Exception) { }
#else
            try
            {
                if (Input.GetKeyDown(KeyCode.Space)) autoDemo = !autoDemo;
                if (Input.GetKeyDown(KeyCode.Tab)) showUI = !showUI;
                if (Input.GetKeyDown(KeyCode.G) && ghostArm != null)
                    ghostArm.SetVisible(!ghostArm.isVisible);
            }
            catch (System.Exception) { }
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

            if (rosBridge != null && rosBridge.streamCommandsToRos)
            {
                rosBridge.SendTargetPoseToRos(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }
        }

        void OnGUI()
        {
            if (Event.current.type == EventType.KeyDown)
            {
                if (Event.current.keyCode == KeyCode.Space) { autoDemo = !autoDemo; Event.current.Use(); }
                else if (Event.current.keyCode == KeyCode.Tab) { showUI = !showUI; Event.current.Use(); }
                else if (Event.current.keyCode == KeyCode.G && ghostArm != null)
                {
                    ghostArm.SetVisible(!ghostArm.isVisible);
                    Event.current.Use();
                }
            }

            if (!showUI)
            {
                if (GUI.Button(new Rect(15, 15, 120, 30), "Show Controls")) showUI = true;
                return;
            }

            int panelWidth = 360;
            int panelHeight = 470;
            GUI.Box(new Rect(15, 15, panelWidth, panelHeight), "EB15 Digital Twin & Ghost Preview");

            GUILayout.BeginArea(new Rect(25, 42, panelWidth - 20, panelHeight - 35));

            // Status badge
            Color oldCol = GUI.color;
            if (isHardwareEncoderLive)
            {
                GUI.color = Color.green;
                GUILayout.Label("• TWIN MODE: PHYSICAL HARDWARE ENCODER FEEDBACK");
            }
            else
            {
                GUI.color = new Color(0.2f, 0.85f, 1.0f);
                GUILayout.Label("• TWIN MODE: PHYSICAL VELOCITY-MATCHED SIMULATION");
            }
            GUI.color = oldCol;

            GUILayout.Space(4);

            // Ghost Preview controls
            GUILayout.BeginHorizontal();
            if (ghostArm != null)
            {
                bool ghostVis = GUILayout.Toggle(ghostArm.isVisible, " Ghost Preview (G)");
                if (ghostVis != ghostArm.isVisible) ghostArm.SetVisible(ghostVis);
            }
            if (GUILayout.Button("Snap Ghost to Arm", GUILayout.Width(130)))
            {
                SnapGhostToPhysicalPose();
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(6);
            GUILayout.Label("Ghost Target Sliders (Instant Goal):");

            // Slider 1
            float prevJ1 = targetJoint1;
            GUILayout.Label($"Joint 1 (Base): {targetJoint1:F1}° (Real: {currentJoint1:F1}°)");
            targetJoint1 = GUILayout.HorizontalSlider(targetJoint1, -180f, 180f);

            // Slider 2
            float prevJ2 = targetJoint2;
            GUILayout.Label($"Joint 2 (Shoulder): {targetJoint2:F1}° (Real: {currentJoint2:F1}°)");
            targetJoint2 = GUILayout.HorizontalSlider(targetJoint2, -90f, 90f);

            // Slider 3
            float prevJ3 = targetJoint3;
            GUILayout.Label($"Joint 3 (Elbow): {targetJoint3:F1}° (Real: {currentJoint3:F1}°)");
            targetJoint3 = GUILayout.HorizontalSlider(targetJoint3, -154.7f, 154.7f);

            // Slider Wrist
            float prevWrist = targetWrist;
            GUILayout.Label($"Wrist: {targetWrist:F1}° (Real: {currentWrist:F1}°)");
            targetWrist = GUILayout.HorizontalSlider(targetWrist, -90f, 90f);

            // Slider Gripper
            float prevGrip = targetGripper;
            GUILayout.Label($"Gripper: {(targetGripper * 100f):F0}% (Real: {(currentGripper * 100f):F0}%)");
            targetGripper = GUILayout.HorizontalSlider(targetGripper, 0f, 1f);

            // Send to ROS when slider changes or on button press
            bool slidersChanged = Mathf.Abs(targetJoint1 - prevJ1) > 0.01f ||
                                  Mathf.Abs(targetJoint2 - prevJ2) > 0.01f ||
                                  Mathf.Abs(targetJoint3 - prevJ3) > 0.01f ||
                                  Mathf.Abs(targetWrist - prevWrist) > 0.01f ||
                                  Mathf.Abs(targetGripper - prevGrip) > 0.005f;

            if (slidersChanged && rosBridge != null && rosBridge.streamCommandsToRos)
            {
                rosBridge.SendTargetPoseToRos(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }

            GUILayout.Space(8);

            GUILayout.BeginHorizontal();
            if (GUILayout.Button(autoDemo ? "Stop Demo (Space)" : "Run Waypoint Demo"))
            {
                autoDemo = !autoDemo;
            }
            if (rosBridge != null && GUILayout.Button("Send Goal to Robot"))
            {
                rosBridge.SendTargetPoseToRos(targetJoint1, targetJoint2, targetJoint3, targetWrist, targetGripper);
            }
            GUILayout.EndHorizontal();

            // Real-time tracking delta
            float lag = Mathf.Abs(targetJoint1 - currentJoint1) +
                        Mathf.Abs(targetJoint2 - currentJoint2) +
                        Mathf.Abs(targetJoint3 - currentJoint3) +
                        Mathf.Abs(targetWrist - currentWrist);
            GUILayout.Label($"Goal Lag (Total Error): {lag:F1}°");

            GUILayout.EndArea();
        }
    }
}
